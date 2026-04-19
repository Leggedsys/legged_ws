"""
motor_bus_node

Manages all motors on a single RS485 serial bus. Cycles through every
assigned joint sequentially in each tick — only one process ever holds
the serial port, eliminating bus collisions.

Two instances are started: motor_bus_front (FR/FL) and motor_bus_rear (RR/RL).

Gain tuning at runtime without restart:
  ros2 param set /motor_bus_front kp 5.0
  ros2 param set /motor_bus_front kd 0.3

Graceful stop (Ctrl+C / any node shutdown):
  When /joint_commands stops arriving, the bus holds the last target for
  _ESTOP_HOLD seconds, then linearly fades kp to 0 over _ESTOP_FADE seconds.
  kd is kept throughout so the descent is damped, not a free-fall.
  The robot settles gently rather than collapsing instantly.
"""

import os
import statistics
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState


_ESTOP_HOLD = 0.5  # seconds to hold last target after commands stop
_ESTOP_FADE = 2.0  # seconds to fade kp from full to 0 after hold period


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' -> 'fr/hip'"""
    leg, joint = name.split("_", 1)
    return f"{leg.lower()}/{joint.lower()}"


def _filter_joints(all_joints: list, joint_names: list) -> list:
    """Return joint dicts from all_joints whose 'name' is in joint_names,
    preserving the order they appear in all_joints."""
    name_set = set(joint_names)
    return [j for j in all_joints if j["name"] in name_set]


class MotorBusNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_bus_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("joint_names", [""])
        self.declare_parameter("kp", 20.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("loop_hz", 1000.0)

        # Deferred import: load_sdk requires the compiled SDK shared library
        # and is not needed for unit-testing pure helper functions.
        from unitree_motor_ros2.sdk_loader import load_sdk  # noqa: PLC0415

        joint_names = self.get_parameter("joint_names").value
        cfg = self._load_config()
        joints = _filter_joints(cfg["joints"], joint_names)
        if not joints:
            raise RuntimeError(
                f"motor_bus_node: no joints found for names {joint_names}"
            )

        sdk = load_sdk()
        self._sdk = sdk
        _sdk_ratio = sdk.queryGearRatio(sdk.MotorType.GO_M8010_6)
        serial_port = self.get_parameter("serial_port").value
        self._serial = sdk.SerialPort(serial_port)

        self._names = [j["name"] for j in joints]
        # Latest position command received from /joint_commands (motor frame).
        self._targets = {name: 0.0 for name in [j["name"] for j in joints]}
        self._cmd_time: float | None = None  # monotonic time of last /joint_commands
        self._estop_logged: bool = False
        self._estop_done_logged: bool = False
        self._gear_ratios = {
            j["name"]: float(j.get("gear_ratio", _sdk_ratio)) for j in joints
        }

        global_kp = float(self.get_parameter("kp").value)
        global_kd = float(self.get_parameter("kd").value)
        self._global_kp_init = global_kp
        self._global_kd_init = global_kd
        control_cfg = cfg.get("control", {})
        calf_kp = float(control_cfg["kp_calf"]) if "kp_calf" in control_cfg else global_kp
        calf_kd = float(control_cfg["kd_calf"]) if "kd_calf" in control_cfg else global_kd
        # Declare per-joint kp/kd parameters (runtime-tunable via ros2 param set).
        # Priority: per-joint kp/kd in joint config > group (kp_calf) > global.
        for j in joints:
            name = j["name"]
            is_calf = name.endswith("_calf")
            group_kp = calf_kp if is_calf else global_kp
            group_kd = calf_kd if is_calf else global_kd
            self.declare_parameter(f"kp_{name}", float(j["kp"]) if "kp" in j else group_kp)
            self.declare_parameter(f"kd_{name}", float(j["kd"]) if "kd" in j else group_kd)

        self._cmds = []
        self._datas = []
        for j in joints:
            cmd = sdk.MotorCmd()
            cmd.motorType = sdk.MotorType.GO_M8010_6
            cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
            cmd.id = j["motor_id"]
            cmd.q = float(j["default_q"])
            cmd.dq = 0.0
            cmd.tau = 0.0
            self._cmds.append(cmd)

            data = sdk.MotorData()
            data.motorType = sdk.MotorType.GO_M8010_6
            self._datas.append(data)

        self._pubs = [
            self.create_publisher(
                JointState,
                f"/{_ns_from_joint_name(name)}/joint_states",
                10,
            )
            for name in self._names
        ]

        self._offsets = self._calibrate_offsets()
        # Last known-good position per joint; initialised to zero (power-on pose)
        self._last_pos = {name: 0.0 for name in self._names}

        self.create_subscription(JointState, "/joint_commands", self._on_joint_cmd, 10)
        self.add_on_set_parameters_callback(self._on_gains_changed)

        loop_hz = self.get_parameter("loop_hz").value
        self.create_timer(1.0 / loop_hz, self._tick)

        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        self.get_logger().info(
            f"Motor bus ready — {len(joints)} joints on {serial_port}  kp={kp}  kd={kd}"
        )

    def _calibrate_offsets(self, n_samples: int = 50) -> dict:
        """Sample current positions at power-on and use them as zero reference.

        Sends passive commands (kp=kd=0) and collects n_samples readings per
        joint. Uses the median to reject occasional garbage frames. All
        subsequent position readings and commands are relative to this offset.
        """
        sdk = self._sdk
        samples: dict = {name: [] for name in self._names}

        self.get_logger().info("Calibrating zero offsets — keep robot still...")
        # Warm up: send passive commands for 1 s so motor firmware fully initialises
        # before we sample. Without this, early frames may return garbage positions.
        for _ in range(100):
            for cmd, data, name in zip(self._cmds, self._datas, self._names):
                data.motorType = sdk.MotorType.GO_M8010_6
                cmd.motorType = sdk.MotorType.GO_M8010_6
                cmd.mode = sdk.queryMotorMode(
                    sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC
                )
                cmd.kp = 0.0
                cmd.kd = 0.0
                cmd.q = 0.0
                cmd.dq = 0.0
                cmd.tau = 0.0
                self._serial.sendRecv(cmd, data)
            time.sleep(0.01)
        for _ in range(n_samples):
            for cmd, data, name in zip(self._cmds, self._datas, self._names):
                data.motorType = sdk.MotorType.GO_M8010_6
                cmd.motorType = sdk.MotorType.GO_M8010_6
                cmd.mode = sdk.queryMotorMode(
                    sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC
                )
                cmd.kp = 0.0
                cmd.kd = 0.0
                cmd.q = 0.0
                cmd.dq = 0.0
                cmd.tau = 0.0
                self._serial.sendRecv(cmd, data)
                samples[name].append(float(data.q) / self._gear_ratios[name])
            time.sleep(0.01)

        offsets = {name: statistics.median(samples[name]) for name in self._names}
        for name, off in offsets.items():
            self.get_logger().info(f"  {name}: offset={off:.4f} rad")
        return offsets

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_joint_cmd(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._targets:
                self._targets[name] = float(pos)
        self._cmd_time = time.monotonic()
        self._estop_logged = False
        self._estop_done_logged = False

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        for p in params:
            if p.name in ("kp", "kd"):
                self.get_logger().info(f"Gain updated: {p.name}={p.value}")
        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        now = time.monotonic()
        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        sdk = self._sdk

        # Graceful stop: fade kp to 0 when commands have been absent too long.
        # kd is preserved so the descent is damped, not a free-fall.
        if self._cmd_time is None:
            effective_kp = kp
            effective_kd = kd
        else:
            cmd_age = now - self._cmd_time
            if cmd_age <= _ESTOP_HOLD:
                effective_kp = kp
                effective_kd = kd
            else:
                fade = min((cmd_age - _ESTOP_HOLD) / _ESTOP_FADE, 1.0)
                effective_kp = kp * (1.0 - fade)
                effective_kd = kd
                if not self._estop_logged:
                    self.get_logger().warn(
                        f"[estop] /joint_commands lost — fading kp to 0 over "
                        f"{_ESTOP_FADE:.1f}s"
                    )
                    self._estop_logged = True
                if fade >= 1.0 and not self._estop_done_logged:
                    self.get_logger().warn("[estop] kp=0 — motors passive")
                    self._estop_done_logged = True

        for cmd, data, pub, name in zip(
            self._cmds, self._datas, self._pubs, self._names
        ):
            gr = self._gear_ratios[name]
            # Re-set motorType/mode every tick — sendRecv may overwrite them
            data.motorType = sdk.MotorType.GO_M8010_6
            cmd.motorType = sdk.MotorType.GO_M8010_6
            cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
            offset = self._offsets[name]
            ratio = effective_kp / self._global_kp_init if self._global_kp_init > 0 else 0.0
            cmd.kp = ratio * float(self.get_parameter(f"kp_{name}").value)
            ratio_kd = effective_kd / self._global_kd_init if self._global_kd_init > 0 else 0.0
            cmd.kd = ratio_kd * float(self.get_parameter(f"kd_{name}").value)
            cmd.q = (self._targets[name] + offset) * gr
            cmd.dq = 0.0
            cmd.tau = 0.0
            self._serial.sendRecv(cmd, data)

            pos = float(data.q) / gr - offset
            # Reject single-frame spikes: clamp to ±1.0 rad change per tick
            if abs(pos - self._last_pos[name]) < 1.0:
                self._last_pos[name] = pos
            else:
                pos = self._last_pos[name]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [name]
            msg.position = [pos]
            msg.velocity = [float(data.dq) / gr]
            msg.effort = [float(data.tau)]
            pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotorBusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
