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
from std_msgs.msg import Float32MultiArray


_ESTOP_HOLD = 0.5  # seconds to hold last target after commands stop
_ESTOP_FADE = 2.0  # seconds to fade kp from full to 0 after hold period
_GAINS_TIMEOUT = 0.2  # s — revert to default kp/kd if /joint_gains goes stale
_EXTRAPOLATE_CAP = 0.05  # s — max lookahead for velocity-based target extrapolation
                          # between /joint_commands updates. Without this, the bus
                          # holds the last received q as a flat step (ZOH) for the
                          # whole inter-command gap, which looks like discrete
                          # "jerky" jumps at low controller rates. Capped well
                          # below _ESTOP_HOLD so a stalled command stream still
                          # falls back to the plain hold, not runaway extrapolation.

_HEALTH_PERIOD = 10.0  # s — interval between bus-health log lines
_HEALTH_WARN_PCT = 70  # warn when any motor's reply rate drops below this

_YAML_JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' -> 'fr/hip'"""
    leg, joint = name.split("_", 1)
    return f"{leg.lower()}/{joint.lower()}"


def _filter_joints(all_joints: list, joint_names: list) -> list:
    """Return joint dicts from all_joints whose 'name' is in joint_names,
    preserving the order they appear in all_joints."""
    name_set = set(joint_names)
    return [j for j in all_joints if j["name"] in name_set]


def _health_summary(ok: dict, attempts: dict, ticks: int, elapsed: float) -> tuple[str, int]:
    """Format per-motor reply rates + effective loop rate; returns (line, worst_pct)."""
    parts = []
    worst = 100
    for name, att in attempts.items():
        pct = (100 * ok.get(name, 0) // att) if att else 100
        worst = min(worst, pct)
        parts.append(f"{name} {pct}%")
    hz = ticks / elapsed if elapsed > 0 else 0.0
    return f"loop {hz:.0f}Hz | " + "  ".join(parts), worst


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
        control_cfg = cfg.get("control", {})
        # 闭源 SDK 的收包超时写死 20ms(正常应答 <0.3ms):丢包时每丢一包全总线
        # 卡 20ms,1kHz 循环实测掉到 30~40Hz。FastSerialPort 是 wrapper.cpp 里
        # 自己实现的串口 I/O(编解码仍用闭源库),超时真正可控。
        timeout_us = int(control_cfg.get("serial_timeout_us", 1500))
        self._retries = int(control_cfg.get("serial_retries", 1))
        try:
            self._serial = sdk.FastSerialPort(serial_port, timeout_us)
        except AttributeError:
            # old extension without FastSerialPort — rebuild unitree_actuator_sdk
            timeout_us = 20000
            self._serial = sdk.SerialPort(serial_port)
        self._timeout_us = timeout_us

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

        self._motor_ids = {j["name"]: int(j["motor_id"]) for j in joints}

        self._cmds = []
        self._datas = []
        for j in joints:
            cmd = sdk.MotorCmd()
            cmd.motorType = sdk.MotorType.GO_M8010_6
            cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
            cmd.id = j["motor_id"]
            cmd.q = 0.0
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

        # bus health stats — reply rate per motor + effective loop rate, logged
        # every _HEALTH_PERIOD so packet loss is visible in the field console.
        self._stat_ok = {name: 0 for name in self._names}
        self._stat_attempts = {name: 0 for name in self._names}
        self._stat_ticks = 0
        self._stat_t0 = time.monotonic()

        self._offsets = self._calibrate_offsets()
        # velocity feedforward targets (motor-convention joint frame, rad/s)
        self._dq_targets: dict[str, float] = {j["name"]: 0.0 for j in joints}
        self._tau_targets: dict[str, float] = {j["name"]: 0.0 for j in joints}
        self._kp_dynamic:  dict[str, float | None] = {j["name"]: None for j in joints}
        self._kd_dynamic:  dict[str, float | None] = {j["name"]: None for j in joints}
        self._gains_stamp: float | None = None

        self.create_subscription(JointState, "/joint_commands", self._on_joint_cmd, 10)
        self.create_subscription(
            Float32MultiArray, "/joint_gains", self._on_joint_gains, 10
        )
        self.add_on_set_parameters_callback(self._on_gains_changed)

        loop_hz = self.get_parameter("loop_hz").value
        self.create_timer(1.0 / loop_hz, self._tick)

        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        self.get_logger().info(
            f"Motor bus ready — {len(joints)} joints on {serial_port}  kp={kp}  kd={kd}"
            f"  recv_timeout={self._timeout_us}us  retries={self._retries}"
        )
        if self._timeout_us >= 20000:
            self.get_logger().warn(
                "SDK binding without timeOutUs — rebuild unitree_actuator_sdk; "
                "using 20ms recv timeout (lossy bus will stall the loop)"
            )

    def _send_recv(self, cmd, data, mid: int) -> bool:
        """sendRecv with stale-flag reset + bounded retries.

        The SDK does NOT clear data.correct/motor_id on a failed exchange, so
        after one success every later failure still looks like fresh data
        (measured 2026-07-14 on a lossy motor: 16/100 real replies but 92/100
        'correct' flags). Reset both before every attempt, or stale readings
        get republished as live ones.
        """
        sdk = self._sdk
        for _ in range(1 + self._retries):
            # sendRecv may overwrite these fields — re-set on every attempt
            data.motorType = sdk.MotorType.GO_M8010_6
            cmd.motorType = sdk.MotorType.GO_M8010_6
            cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
            cmd.id = mid
            data.correct = False
            data.motor_id = 255
            self._serial.sendRecv(cmd, data)
            if data.correct and int(data.motor_id) == mid:
                return True
        return False

    def _calibrate_offsets(self, n_samples: int = 50) -> dict:
        """Sample current positions at power-on and use them as zero reference.

        Sends passive commands (kp=kd=0) and collects n_samples readings per
        joint. Uses the median to reject occasional garbage frames. All
        subsequent position readings and commands are relative to this offset.
        """
        samples: dict = {name: [] for name in self._names}

        self.get_logger().info("Calibrating zero offsets — keep robot still...")
        # Warm up: send passive commands for 1 s so motor firmware fully initialises
        # before we sample. Without this, early frames may return garbage positions.
        for _ in range(100):
            for cmd, data, name in zip(self._cmds, self._datas, self._names):
                cmd.kp = 0.0
                cmd.kd = 0.0
                cmd.q = 0.0
                cmd.dq = 0.0
                cmd.tau = 0.0
                self._send_recv(cmd, data, self._motor_ids[name])
            time.sleep(0.01)
        for _ in range(n_samples):
            for cmd, data, name in zip(self._cmds, self._datas, self._names):
                cmd.kp = 0.0
                cmd.kd = 0.0
                cmd.q = 0.0
                cmd.dq = 0.0
                cmd.tau = 0.0
                if self._send_recv(cmd, data, self._motor_ids[name]):
                    samples[name].append(float(data.q) / self._gear_ratios[name])
            time.sleep(0.01)

        offsets: dict = {}
        for name in self._names:
            vals = samples[name]
            if not vals:
                self.get_logger().warn(
                    f"  {name}: no valid samples — using zero offset"
                )
                offsets[name] = 0.0
            else:
                offsets[name] = statistics.median(vals)
        for name, off in offsets.items():
            self.get_logger().info(f"  {name}: offset={off:.4f} rad")
        return offsets

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_joint_cmd(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if name in self._targets and i < len(msg.position):
                self._targets[name] = float(msg.position[i])
            if name in self._dq_targets and i < len(msg.velocity):
                self._dq_targets[name] = float(msg.velocity[i])
        if len(msg.effort) == len(msg.name):
            for i, name in enumerate(msg.name):
                if name in self._tau_targets:
                    self._tau_targets[name] = float(msg.effort[i])
        self._cmd_time = time.monotonic()
        self._estop_logged = False
        self._estop_done_logged = False

    def _on_joint_gains(self, msg: Float32MultiArray) -> None:
        # data layout: [kp0,kd0,kp1,kd1,...,kp11,kd11] — _YAML_JOINTS order (all 12 joints)
        if len(msg.data) != 2 * len(_YAML_JOINTS):
            return
        for i, jname in enumerate(_YAML_JOINTS):
            if jname in self._kp_dynamic:
                self._kp_dynamic[jname] = float(msg.data[2 * i])
                self._kd_dynamic[jname] = float(msg.data[2 * i + 1])
        self._gains_stamp = time.monotonic()

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        for p in params:
            if p.name in ("kp", "kd"):
                self.get_logger().info(f"Gain updated: {p.name}={p.value}")
        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        now = time.monotonic()
        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value

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

        extrap_dt = min(now - self._cmd_time, _EXTRAPOLATE_CAP) if self._cmd_time is not None else 0.0

        for cmd, data, pub, name in zip(
            self._cmds, self._datas, self._pubs, self._names
        ):
            gr = self._gear_ratios[name]
            offset = self._offsets[name]
            ratio = effective_kp / self._global_kp_init if self._global_kp_init > 0 else 0.0
            ratio_kd = effective_kd / self._global_kd_init if self._global_kd_init > 0 else 0.0
            gains_fresh = (
                self._gains_stamp is not None
                and (now - self._gains_stamp) < _GAINS_TIMEOUT
            )
            base_kp = float(self.get_parameter(f"kp_{name}").value)
            base_kd = float(self.get_parameter(f"kd_{name}").value)
            dyn_kp  = self._kp_dynamic.get(name) if gains_fresh else None
            dyn_kd  = self._kd_dynamic.get(name) if gains_fresh else None
            cmd.kp  = ratio * (dyn_kp if dyn_kp is not None else base_kp)
            cmd.kd  = ratio_kd * (dyn_kd if dyn_kd is not None else base_kd)
            # Extrapolate toward the next expected setpoint using the last known
            # velocity, instead of holding the received q flat until the next
            # /joint_commands arrives — smooths the visible per-tick jump.
            q_target = self._targets[name] + self._dq_targets[name] * extrap_dt
            cmd.q   = (q_target + offset) * gr
            # velocity feedforward in rotor rad/s; fades to 0 with kp on estop
            cmd.dq  = self._dq_targets[name] * gr * ratio
            cmd.tau = ratio * self._tau_targets.get(name, 0.0)
            ok = self._send_recv(cmd, data, self._motor_ids[name])

            self._stat_attempts[name] += 1
            if not ok:
                continue
            self._stat_ok[name] += 1

            # Motor temp / error monitoring
            t = int(data.temp)
            me = int(data.merror)
            if t > 80:
                self.get_logger().error(f"MOTOR OVERHEAT {name}: {t}°C", throttle_duration_sec=2.0)
            elif t > 65:
                self.get_logger().warn(f"motor warm {name}: {t}°C", throttle_duration_sec=5.0)
            if me != 0:
                labels = {1: "overheat", 2: "overcurrent", 3: "overvoltage", 4: "encoder"}
                self.get_logger().error(
                    f"MOTOR FAULT {name}: {labels.get(me, f'unknown({me})')}", throttle_duration_sec=2.0
                )

            pos = float(data.q) / gr - offset

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [name]
            msg.position = [pos]
            msg.velocity = [float(data.dq) / gr]
            msg.effort = [float(data.tau)]
            pub.publish(msg)

        self._stat_ticks += 1
        elapsed = now - self._stat_t0
        if elapsed >= _HEALTH_PERIOD:
            line, worst = _health_summary(
                self._stat_ok, self._stat_attempts, self._stat_ticks, elapsed
            )
            log = (
                self.get_logger().warn
                if worst < _HEALTH_WARN_PCT
                else self.get_logger().info
            )
            log(f"[485] {line}")
            for name in self._names:
                self._stat_ok[name] = 0
                self._stat_attempts[name] = 0
            self._stat_ticks = 0
            self._stat_t0 = now


def main() -> None:
    rclpy.init()
    node = MotorBusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
