"""single_motor_test — direct RS485 motor test, bypassing all intermediate ROS nodes.

Talks to ONE motor by ID over the raw serial bus. Runs a frequency sweep or sine
trajectory in URDF frame and logs commanded vs measured position.

Use to isolate: is poor tracking coming from the motor+RS485 chain, or from
intermediate nodes (joint_agg, motor_command_bridge)?

Usage:
  ros2 run legged_control single_motor_test -p motor_id:=0

Output: ~/.legged_logs/single_motor_<ts>/single_motor.csv + .png
"""

from __future__ import annotations

import os
import statistics
import time as _time

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


def _load_robot_cfg() -> dict:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _find_joint_by_motor_id(cfg: dict, motor_id: int) -> dict | None:
    for j in cfg.get("joints", []):
        if int(j.get("motor_id", -1)) == motor_id:
            return j
    return None


_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


def _make_log_dir() -> str:
    ts = _time.strftime("%Y%m%d_%H%M%S")
    d = os.path.expanduser(f"~/.legged_logs/single_motor_{ts}")
    os.makedirs(d, exist_ok=True)
    return d


def _determine_serial_port(joint_name: str, base_cfg: dict) -> str:
    ctrl = base_cfg.get("control", {})
    leg = joint_name.split("_")[0]
    port_key = "serial_port_front" if leg in ("FR", "FL") else "serial_port_rear"
    return ctrl.get(port_key, "/dev/ttyUSB0")


class SingleMotorTest(Node):
    def __init__(self) -> None:
        super().__init__("single_motor_test")

        self.declare_parameter("motor_id", 0)
        self.declare_parameter("freq", 2.0)
        self.declare_parameter("freq_end", 8.0)
        self.declare_parameter("amplitude", 0.25)
        self.declare_parameter("duration", 10.0)
        self.declare_parameter("ramp_dur", 3.0)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("kd", 0.0125)
        self.declare_parameter("loop_hz", 500.0)

        motor_id = int(self.get_parameter("motor_id").value)
        self._freq = float(self.get_parameter("freq").value)
        self._freq_end = float(self.get_parameter("freq_end").value)
        self._amp = float(self.get_parameter("amplitude").value)
        self._duration = float(self.get_parameter("duration").value)
        self._ramp_dur = float(self.get_parameter("ramp_dur").value)
        self._kp = float(self.get_parameter("kp").value)
        self._kd = float(self.get_parameter("kd").value)
        loop_hz = float(self.get_parameter("loop_hz").value)

        cfg = _load_robot_cfg()
        joint = _find_joint_by_motor_id(cfg, motor_id)
        if joint is None:
            raise RuntimeError(f"motor_id={motor_id} not found in robot.yaml joints")

        self._joint_name = joint["name"]
        self._gr = float(joint.get("gear_ratio", 6.33))
        self._q_min = float(joint.get("q_min", -1.0))
        self._q_max = float(joint.get("q_max", 1.0))
        serial_port = _determine_serial_port(self._joint_name, cfg)

        from unitree_motor_ros2.sdk_loader import load_sdk
        sdk = load_sdk()
        self._sdk = sdk
        self._serial = sdk.SerialPort(serial_port)
        self._motor_id = motor_id

        mt = sdk.MotorType.GO_M8010_6
        self._mode = sdk.queryMotorMode(mt, sdk.MotorMode.FOC)

        self._cmd = sdk.MotorCmd()
        self._data = sdk.MotorData()
        self._offset = self._calibrate_offset()

        self._center: float = 0.0
        self._pos_seen = False
        self._start_time: float | None = None
        self._rows: list[list[str]] = []

        self.get_logger().info(
            f"single_motor_test ready — motor_id={motor_id} ({self._joint_name}) "
            f"gr={self._gr} q_limits=[{self._q_min},{self._q_max}] "
            f"kp={self._kp} kd={self._kd} port={serial_port}"
        )

        self.create_timer(1.0 / loop_hz, self._tick)
        self.create_timer(0.1, self._check_duration)

    def _calibrate_offset(self, n_samples: int = 50) -> float:
        sdk = self._sdk
        self.get_logger().info("calibrating zero offset — keep motor still...")
        for _ in range(100):
            self._cmd.motorType = sdk.MotorType.GO_M8010_6
            self._cmd.mode = self._mode
            self._cmd.id = self._motor_id
            self._cmd.kp = 0.0
            self._cmd.kd = 0.0
            self._cmd.q = 0.0
            self._cmd.dq = 0.0
            self._cmd.tau = 0.0
            self._data.motorType = sdk.MotorType.GO_M8010_6
            self._serial.sendRecv(self._cmd, self._data)
            _time.sleep(0.01)

        samples: list[float] = []
        for _ in range(n_samples):
            self._data.motorType = sdk.MotorType.GO_M8010_6
            self._cmd.motorType = sdk.MotorType.GO_M8010_6
            self._cmd.mode = self._mode
            self._cmd.id = self._motor_id
            self._cmd.kp = 0.0
            self._cmd.kd = 0.0
            self._cmd.q = 0.0
            self._cmd.dq = 0.0
            self._cmd.tau = 0.0
            self._serial.sendRecv(self._cmd, self._data)
            if self._data.correct and int(self._data.motor_id) == self._motor_id:
                samples.append(float(self._data.q) / self._gr)
            _time.sleep(0.01)

        offset = statistics.median(samples) if samples else 0.0
        self.get_logger().info(f"zero offset = {offset:.4f} rad (URDF)")
        return offset

    def _tick(self) -> None:
        sdk = self._sdk

        self._data.motorType = sdk.MotorType.GO_M8010_6
        self._cmd.motorType = sdk.MotorType.GO_M8010_6
        self._cmd.mode = self._mode
        self._cmd.id = self._motor_id

        if self._start_time is None:
            self._start_time = _time.monotonic()
            self._center = 0.0
            # send one passive read to get initial position
            self._cmd.kp = 0.0
            self._cmd.kd = 0.0
            self._cmd.q = 0.0
            self._cmd.dq = 0.0
            self._cmd.tau = 0.0
            self._serial.sendRecv(self._cmd, self._data)
            if self._data.correct and int(self._data.motor_id) == self._motor_id:
                self._center = float(self._data.q) / self._gr - self._offset
                self._pos_seen = True
                self.get_logger().info(f"locked center = {self._center:.4f} rad")
            return

        t = _time.monotonic() - self._start_time

        # ramp to center
        ramp_alpha = min(t / max(self._ramp_dur, 1e-6), 1.0)
        test_dur = max(self._duration - self._ramp_dur, 1e-6)
        t_test = max(t - self._ramp_dur, 0.0)

        # chirp phase
        f0, f1 = self._freq, self._freq_end
        phase = f0 * t_test + (f1 - f0) * t_test * t_test / (2.0 * test_dur) if t_test > 0 else 0.0
        sine_val = self._amp * np.sin(2.0 * np.pi * phase)
        sine_scale = 1.0 if t_test > 0 else 0.0

        target_urdf = self._center + sine_val * sine_scale
        target_urdf = np.clip(target_urdf, self._q_min + 0.01, self._q_max - 0.01)

        self._cmd.kp = self._kp
        self._cmd.kd = self._kd
        self._cmd.q = (target_urdf + self._offset) * self._gr
        self._cmd.dq = 0.0
        self._cmd.tau = 0.0

        self._serial.sendRecv(self._cmd, self._data)

        cmd_urdf = target_urdf
        act_urdf = 0.0
        if self._data.correct and int(self._data.motor_id) == self._motor_id:
            act_urdf = float(self._data.q) / self._gr - self._offset

        row = [f"{_time.monotonic():.6f}", f"{cmd_urdf:.6f}", f"{act_urdf:.6f}"]
        self._rows.append(row)

    def _check_duration(self) -> None:
        if self._start_time is not None and _time.monotonic() - self._start_time >= self._duration:
            self.get_logger().info("duration reached — stopping")
            self._write_output()
            rclpy.shutdown()

    def _write_output(self) -> None:
        log_dir = _make_log_dir()
        csv_path = os.path.join(log_dir, "single_motor.csv")
        with open(csv_path, "w") as f:
            f.write("t_mono,cmd_rad,act_rad\n")
            for row in self._rows:
                f.write(",".join(row) + "\n")
        self.get_logger().info(f"CSV: {csv_path}")
        try:
            self._plot(log_dir)
        except Exception as e:
            self.get_logger().error(f"plot error: {e}")

    def _plot(self, log_dir: str) -> None:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        if not self._rows:
            return
        data = np.array([[float(v) for v in r] for r in self._rows])
        t = data[:, 0] - data[0, 0]
        cmd = data[:, 1]
        act = data[:, 2]
        err = cmd - act

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        ax1.plot(t, cmd, "b-", alpha=0.7, linewidth=0.8, label="cmd")
        ax1.plot(t, act, "r-", alpha=0.7, linewidth=0.8, label="act")
        ax1.legend(loc="upper right", fontsize=8)
        ax1.set_ylabel("rad (URDF)")
        rms = np.sqrt(np.mean(err[int(len(err)*0.3):]**2))
        ax1.set_title(
            f"single_motor  id={self._motor_id} ({self._joint_name})  "
            f"f={self._freq}→{self._freq_end}Hz  amp={self._amp}  RMS={rms:.4f}"
        )
        ax1.grid(True, alpha=0.3)
        ax2.plot(t, err, "k-", alpha=0.6, linewidth=0.6)
        ax2.set_ylabel("error (rad)")
        ax2.axhline(0, color="gray", linewidth=0.5)
        ax2.set_xlabel("time (s)")
        ax2.grid(True, alpha=0.3)
        fig.tight_layout()
        png = os.path.join(log_dir, "single_motor.png")
        fig.savefig(png, dpi=120)
        plt.close(fig)
        self.get_logger().info(f"plot: {png}")


def main() -> None:
    rclpy.init()
    node = SingleMotorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
