"""
standup_node

Three-phase stand-up sequence with error-limited target tracking:

  Phase 1  HOLD   hold all joints at 0 for `hold_duration` seconds.
                  Lets motors finish initialising before any movement.
  Phase 2  RAMP   continuously move the target toward default_q, but clamp
                  each joint's commanded position error relative to measured
                  feedback so PD output cannot jump too hard.
  Phase 3  STAND  hold at default_q indefinitely.

Typical usage:
  1. Place robot in starting pose and power on.
  2. ros2 launch legged_control robot.launch.py mode:=standup
  3. Watch terminal — it prints phase transitions and a live progress bar.

Runtime-tunable parameters (no relaunch required):
  ros2 param set /standup_node ramp_duration 8.0
  ros2 param set /standup_node max_position_error 0.05
  ros2 param set /standup_node kp 20.0
  ros2 param set /standup_node kd 1.0

kp/kd changes are immediately broadcast to both motor bus nodes.
"""

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import (
    Parameter,
    ParameterType,
    ParameterValue,
    SetParametersResult,
)
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState

_HOLD_DURATION_DEFAULT = 2.0  # seconds at 0 before starting ramp
_RAMP_DURATION_DEFAULT = 3.0  # nominal 0 -> default_q transition time
_MAX_POSITION_ERROR_DEFAULT = 0.14  # cap commanded error per joint (rad)
_LOG_INTERVAL = 1.0  # seconds between progress log lines during ramp


def _smoothstep(t: float) -> float:
    """Kept for compatibility with gait_node standup helpers."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _clamp_error_targets(
    measured: list[float], target: list[float], max_position_error: float
) -> list[float]:
    out = []
    for cur, dst in zip(measured, target):
        delta = dst - cur
        delta = max(-max_position_error, min(max_position_error, delta))
        out.append(cur + delta)
    return out


def _progress_fraction(current: list[float], target: list[float]) -> float:
    total = sum(abs(dst) for dst in target)
    if total <= 1e-9:
        return 1.0
    remaining = sum(abs(dst - cur) for cur, dst in zip(current, target))
    return max(0.0, min(1.0, 1.0 - remaining / total))


def _load_joint_defaults(joints_cfg: list) -> tuple[list, list]:
    names = [j["name"] for j in joints_cfg]
    defaults = [float(j["default_q"]) for j in joints_cfg]
    return names, defaults


class StandupNode(Node):
    def __init__(self) -> None:
        super().__init__("standup_node")

        cfg = self._load_config()
        self._names, self._default_q = _load_joint_defaults(cfg["joints"])

        kp_init = float(cfg["control"]["kp"])
        kd_init = float(cfg["control"]["kd"])
        self.declare_parameter("kp", kp_init)
        self.declare_parameter("kd", kd_init)
        self.declare_parameter("hold_duration", _HOLD_DURATION_DEFAULT)
        self.declare_parameter("ramp_duration", _RAMP_DURATION_DEFAULT)
        self.declare_parameter("max_position_error", _MAX_POSITION_ERROR_DEFAULT)

        self._start_time: float | None = None
        self._phase = "HOLD"  # 'HOLD' | 'RAMP' | 'STAND'
        self._last_log_t: float = 0.0
        self._latest: dict[str, float | None] = {name: None for name in self._names}

        self._gain_clients = [
            self.create_client(SetParameters, "/motor_bus_front/set_parameters"),
            self.create_client(SetParameters, "/motor_bus_rear/set_parameters"),
        ]

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joint_states, 10
        )
        self.create_timer(1.0 / 50.0, self._publish)
        self.add_on_set_parameters_callback(self._on_gains_changed)

        hold = self.get_parameter("hold_duration").value
        ramp = self.get_parameter("ramp_duration").value
        max_error = self.get_parameter("max_position_error").value
        self.get_logger().info(
            f"[standup] ready — hold {hold:.1f}s then ramp {ramp:.1f}s to default_q  "
            f"max_position_error={max_error:.3f}  "
            f"kp={kp_init}  kd={kd_init}"
        )

    # ── config ───────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_joint_states(self, msg: JointState) -> None:
        positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        for name in self._names:
            if name in positions:
                self._latest[name] = float(positions[name])

    def _measured_positions(self) -> list[float] | None:
        values = [self._latest[name] for name in self._names]
        if any(value is None for value in values):
            return None
        return [float(value) for value in values]

    # ── gain broadcast ────────────────────────────────────────────────────────

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        new_kp = next((p.value for p in params if p.name == "kp"), None)
        new_kd = next((p.value for p in params if p.name == "kd"), None)
        if new_kp is not None or new_kd is not None:
            kp = new_kp if new_kp is not None else self.get_parameter("kp").value
            kd = new_kd if new_kd is not None else self.get_parameter("kd").value
            req = SetParameters.Request()
            req.parameters = [
                Parameter(
                    name="kp",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp)
                    ),
                ),
                Parameter(
                    name="kd",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd)
                    ),
                ),
            ]
            for client in self._gain_clients:
                if client.service_is_ready():
                    client.call_async(req)
            self.get_logger().info(f"[standup] gains updated → kp={kp}  kd={kd}")
        return SetParametersResult(successful=True)

    # ── main loop ─────────────────────────────────────────────────────────────

    def _publish(self) -> None:
        now = time.monotonic()
        if self._start_time is None:
            self._start_time = now

        hold_dur = self.get_parameter("hold_duration").value
        ramp_dur = float(self.get_parameter("ramp_duration").value)
        max_position_error = float(self.get_parameter("max_position_error").value)
        elapsed = now - self._start_time

        if elapsed < hold_dur:
            # ── Phase 1: HOLD ──────────────────────────────────────────────
            if self._phase != "HOLD":
                self._phase = "HOLD"
            targets = [0.0] * len(self._names)

        else:
            measured = self._measured_positions()
            if measured is None:
                if self._phase != "RAMP":
                    self._phase = "RAMP"
                    self.get_logger().info(
                        "[standup] waiting for /joint_states_aggregated before ramping"
                    )
                targets = [0.0] * len(self._names)
            elif _progress_fraction(measured, self._default_q) >= 0.999:
                if self._phase != "STAND":
                    self._phase = "STAND"
                    self.get_logger().info(
                        "[standup] STAND reached — holding at default_q"
                    )
                targets = list(self._default_q)
            else:
                if self._phase != "RAMP":
                    self._phase = "RAMP"
                    self.get_logger().info(
                        "[standup] RAMP started — moving to default_q with position-error clamp"
                    )

                alpha = _smoothstep((elapsed - hold_dur) / max(ramp_dur, 1e-6))
                raw_targets = [alpha * dq for dq in self._default_q]
                targets = _clamp_error_targets(
                    measured, raw_targets, max_position_error
                )

                if now - self._last_log_t >= _LOG_INTERVAL:
                    pct = int(_progress_fraction(measured, self._default_q) * 100)
                    bar = "█" * (pct // 5) + "░" * (20 - pct // 5)
                    self.get_logger().info(f"[standup] [{bar}] {pct:3d}%")
                    self._last_log_t = now

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._names)
        msg.position = targets
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StandupNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
