"""
teleop_node — reads /joy, publishes /cmd_vel and /posture_command.

Pure functions (_apply_deadzone, _scale_axis, _normalize_trigger_axis,
_button_is_rising_edge) have no ROS2 dependency and can be unit-tested directly.
"""

import os
import yaml


def _apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone and linearly remap to [-1, 1].

    Inside the deadzone (|value| < deadzone), returns 0.0.
    Outside, linearly remaps so that the deadzone edge maps to 0
    and ±1 maps to ±1 (no discontinuity at the boundary).
    """
    if deadzone >= 1.0:
        return 0.0
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _scale_axis(raw: float, deadzone: float, max_vel: float, invert: bool) -> float:
    """Apply deadzone, scale to physical units, and optionally invert.

    Returns velocity in the same units as max_vel (m/s or rad/s).
    """
    scaled = _apply_deadzone(raw, deadzone) * max_vel
    return -scaled if invert else scaled


def _normalize_trigger_axis(raw: float, released_raw: float | None = None) -> float:
    """Map trigger axis to [0, 1] using the observed released baseline.

    Supports common conventions including:
    - released=0.0, pressed=1.0
    - released=-1.0, pressed=1.0
    - released=1.0, pressed=-1.0
    """
    if released_raw is None:
        released_raw = 0.0

    if released_raw >= 0.5:
        denom = released_raw - (-1.0)
        normalized = (released_raw - raw) / denom if denom > 1e-6 else 0.0
    elif released_raw <= -0.5:
        denom = 1.0 - released_raw
        normalized = (raw - released_raw) / denom if denom > 1e-6 else 0.0
    else:
        denom = 1.0 - released_raw
        normalized = (raw - released_raw) / denom if denom > 1e-6 else 0.0

    return max(0.0, min(1.0, normalized))


def _button_is_rising_edge(previous: int, current: int) -> bool:
    return previous == 0 and current == 1


def _integrate_height(current: float, rate: float, dt: float,
                      lo: float, hi: float) -> float:
    """Integrate a height rate (m/s) into an absolute target height, clamped.

    Used to turn the LT/RT trigger rate into the absolute stance-height command
    the policy expects on /height_command.
    """
    return max(lo, min(hi, current + rate * dt))


# ROS2-dependent node class (only define if ROS2 is available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Float32, Int8
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from ament_index_python.packages import get_package_share_directory

    class TeleopNode(Node):
        """Gamepad → /cmd_vel bridge.

        Reads all configuration from robot.yaml teleop section.
        Publishes geometry_msgs/Twist on every /joy message received.
        During e-stop (btn_emergency_stop held), publishes zero Twist every frame.
        """

        def __init__(self):
            super().__init__("teleop_node")
            cfg = self._load_teleop_config()

            self._max_vx = float(cfg["max_vx"])
            self._max_vy = float(cfg["max_vy"])
            self._max_yaw = float(cfg["max_yaw"])
            self._deadzone = float(cfg["deadzone"])
            self._axis_vx = int(cfg["axis_vx"])
            self._axis_vy = int(cfg["axis_vy"])
            self._axis_yaw = int(cfg["axis_yaw"])
            self._invert_vx = bool(cfg["invert_vx"])
            self._invert_vy = bool(cfg["invert_vy"])
            self._invert_yaw = bool(cfg["invert_yaw"])
            self._btn_estop = int(cfg["btn_emergency_stop"])
            self._btn_posture_toggle = int(cfg.get("btn_posture_toggle", 0))
            # Stair step-up triggers: X = front pair, Y = rear pair, both
            # together = clear all learned per-leg levels (manual exit).
            self._btn_step_front = int(cfg.get("btn_step_front", 2))
            self._btn_step_rear = int(cfg.get("btn_step_rear", 3))
            self._prev_step_front = 0
            self._prev_step_rear = 0
            # Gait-mode toggles: LB = stair mode, RB = hurdle mode (auto-
            # enables stair when turned on). The buttons flip mpc_node's
            # parameters through its set_parameters service — one source of
            # truth, `ros2 param get/set` still agrees with the pad.
            self._btn_stair = int(cfg.get("btn_stair_mode", 6))
            self._btn_hurdle = int(cfg.get("btn_hurdle_mode", 7))
            self._prev_stair_btn = 0
            self._prev_hurdle_btn = 0
            self._stair_on = False
            self._hurdle_on = False
            self._axis_lt = int(cfg.get("axis_lt", 2))
            self._axis_rt = int(cfg.get("axis_rt", 5))
            self._max_dz = float(cfg.get("max_dz", 0.03))
            self._height_min = float(cfg.get("height_min", 0.15))
            self._height_max = float(cfg.get("height_max", 0.28))
            self._height_target = float(cfg.get("height_init", 0.22))
            self._last_joy_time: float | None = None
            self._prev_posture_toggle = 0
            self._posture_standing = False
            self._lt_released_raw: float | None = None
            self._rt_released_raw: float | None = None
            self._estop_lying_down = False

            self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self._posture_command_pub = self.create_publisher(Bool, "/posture_command", 10)
            self._height_pub = self.create_publisher(Float32, "/height_command", 10)
            # /step_command: 1 = front legs up one level, 2 = rear legs up,
            # 3 = clear all levels. mpc_node consumes at each leg's next
            # lift-off — press timing is the operator's job.
            self._step_pub = self.create_publisher(Int8, "/step_command", 10)
            self._mpc_param_cli = self.create_client(
                SetParameters, "/mpc_node/set_parameters"
            )
            self.create_subscription(Joy, "/joy", self._on_joy, 10)
            # Publish the height target on a timer too, so a fresh /height_command
            # keeps flowing even when the gamepad is idle (joy_node may go silent).
            self.create_timer(0.05, self._publish_height)
            self.get_logger().info(
                f"teleop_node ready  "
                f"(max_vx={self._max_vx}, max_vy={self._max_vy}, "
                f"max_yaw={self._max_yaw}, deadzone={self._deadzone}, "
                f"btn_estop={self._btn_estop}, btn_posture_toggle={self._btn_posture_toggle})"
            )

        def _send_mpc_bools(self, changes: dict) -> bool:
            """Set boolean parameters on mpc_node. Returns False (and warns)
            when the service isn't up yet, so callers don't flip their local
            toggle state on a press that went nowhere."""
            if not self._mpc_param_cli.service_is_ready():
                self.get_logger().warn(
                    "mpc_node parameter service not ready — mode button ignored"
                )
                return False
            req = SetParameters.Request()
            for name, val in changes.items():
                p = Parameter()
                p.name = name
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=bool(val)
                )
                req.parameters.append(p)
            self._mpc_param_cli.call_async(req)
            return True

        def _load_teleop_config(self) -> dict:
            share = get_package_share_directory("legged_control")
            with open(os.path.join(share, "config", "robot.yaml")) as f:
                return yaml.safe_load(f)["teleop"]

        def _on_joy(self, msg: Joy) -> None:
            twist = Twist()
            buttons = msg.buttons

            now = self.get_clock().now().nanoseconds * 1e-9
            dt = 0.0 if self._last_joy_time is None else max(0.0, min(0.1, now - self._last_joy_time))
            self._last_joy_time = now
            height_rate = 0.0

            posture_toggle_state = (
                buttons[self._btn_posture_toggle]
                if 0 <= self._btn_posture_toggle < len(buttons)
                else 0
            )
            if _button_is_rising_edge(self._prev_posture_toggle, posture_toggle_state):
                self._posture_standing = not self._posture_standing
                self._posture_command_pub.publish(Bool(data=self._posture_standing))
                self.get_logger().info(
                    f"POSTURE: toggle -> standing={self._posture_standing}"
                )
            self._prev_posture_toggle = posture_toggle_state

            step_front = (
                buttons[self._btn_step_front]
                if 0 <= self._btn_step_front < len(buttons) else 0
            )
            step_rear = (
                buttons[self._btn_step_rear]
                if 0 <= self._btn_step_rear < len(buttons) else 0
            )
            front_edge = _button_is_rising_edge(self._prev_step_front, step_front)
            rear_edge = _button_is_rising_edge(self._prev_step_rear, step_rear)
            if (front_edge and step_rear) or (rear_edge and step_front):
                self._step_pub.publish(Int8(data=3))  # both held → clear levels
                self.get_logger().info("STEP: clear all levels")
            elif front_edge:
                self._step_pub.publish(Int8(data=1))
                self.get_logger().info("STEP: front pair up one level")
            elif rear_edge:
                self._step_pub.publish(Int8(data=2))
                self.get_logger().info("STEP: rear pair up one level")
            self._prev_step_front = step_front
            self._prev_step_rear = step_rear

            stair_btn = (
                buttons[self._btn_stair]
                if 0 <= self._btn_stair < len(buttons) else 0
            )
            hurdle_btn = (
                buttons[self._btn_hurdle]
                if 0 <= self._btn_hurdle < len(buttons) else 0
            )
            if _button_is_rising_edge(self._prev_stair_btn, stair_btn):
                if self._send_mpc_bools({"stair_mode": not self._stair_on}):
                    self._stair_on = not self._stair_on
                    self.get_logger().info(
                        f"STAIR MODE -> {'ON' if self._stair_on else 'OFF'}"
                        " (takes effect at standstill)"
                    )
            if _button_is_rising_edge(self._prev_hurdle_btn, hurdle_btn):
                if not self._hurdle_on:
                    # hurdle rides on the crawl — turning it on brings stair
                    # mode with it so one button does the whole preparation
                    if self._send_mpc_bools(
                        {"stair_mode": True, "hurdle_mode": True}
                    ):
                        self._stair_on = True
                        self._hurdle_on = True
                        self.get_logger().info(
                            "HURDLE MODE -> ON (+stair; takes effect at standstill)"
                        )
                else:
                    # off only releases the hurdle; stair stays until LB
                    if self._send_mpc_bools({"hurdle_mode": False}):
                        self._hurdle_on = False
                        self.get_logger().info(
                            "HURDLE MODE -> OFF (stair mode still ON — LB to exit)"
                        )
            self._prev_stair_btn = stair_btn
            self._prev_hurdle_btn = hurdle_btn

            estop_active = (
                self._btn_estop >= 0
                and self._btn_estop < len(buttons)
                and buttons[self._btn_estop] == 1
            )

            if not estop_active:
                self._estop_lying_down = False
                axes = msg.axes
                dz = self._deadzone

                def _safe(idx: int, max_v: float, invert: bool) -> float:
                    if idx < 0 or idx >= len(axes):
                        self.get_logger().warn(
                            f"Axis index {idx} out of range (axes has {len(axes)} elements)",
                            throttle_duration_sec=5.0,
                        )
                        return 0.0
                    return _scale_axis(axes[idx], dz, max_v, invert)

                # Proportional stick, both directions: forward AND backward
                # (was: forward binarised to a fixed 0.4 m/s, backward dropped).
                # The gait's stance stroke / Raibert offset handle vx < 0.
                twist.linear.x = _safe(self._axis_vx, self._max_vx, self._invert_vx)
                twist.linear.y = _safe(self._axis_vy, self._max_vy, self._invert_vy)
                twist.angular.z = _safe(self._axis_yaw, self._max_yaw, self._invert_yaw)
                # Calibrate released trigger baselines from live /joy data so this
                # works across 0..1, -1..1, and 1..-1 trigger conventions.
                rt_raw = axes[self._axis_rt] if 0 <= self._axis_rt < len(axes) else 0.0
                lt_raw = axes[self._axis_lt] if 0 <= self._axis_lt < len(axes) else 0.0
                if self._rt_released_raw is None:
                    self._rt_released_raw = float(rt_raw)
                if self._lt_released_raw is None:
                    self._lt_released_raw = float(lt_raw)
                rt = _scale_axis(
                    _normalize_trigger_axis(rt_raw, self._rt_released_raw),
                    dz,
                    self._max_dz,
                    False,
                )
                lt = _scale_axis(
                    _normalize_trigger_axis(lt_raw, self._lt_released_raw),
                    dz,
                    self._max_dz,
                    False,
                )
                twist.linear.z = rt - lt
                height_rate = rt - lt

            else:
                # E-stop active — publish zero twist and trigger lie-down once
                if not self._estop_lying_down:
                    self._estop_lying_down = True
                    self._posture_command_pub.publish(Bool(data=False))
                    self.get_logger().warn("E-STOP: posture_command=false → lie-down")

            # Integrate trigger rate into the absolute stance-height command the
            # policy consumes (held during e-stop since height_rate stays 0).
            self._height_target = _integrate_height(
                self._height_target, height_rate, dt,
                self._height_min, self._height_max,
            )
            self._height_pub.publish(Float32(data=float(self._height_target)))

            self._pub.publish(twist)

        def _publish_height(self) -> None:
            # Periodic re-publish so /height_command stays fresh when /joy is idle.
            self._height_pub.publish(Float32(data=float(self._height_target)))

except ImportError:
    # ROS2 not available; pure functions still work for testing
    pass


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
