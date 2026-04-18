"""
teleop_node — reads /joy, publishes /cmd_vel for gait_node.

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


# ROS2-dependent node class (only define if ROS2 is available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from ament_index_python.packages import get_package_share_directory

    class TeleopNode(Node):
        """Gamepad → /cmd_vel bridge.

        Reads all configuration from robot.yaml teleop section.
        Publishes geometry_msgs/Twist on every /joy message received.
        During e-stop (btn_emergency_stop held), publishes zero Twist every frame
        so gait_node keeps receiving commands and stays in standing posture.
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
            self._axis_lt = int(cfg.get("axis_lt", 2))
            self._axis_rt = int(cfg.get("axis_rt", 5))
            self._max_dz = float(cfg.get("max_dz", 0.03))
            self._prev_posture_toggle = 0
            self._posture_standing = False
            self._lt_released_raw: float | None = None
            self._rt_released_raw: float | None = None

            self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self._posture_command_pub = self.create_publisher(Bool, "/posture_command", 10)
            self.create_subscription(Joy, "/joy", self._on_joy, 10)
            self.get_logger().info(
                f"teleop_node ready  "
                f"(max_vx={self._max_vx}, max_vy={self._max_vy}, "
                f"max_yaw={self._max_yaw}, deadzone={self._deadzone}, "
                f"btn_estop={self._btn_estop}, btn_posture_toggle={self._btn_posture_toggle})"
            )

        def _load_teleop_config(self) -> dict:
            share = get_package_share_directory("legged_control")
            with open(os.path.join(share, "config", "robot.yaml")) as f:
                return yaml.safe_load(f)["teleop"]

        def _on_joy(self, msg: Joy) -> None:
            twist = Twist()
            buttons = msg.buttons

            posture_toggle_state = (
                buttons[self._btn_posture_toggle]
                if 0 <= self._btn_posture_toggle < len(buttons)
                else 0
            )
            if _button_is_rising_edge(self._prev_posture_toggle, posture_toggle_state):
                self._posture_standing = not self._posture_standing
                self._posture_command_pub.publish(Bool(data=self._posture_standing))
            self._prev_posture_toggle = posture_toggle_state

            estop_active = (
                self._btn_estop >= 0
                and self._btn_estop < len(buttons)
                and buttons[self._btn_estop] == 1
            )

            if not estop_active:
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

            self._pub.publish(twist)

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
