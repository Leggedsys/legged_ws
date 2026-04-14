"""
teleop_node — reads /joy, publishes /cmd_vel for policy_node.

Pure functions (_apply_deadzone, _scale_axis) have no ROS2 dependency
and can be unit-tested directly.
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


# ROS2-dependent node class (only define if ROS2 is available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import Twist
    from ament_index_python.packages import get_package_share_directory

    class TeleopNode(Node):
        """Gamepad → /cmd_vel bridge.

        Reads all configuration from robot.yaml teleop section.
        Publishes geometry_msgs/Twist on every /joy message received.
        During e-stop (btn_emergency_stop held), publishes zero Twist every frame
        so policy_node keeps receiving commands and stays in standing posture.
        """

        def __init__(self):
            super().__init__('teleop_node')
            cfg = self._load_teleop_config()

            self._max_vx    = float(cfg['max_vx'])
            self._max_vy    = float(cfg['max_vy'])
            self._max_yaw   = float(cfg['max_yaw'])
            self._deadzone  = float(cfg['deadzone'])
            self._axis_vx   = int(cfg['axis_vx'])
            self._axis_vy   = int(cfg['axis_vy'])
            self._axis_yaw  = int(cfg['axis_yaw'])
            self._invert_vx  = bool(cfg['invert_vx'])
            self._invert_vy  = bool(cfg['invert_vy'])
            self._invert_yaw = bool(cfg['invert_yaw'])
            self._btn_estop  = int(cfg['btn_emergency_stop'])

            self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.create_subscription(Joy, '/joy', self._on_joy, 10)
            self.get_logger().info(
                f'teleop_node ready  '
                f'(max_vx={self._max_vx}, max_vy={self._max_vy}, '
                f'max_yaw={self._max_yaw}, deadzone={self._deadzone}, '
                f'btn_estop={self._btn_estop})'
            )

        def _load_teleop_config(self) -> dict:
            share = get_package_share_directory('legged_control')
            with open(os.path.join(share, 'config', 'robot.yaml')) as f:
                return yaml.safe_load(f)['teleop']

        def _on_joy(self, msg: Joy) -> None:
            twist = Twist()

            estop_active = (
                self._btn_estop >= 0
                and self._btn_estop < len(msg.buttons)
                and msg.buttons[self._btn_estop] == 1
            )

            if not estop_active:
                axes = msg.axes
                dz   = self._deadzone

                def _safe(idx: int, max_v: float, invert: bool) -> float:
                    if idx < 0 or idx >= len(axes):
                        self.get_logger().warn(
                            f'Axis index {idx} out of range (axes has {len(axes)} elements)',
                            throttle_duration_sec=5.0,
                        )
                        return 0.0
                    return _scale_axis(axes[idx], dz, max_v, invert)

                twist.linear.x  = _safe(self._axis_vx,  self._max_vx,  self._invert_vx)
                twist.linear.y  = _safe(self._axis_vy,  self._max_vy,  self._invert_vy)
                twist.angular.z = _safe(self._axis_yaw, self._max_yaw, self._invert_yaw)

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
        rclpy.shutdown()
