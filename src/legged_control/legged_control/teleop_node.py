"""
teleop_node

Reads joystick input (sensor_msgs/Joy) and publishes velocity commands
(geometry_msgs/Twist) to /cmd_vel for the locomotion policy.

Subscribed topic:
  /joy  (sensor_msgs/Joy)

Published topic:
  /cmd_vel  (geometry_msgs/Twist)
    linear.x:  forward velocity  vx  (m/s)
    linear.y:  lateral velocity  vy  (m/s)
    angular.z: yaw rate               (rad/s)

Parameters (loaded from robot.yaml via ROS params):
  max_vx, max_vy, max_yaw, deadzone
  axis_vx, axis_vy, axis_yaw
  invert_vx, invert_vy, invert_yaw
"""

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import yaml
import os


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_node')

        cfg = self._load_config()['teleop']

        self._max_vx  = cfg['max_vx']
        self._max_vy  = cfg['max_vy']
        self._max_yaw = cfg['max_yaw']
        self._dz      = cfg['deadzone']
        self._ax_vx   = cfg['axis_vx']
        self._ax_vy   = cfg['axis_vy']
        self._ax_yaw  = cfg['axis_yaw']
        self._inv_vx  = -1.0 if cfg['invert_vx']  else 1.0
        self._inv_vy  = -1.0 if cfg['invert_vy']  else 1.0
        self._inv_yaw = -1.0 if cfg['invert_yaw'] else 1.0

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sub = self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info('Teleop node ready. Waiting for /joy ...')

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        path = os.path.join(share, 'config', 'robot.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    def _apply_deadzone(self, v: float) -> float:
        if abs(v) < self._dz:
            return 0.0
        # Rescale so output starts from 0 at deadzone boundary
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self._dz) / (1.0 - self._dz)

    def _on_joy(self, msg: Joy) -> None:
        axes = msg.axes

        def get(ax: int) -> float:
            return axes[ax] if ax < len(axes) else 0.0

        vx  = self._inv_vx  * self._apply_deadzone(get(self._ax_vx))  * self._max_vx
        vy  = self._inv_vy  * self._apply_deadzone(get(self._ax_vy))  * self._max_vy
        yaw = self._inv_yaw * self._apply_deadzone(get(self._ax_yaw)) * self._max_yaw

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = vy
        cmd.angular.z = yaw
        self._pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
