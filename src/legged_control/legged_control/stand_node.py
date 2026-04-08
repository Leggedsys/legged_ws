"""
stand_node

Publishes default_q for all 12 joints at 50 Hz on /joint_commands.
Motors run with PD gains from robot.yaml, holding the robot in its
default standing pose. Used for manual kp/kd tuning: edit robot.yaml
and restart the launch to apply new gains.

No joystick input, no feedback subscription.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _load_joint_defaults(joints_cfg: list) -> tuple:
    """Return (names, default_q_values) from joints config list."""
    names    = [j['name'] for j in joints_cfg]
    defaults = [float(j['default_q']) for j in joints_cfg]
    return names, defaults


class StandNode(Node):
    def __init__(self) -> None:
        super().__init__('stand_node')

        cfg = self._load_config()
        self._names, self._default_q = _load_joint_defaults(cfg['joints'])

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(1.0 / 50.0, self._publish)

        self.get_logger().info(
            f'Stand node ready — holding {len(self._names)} joints at default_q'
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self._names)
        msg.position = list(self._default_q)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
