"""
watchdog_node

Monitors /joint_commands heartbeat from policy_node.
If no message is received within `timeout` seconds after the policy
has started publishing, enters fallback mode: publishes default_q
commands continuously so the robot holds its standing pose.

Fallback is latched until the node is restarted (no auto-recovery),
which prevents oscillating between policy and watchdog output.

Parameters (ROS):
  timeout  (float, default 0.5)  seconds of silence before fallback triggers

Subscribed topic:
  /joint_commands  sensor_msgs/JointState  (published by policy_node)

Published topic (fallback only):
  /joint_commands  sensor_msgs/JointState  (default_q hold)
"""

from __future__ import annotations
import os
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__('watchdog_node')

        self.declare_parameter('timeout', 0.5)
        self._timeout: float = self.get_parameter('timeout').value

        cfg = self._load_config()
        joints = cfg['joints']
        self._joint_names: list[str]   = [j['name']      for j in joints]
        self._default_q:   list[float] = [j['default_q'] for j in joints]

        self._last_msg_time  = self.get_clock().now()
        self._first_msg_seen = False
        self._in_fallback    = False

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)
        self._sub = self.create_subscription(
            JointState, '/joint_commands', self._on_cmd, 10)

        # Check at 10 Hz (fast enough to react within ~150 ms of timeout)
        self.create_timer(0.1, self._check)
        self.get_logger().info(f'Watchdog ready. Timeout = {self._timeout} s.')

    # ------------------------------------------------------------------ helpers

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        path = os.path.join(share, 'config', 'robot.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    # ---------------------------------------------------------------- callbacks

    def _on_cmd(self, msg: JointState) -> None:
        if self._in_fallback:
            # Ignore all messages during fallback — may be our own publications.
            return
        self._first_msg_seen = True
        self._last_msg_time = self.get_clock().now()

    def _check(self) -> None:
        if self._in_fallback:
            self._publish_default()
            return

        # Don't trigger before the policy has produced its first message.
        if not self._first_msg_seen:
            return

        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if elapsed > self._timeout:
            self.get_logger().error(
                f'Policy silent for {elapsed:.2f} s — entering fallback '
                f'(holding default_q). Restart node to clear.')
            self._in_fallback = True
            self._publish_default()

    def _publish_default(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self._joint_names
        msg.position = self._default_q
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
