"""
fixed_cmd_node — publishes a standardized constant command instead of the joystick,
so the deployment-side obs collection uses the exact same command as the training
side (see docs/training_obs_collection_prompt.md).

Publishes at 50 Hz:
  /cmd_vel        (geometry_msgs/Twist)  — linear.x, linear.y, angular.z
  /height_command (std_msgs/Float32)     — stance height

Run this INSTEAD of teleop_node during obs collection (don't touch the joystick).

Params (override with -p):
  cmd_vx   (default 0.4)   forward m/s
  cmd_vy   (default 0.0)   lateral m/s
  cmd_wz   (default 0.0)   yaw rad/s
  height   (default 0.25)  stance height m
  auto_standup (default False) — if True, publish /posture_command=true once at
                                 start (hands-free stand up). Use with care.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class FixedCmdNode(Node):
    def __init__(self) -> None:
        super().__init__("fixed_cmd_node")

        self.declare_parameter("cmd_vx", 0.4)
        self.declare_parameter("cmd_vy", 0.0)
        self.declare_parameter("cmd_wz", 0.0)
        self.declare_parameter("height", 0.25)
        self.declare_parameter("auto_standup", False)

        self._vx = float(self.get_parameter("cmd_vx").value)
        self._vy = float(self.get_parameter("cmd_vy").value)
        self._wz = float(self.get_parameter("cmd_wz").value)
        self._height = float(self.get_parameter("height").value)
        self._auto_standup = bool(self.get_parameter("auto_standup").value)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._height_pub = self.create_publisher(Float32, "/height_command", 10)
        self._posture_pub = self.create_publisher(Bool, "/posture_command", 10)

        if self._auto_standup:
            self.create_timer(1.0, self._standup_once)
        self._standup_done = False

        self.create_timer(0.02, self._publish)
        self.get_logger().info(
            f"fixed_cmd_node ready — cmd=({self._vx},{self._vy},{self._wz}) "
            f"height={self._height} auto_standup={self._auto_standup}"
        )

    def _standup_once(self) -> None:
        if self._standup_done:
            return
        self._posture_pub.publish(Bool(data=True))
        self._standup_done = True
        self.get_logger().info("fixed_cmd_node: posture_command=true (auto standup)")

    def _publish(self) -> None:
        twist = Twist()
        twist.linear.x = self._vx
        twist.linear.y = self._vy
        twist.angular.z = self._wz
        self._cmd_pub.publish(twist)
        self._height_pub.publish(Float32(data=self._height))


def main() -> None:
    rclpy.init()
    node = FixedCmdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
