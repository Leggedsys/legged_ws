"""vel_viz_node

Publishes velocity arrows as MarkerArray for RViz visualization.

  Green arrow — estimated linear velocity (from /state_estimate, yaw frame)
  Blue arrow  — commanded linear velocity (from /cmd_vel)

Both arrows are anchored at base_link origin, scaled to vector magnitude.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def _arrow_marker(
    marker_id: int,
    frame_id: str,
    vx: float,
    vy: float,
    r: float,
    g: float,
    b: float,
    stamp,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "vel_viz"
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.z = 0.50
    m.pose.orientation.w = 1.0

    speed = math.hypot(vx, vy)
    m.scale.x = max(speed, 0.01)
    m.scale.y = 0.03
    m.scale.z = 0.03

    yaw = math.atan2(vy, vx)
    m.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.orientation.w = math.cos(yaw / 2.0)

    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 0.85 if speed > 0.01 else 0.2
    return m


class VelVizNode(Node):
    def __init__(self) -> None:
        super().__init__("vel_viz_node")

        self._est_vx = 0.0
        self._est_vy = 0.0
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0

        self._pub = self.create_publisher(MarkerArray, "/vel_viz", 10)

        self.create_subscription(
            Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self.create_timer(0.1, self._publish)
        self.get_logger().info("vel_viz_node ready")

    def _on_state(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._est_vx = float(msg.data[0])
            self._est_vy = float(msg.data[1])

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vx = float(msg.linear.x)
        self._cmd_vy = float(msg.linear.y)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        array.markers.append(_arrow_marker(
            0, "base_link",
            self._est_vx, self._est_vy,
            0.0, 0.9, 0.0, now,
        ))
        array.markers.append(_arrow_marker(
            1, "base_link",
            self._cmd_vx, self._cmd_vy,
            0.0, 0.4, 1.0, now,
        ))
        self._pub.publish(array)


def main() -> None:
    rclpy.init()
    node = VelVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
