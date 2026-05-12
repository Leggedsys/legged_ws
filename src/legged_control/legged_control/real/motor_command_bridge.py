"""motor_command_bridge

Subscribes /joint_commands (URDF frame, YAML order) and converts
to motor frame for motor_bus_node.

q_motor = direction * (q_urdf - zero_offset)

Publishes /joint_commands_motor (motor frame) — motor_bus_node subscribes
to this via launch remap.  Only needed on real hardware; simulation uses
gazebo_control_bridge directly.
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MotorCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("motor_command_bridge")

        self.declare_parameter("config_path", "")
        cfg = self._load_config()
        self._joint_cfg = {j["name"]: j for j in cfg["joints"]}
        self._names = [j["name"] for j in cfg["joints"]]

        self._pub = self.create_publisher(
            JointState, "/joint_commands_motor", 10
        )
        self.create_subscription(
            JointState, "/joint_commands", self._on_command, 10,
        )
        self.get_logger().info("motor_command_bridge ready — /joint_commands → /joint_commands_motor")

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        path = str(self.get_parameter("config_path").value or "").strip()
        if not path:
            path = os.path.join(share, "config", "robot.yaml")
        with open(path) as f:
            return yaml.safe_load(f)

    def _on_command(self, msg: JointState) -> None:
        pos_map = dict(zip(msg.name, msg.position))

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._names)
        out.position = []
        for name in self._names:
            cfg = self._joint_cfg[name]
            direction = float(cfg["direction"])
            zero_offset = float(cfg["zero_offset"])
            q_urdf = float(pos_map.get(name, 0.0))
            q_motor = direction * (q_urdf - zero_offset)
            q_motor = float(
                max(float(cfg["q_min"]), min(float(cfg["q_max"]), q_motor))
            )
            out.position.append(q_motor)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MotorCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
