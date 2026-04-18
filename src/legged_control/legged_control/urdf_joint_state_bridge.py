"""Bridge aggregated motor-frame joint states into URDF joint states."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState


def _joint_name_to_urdf_joint(name: str) -> str:
    return f"{name}_joint"


def _motor_to_urdf_joint_state(
    names: list[str],
    positions: list[float],
    velocities: list[float],
    joint_cfg: dict[str, dict],
) -> tuple[list[str], list[float], list[float]]:
    urdf_names = []
    urdf_positions = []
    urdf_velocities = []
    for idx, name in enumerate(names):
        cfg = joint_cfg[name]
        direction = float(cfg["direction"])
        zero_offset = float(cfg["zero_offset"])
        pos = float(positions[idx]) if idx < len(positions) else 0.0
        vel = float(velocities[idx]) if idx < len(velocities) else 0.0
        urdf_names.append(_joint_name_to_urdf_joint(name))
        urdf_positions.append(direction * pos + zero_offset)
        urdf_velocities.append(direction * vel)
    return urdf_names, urdf_positions, urdf_velocities


class URDFJointStateBridgeNode:  # pragma: no cover - exercised via ROS runtime
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__("urdf_joint_state_bridge")
                self.declare_parameter("config_path", "")
                self._joint_cfg = self._load_joint_cfg()
                self._pub = self.create_publisher(JointState, "/joint_states", 10)
                self.create_subscription(
                    JointState, "/joint_states_aggregated", self._on_joint_states, 10
                )
                self.get_logger().info(
                    "urdf_joint_state_bridge ready — publishing /joint_states in URDF frame"
                )

            def _load_joint_cfg(self) -> dict[str, dict]:
                share = get_package_share_directory("legged_control")
                config_path = str(self.get_parameter("config_path").value or "").strip()
                if not config_path:
                    config_path = os.path.join(share, "config", "robot.yaml")
                with open(config_path) as f:
                    cfg = yaml.safe_load(f)
                return {joint["name"]: joint for joint in cfg["joints"]}

            def _on_joint_states(self, msg: JointState) -> None:
                names, positions, velocities = _motor_to_urdf_joint_state(
                    list(msg.name),
                    list(msg.position),
                    list(msg.velocity),
                    self._joint_cfg,
                )
                out = JointState()
                out.header.stamp = self.get_clock().now().to_msg()
                out.name = names
                out.position = positions
                out.velocity = velocities
                self._pub.publish(out)

        self._rclpy = rclpy
        self._node = _Node()

    def spin(self) -> None:
        self._rclpy.spin(self._node)

    def shutdown(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy.ok():
            self._rclpy.shutdown()


def main() -> None:
    import rclpy

    rclpy.init()
    app = URDFJointStateBridgeNode()
    try:
        app.spin()
    finally:
        app.shutdown()
