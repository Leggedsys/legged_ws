"""Bridge legged_control topics to Gazebo ros2_control topics."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory


URDF_JOINT_ORDER = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
]


def _joint_name_to_urdf_joint(name: str) -> str:
    return f"{name}_joint"


def _urdf_joint_to_joint_name(name: str) -> str:
    return name[:-6] if name.endswith("_joint") else name


def _load_joint_cfg_from_path(config_path: str) -> dict[str, dict]:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return {joint["name"]: joint for joint in cfg["joints"]}


def _motor_commands_to_urdf_positions(
    names: list[str],
    positions: list[float],
    joint_cfg: dict[str, dict],
    urdf_order: list[str],
) -> list[float]:
    by_name = {name: float(pos) for name, pos in zip(names, positions)}
    out = []
    for urdf_joint in urdf_order:
        joint_name = _urdf_joint_to_joint_name(urdf_joint)
        cfg = joint_cfg[joint_name]
        q_motor = by_name.get(joint_name, float(cfg["default_q"]))
        out.append(float(cfg["direction"]) * q_motor + float(cfg["zero_offset"]))
    return out


def _urdf_joint_state_to_motor_aggregated(
    names: list[str],
    positions: list[float],
    velocities: list[float],
    joint_cfg: dict[str, dict],
    joint_order: list[str],
) -> tuple[list[str], list[float], list[float]]:
    pos_by_name = {name: float(pos) for name, pos in zip(names, positions)}
    vel_by_name = {name: float(vel) for name, vel in zip(names, velocities)}

    out_names = []
    out_positions = []
    out_velocities = []
    for joint_name in joint_order:
        urdf_joint = _joint_name_to_urdf_joint(joint_name)
        cfg = joint_cfg[joint_name]
        q_urdf = pos_by_name.get(urdf_joint, float(cfg["zero_offset"]))
        dq_urdf = vel_by_name.get(urdf_joint, 0.0)
        direction = float(cfg["direction"])
        zero_offset = float(cfg["zero_offset"])
        out_names.append(joint_name)
        out_positions.append((q_urdf - zero_offset) / direction)
        out_velocities.append(dq_urdf / direction)
    return out_names, out_positions, out_velocities


class GazeboControlBridgeNode:  # pragma: no cover - runtime node
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__("gazebo_control_bridge")
                self.declare_parameter("config_path", "")

                config_path = str(self.get_parameter("config_path").value or "").strip()
                if not config_path:
                    share = get_package_share_directory("legged_control")
                    config_path = os.path.join(share, "config", "robot.yaml")

                self._joint_cfg = _load_joint_cfg_from_path(config_path)
                self._joint_order = list(self._joint_cfg.keys())
                self._cmd_pub = self.create_publisher(
                    Float64MultiArray, "/gait_position_controller/commands", 10
                )
                self._agg_pub = self.create_publisher(
                    JointState, "/joint_states_aggregated", 10
                )

                self.create_subscription(
                    JointState, "/joint_commands", self._on_joint_commands, 10
                )
                self.create_subscription(
                    JointState, "/joint_states", self._on_joint_states, 10
                )
                self.get_logger().info("gazebo_control_bridge ready")

            def _on_joint_commands(self, msg: JointState) -> None:
                out = Float64MultiArray()
                out.data = _motor_commands_to_urdf_positions(
                    list(msg.name),
                    list(msg.position),
                    self._joint_cfg,
                    URDF_JOINT_ORDER,
                )
                self._cmd_pub.publish(out)

            def _on_joint_states(self, msg: JointState) -> None:
                names, pos, vel = _urdf_joint_state_to_motor_aggregated(
                    list(msg.name),
                    list(msg.position),
                    list(msg.velocity),
                    self._joint_cfg,
                    self._joint_order,
                )
                out = JointState()
                out.header.stamp = self.get_clock().now().to_msg()
                out.name = names
                out.position = pos
                out.velocity = vel
                self._agg_pub.publish(out)

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
    app = GazeboControlBridgeNode()
    try:
        app.spin()
    finally:
        app.shutdown()
