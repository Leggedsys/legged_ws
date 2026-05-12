"""Bridge legged_control topics to Gazebo ros2_control topics.

No coordinate conversion — Gazebo works natively in URDF frame.
Only reformats joint names/order between legged_control and Gazebo.
"""

from __future__ import annotations

# Matches gazebo_ros2_controllers.yaml gait_position_controller joint list
_URDF_JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]


class GazeboControlBridgeNode:  # pragma: no cover - runtime node
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__("gazebo_control_bridge")
                self._cmd_pub = self.create_publisher(
                    Float64MultiArray,
                    "/gait_position_controller/commands",
                    10,
                )
                self._agg_pub = self.create_publisher(
                    JointState, "/joint_states_aggregated", 10
                )
                self.create_subscription(
                    JointState,
                    "/joint_commands",
                    self._on_joint_commands,
                    10,
                )
                self.create_subscription(
                    JointState,
                    "/joint_states",
                    self._on_joint_states,
                    10,
                )
                self.get_logger().info("gazebo_control_bridge ready")

            def _on_joint_states(self, msg: JointState) -> None:
                # Gazebo publishes with _joint suffix; strip it
                pos = {
                    n.removesuffix("_joint"): float(p)
                    for n, p in zip(msg.name, msg.position)
                }
                vel = {
                    n.removesuffix("_joint"): float(v)
                    for n, v in zip(msg.name, msg.velocity)
                }
                out = JointState()
                out.header.stamp = self.get_clock().now().to_msg()
                out.name = list(_URDF_JOINT_NAMES)
                out.position = [pos.get(n, 0.0) for n in _URDF_JOINT_NAMES]
                out.velocity = [vel.get(n, 0.0) for n in _URDF_JOINT_NAMES]
                self._agg_pub.publish(out)

            def _on_joint_commands(self, msg: JointState) -> None:
                # Match by name, send in Gazebo controller order
                pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
                out = Float64MultiArray()
                out.data = [pos.get(n, 0.0) for n in _URDF_JOINT_NAMES]
                self._cmd_pub.publish(out)

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
