"""Bridge aggregated joint states to URDF joint states for robot_state_publisher.

Input (/joint_states_aggregated) is already in URDF frame (joint_aggregator
handles motor→URDF conversion on real hardware).  Just renames to URDF joint
names (_joint suffix) for robot_state_publisher.
"""

from __future__ import annotations

from sensor_msgs.msg import JointState


def _joint_name_to_urdf_joint(name: str) -> str:
    return f"{name}_joint"


class URDFJointStateBridgeNode:  # pragma: no cover - exercised via ROS runtime
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__("urdf_joint_state_bridge")
                self._pub = self.create_publisher(JointState, "/joint_states", 10)
                self.create_subscription(
                    JointState, "/joint_states_aggregated", self._on_joint_states, 10
                )
                self.get_logger().info(
                    "urdf_joint_state_bridge ready — publishing /joint_states in URDF frame"
                )
                self._has_motor_data = False
                self.create_timer(1.0, self._publish_default_fallback)

            def _on_joint_states(self, msg: JointState) -> None:
                self._has_motor_data = True
                out = JointState()
                out.header.stamp = self.get_clock().now().to_msg()
                out.name = [_joint_name_to_urdf_joint(n) for n in msg.name]
                out.position = list(msg.position)
                out.velocity = list(msg.velocity)
                self._pub.publish(out)

            def _publish_default_fallback(self) -> None:
                if self._has_motor_data:
                    return
                out = JointState()
                out.header.stamp = self.get_clock().now().to_msg()
                out.name = [
                    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                ]
                out.position = [0.0] * 12
                out.velocity = [0.0] * 12
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
