"""Simulated motor bus node for RViz-based position control testing."""

from __future__ import annotations

import time


def _ns_from_joint_name(name: str) -> str:
    leg, joint = name.split("_", 1)
    return f"{leg.lower()}/{joint.lower()}"


def _targets_for_joint_names(
    joint_names: list[str], msg_names: list[str], msg_positions: list[float]
) -> dict[str, float]:
    targets = {name: 0.0 for name in joint_names}
    for name, pos in zip(msg_names, msg_positions):
        if name in targets:
            targets[name] = float(pos)
    return targets


class FakeMotorBusNode:  # pragma: no cover - exercised via ROS runtime
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        class _Node(Node):
            def __init__(self) -> None:
                super().__init__("fake_motor_bus_node")
                self.declare_parameter("joint_names", [""])
                self.declare_parameter("loop_hz", 250.0)
                self.declare_parameter("kp", 0.0)
                self.declare_parameter("kd", 0.0)

                self._joint_names = [
                    name for name in self.get_parameter("joint_names").value if name
                ]
                if not self._joint_names:
                    raise RuntimeError(
                        "fake_motor_bus_node: joint_names must not be empty"
                    )

                self._targets = {name: 0.0 for name in self._joint_names}
                self._last_targets = {name: 0.0 for name in self._joint_names}
                self._last_cmd_t = time.monotonic()

                self._pubs = {
                    name: self.create_publisher(
                        JointState, f"/{_ns_from_joint_name(name)}/joint_states", 10
                    )
                    for name in self._joint_names
                }
                self.create_subscription(
                    JointState, "/joint_commands", self._on_joint_commands, 10
                )
                self.create_timer(
                    1.0 / float(self.get_parameter("loop_hz").value), self._tick
                )
                self.get_logger().info(
                    f"fake_motor_bus_node ready — publishing {len(self._joint_names)} simulated joints"
                )

            def _on_joint_commands(self, msg: JointState) -> None:
                self._targets = _targets_for_joint_names(
                    self._joint_names, list(msg.name), list(msg.position)
                )
                self._last_cmd_t = time.monotonic()

            def _tick(self) -> None:
                now = time.monotonic()
                dt = max(now - self._last_cmd_t, 1e-6)
                for name in self._joint_names:
                    pos = self._targets[name]
                    vel = (pos - self._last_targets[name]) / dt
                    self._last_targets[name] = pos
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = [name]
                    msg.position = [pos]
                    msg.velocity = [vel]
                    msg.effort = [0.0]
                    self._pubs[name].publish(msg)

        self._rclpy = rclpy
        self._node = _Node()

    def spin(self) -> None:
        self._rclpy.spin(self._node)

    def shutdown(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()


def main() -> None:
    import rclpy

    rclpy.init()
    app = FakeMotorBusNode()
    try:
        app.spin()
    finally:
        app.shutdown()
