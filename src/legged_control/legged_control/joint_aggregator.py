# src/legged_control/legged_control/joint_aggregator.py
"""
joint_aggregator

Subscribes to 12 individual /<ns>/joint_states topics published by motor_bus_node
and merges them into a single /joint_states_aggregated message.

Published immediately whenever any joint receives a new message.
Joint order matches robot.yaml joints list (FR_hip ... RL_calf, index 0-11).
All values are in motor frame (no direction/zero_offset conversion).

Effort field is intentionally not forwarded — downstream consumers (policy_node)
use only position and velocity.
"""

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' -> 'fr/hip'"""
    leg, joint = name.split("_", 1)
    return f"{leg.lower()}/{joint.lower()}"


def _freshness_status(latest: dict, now_mono: float, timeout: float) -> tuple:
    """Return lists of joints that are unseen or stale."""
    unseen = []
    stale = []
    for name, entry in latest.items():
        stamp = entry["stamp"]
        if stamp is None:
            unseen.append(name)
        elif now_mono - stamp > timeout:
            stale.append(name)
    return unseen, stale


class JointAggregatorNode(Node):
    _TIMEOUT = 0.5  # seconds before a joint is considered stale

    def __init__(self) -> None:
        super().__init__("joint_aggregator")

        cfg = self._load_config()
        self._names = [j["name"] for j in cfg["joints"]]
        self._latest: dict = {
            name: {"position": 0.0, "velocity": 0.0, "stamp": None}
            for name in self._names
        }

        self._pub = self.create_publisher(JointState, "/joint_states_aggregated", 10)

        for name in self._names:
            ns = _ns_from_joint_name(name)
            self.create_subscription(
                JointState,
                f"/{ns}/joint_states",
                lambda msg, n=name: self._on_joint_state(msg, n),
                10,
            )

        self.get_logger().info(
            f"Joint aggregator ready — tracking {len(self._names)} joints"
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_joint_state(self, msg: JointState, name: str) -> None:
        if msg.name and msg.name[0] == name:
            self._latest[name] = {
                "position": msg.position[0] if msg.position else 0.0,
                "velocity": msg.velocity[0] if msg.velocity else 0.0,
                "stamp": time.monotonic(),
            }
        self._publish()

    def _publish(self) -> None:
        now_mono = time.monotonic()
        unseen, stale = _freshness_status(self._latest, now_mono, self._TIMEOUT)
        if unseen or stale:
            details = []
            if unseen:
                details.append(f"unseen: {', '.join(unseen)}")
            if stale:
                details.append(f"stale: {', '.join(stale)}")
            self.get_logger().warn(
                "joint_aggregator: waiting for fresh data from all joints ("
                + "; ".join(details)
                + ")",
                throttle_duration_sec=2.0,
            )
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._names)
        out.position = []
        out.velocity = []
        # effort not forwarded — policy_node uses only position and velocity

        for name in self._names:
            entry = self._latest[name]
            out.position.append(float(entry["position"]))
            out.velocity.append(float(entry["velocity"]))

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JointAggregatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
