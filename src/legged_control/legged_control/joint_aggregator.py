"""
joint_aggregator_node

Subscribes to each motor's individual /*/joint_states topic and merges them
into a single /joint_states_aggregated message with joints ordered according
to the canonical joint list in robot.yaml.

Published topic:
  /joint_states_aggregated  (sensor_msgs/JointState)
    name:     [FR_hip, FR_thigh, ..., RL_calf]  (12 joints, fixed order)
    position: joint angles in radians
    velocity: joint velocities in rad/s
    effort:   joint torques in N·m
    header.stamp: stamp of the most recently received joint update
"""

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os


# Topic published by each motor node (within its namespace)
_MOTOR_TOPIC = 'joint_states'

# Namespace template: motor node is launched under e.g. /fr/hip/motor
# and publishes to /fr/hip/joint_states
_NAMESPACES = [
    'fr/hip', 'fr/thigh', 'fr/calf',
    'fl/hip', 'fl/thigh', 'fl/calf',
    'rr/hip', 'rr/thigh', 'rr/calf',
    'rl/hip', 'rl/thigh', 'rl/calf',
]


class JointAggregator(Node):
    def __init__(self) -> None:
        super().__init__('joint_aggregator')

        cfg = self._load_config()
        self._joint_names: list[str] = [j['name'] for j in cfg['joints']]
        n = len(self._joint_names)

        # Latest state per joint (index matches self._joint_names)
        self._pos = [0.0] * n
        self._vel = [0.0] * n
        self._eff = [0.0] * n
        self._received = [False] * n

        self._pub = self.create_publisher(JointState, '/joint_states_aggregated', 10)

        # Subscribe to each motor namespace
        self._subs = []
        for ns in _NAMESPACES:
            topic = f'/{ns}/{_MOTOR_TOPIC}'
            sub = self.create_subscription(
                JointState, topic, self._make_cb(topic), 10)
            self._subs.append(sub)

        self.get_logger().info(
            f'Aggregating {n} joints: {self._joint_names}')

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        path = os.path.join(share, 'config', 'robot.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    def _make_cb(self, topic: str):
        def cb(msg: JointState) -> None:
            for i, name in enumerate(msg.name):
                if name in self._joint_names:
                    idx = self._joint_names.index(name)
                    if idx < len(msg.position):
                        self._pos[idx] = msg.position[idx]
                    if idx < len(msg.velocity):
                        self._vel[idx] = msg.velocity[idx]
                    if idx < len(msg.effort):
                        self._eff[idx] = msg.effort[idx]
                    self._received[idx] = True
            self._publish(msg.header.stamp)
        return cb

    def _publish(self, stamp) -> None:
        out = JointState()
        out.header.stamp = stamp
        out.name = self._joint_names
        out.position = list(self._pos)
        out.velocity = list(self._vel)
        out.effort = list(self._eff)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JointAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
