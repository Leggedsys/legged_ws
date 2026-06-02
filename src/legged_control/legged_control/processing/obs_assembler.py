"""obs_assembler

Subscribes to 5 observation-source topics and assembles the full
373-dim observation vector used by the policy.

This is the canonical obs assembly point — policy_node subscribes to
/observation for inference input.

Topics subscribed:
  /joint_states_aggregated    JointState         12 URDF-frame joints
  /state_estimate             Float32MultiArray  9 floats (lin_vel, ang_vel, proj_grav)
  /height_scan                Float32MultiArray  325 floats
  /cmd_vel                    Twist              velocity commands
  /raw_policy_action          Float32MultiArray  12 floats (raw policy output)
"""

from __future__ import annotations

import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

_POLICY_JOINT_NAMES = [
    "FL_hip", "FR_hip", "FL_thigh", "FR_thigh", "FL_calf", "FR_calf",
    "RL_hip", "RR_hip", "RL_thigh", "RR_thigh", "RL_calf", "RR_calf",
]

_YAML_TO_POLICY = [_YAML_JOINT_NAMES.index(n) for n in _POLICY_JOINT_NAMES]


def reorder_yaml_to_policy(yaml_vec: np.ndarray) -> np.ndarray:
    return yaml_vec[_YAML_TO_POLICY]


def _assemble(
    state_estimate: np.ndarray,
    cmd_vel: tuple[float, float, float],
    joint_pos_urdf: np.ndarray,
    joint_vel_urdf: np.ndarray,
    q_default_urdf: np.ndarray,
    last_action: np.ndarray,
    height_scan: np.ndarray,
    sign_flip_policy_idx: list[int] | None = None,
) -> np.ndarray:
    joint_pos_rel = reorder_yaml_to_policy(joint_pos_urdf - q_default_urdf)
    joint_vel = reorder_yaml_to_policy(joint_vel_urdf)
    if sign_flip_policy_idx:
        for idx in sign_flip_policy_idx:
            joint_pos_rel[idx] *= -1.0
            joint_vel[idx] *= -1.0
    return np.concatenate([
        state_estimate[:9],
        np.array(cmd_vel, dtype=np.float32),
        joint_pos_rel,
        joint_vel,
        last_action,
        height_scan,
    ]).astype(np.float32)


class ObsAssemblerNode(Node):
    def __init__(self) -> None:
        super().__init__("obs_assembler")

        self._q_default_urdf = self._load_q_default()
        self._sign_flip_policy_idx = self._load_sign_flip_policy_idx()
        self._state_estimate = np.zeros(9, dtype=np.float32)
        self._height_scan = np.zeros(325, dtype=np.float32)
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._joint_pos = np.zeros(12, dtype=np.float32)
        self._joint_vel = np.zeros(12, dtype=np.float32)
        self._last_action = np.zeros(12, dtype=np.float32)

        self._pub = self.create_publisher(Float32MultiArray, "/observation", 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32MultiArray, "/height_scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/raw_policy_action", self._on_raw_action, 10)

        self.create_timer(0.02, self._publish)
        self.get_logger().info("obs_assembler ready — /observation (373 floats)")

    def _load_q_default(self) -> np.ndarray:
        share = get_package_share_directory("legged_control")
        try:
            with open(os.path.join(share, "config", "policy.yaml")) as f:
                pcfg = yaml.safe_load(f).get("policy", {})
            m = pcfg.get("joint_default_q_urdf", {})
            return np.array([float(m.get(n, 0.0)) for n in _YAML_JOINT_NAMES],
                            dtype=np.float32)
        except Exception:
            return np.zeros(12, dtype=np.float32)

    def _load_sign_flip_policy_idx(self) -> list[int]:
        share = get_package_share_directory("legged_control")
        try:
            with open(os.path.join(share, "config", "policy.yaml")) as f:
                pcfg = yaml.safe_load(f).get("policy", {})
            flip_names = pcfg.get("hip_sign_flip", [])
            return [_POLICY_JOINT_NAMES.index(n) for n in flip_names if n in _POLICY_JOINT_NAMES]
        except Exception:
            return []

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_estimate = np.array(msg.data[:9], dtype=np.float32)

    def _on_scan(self, msg: Float32MultiArray) -> None:
        self._height_scan = np.array(msg.data[:325], dtype=np.float32)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _on_joints(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in _YAML_JOINT_NAMES:
                idx = _YAML_JOINT_NAMES.index(name)
                self._joint_pos[idx] = float(pos)
                self._joint_vel[idx] = float(vel)

    def _on_raw_action(self, msg: Float32MultiArray) -> None:
        self._last_action = np.array(msg.data[:12], dtype=np.float32)

    def _publish(self) -> None:
        obs = _assemble(
            self._state_estimate,
            self._cmd_vel,
            self._joint_pos,
            self._joint_vel,
            self._q_default_urdf,
            self._last_action,
            self._height_scan,
            self._sign_flip_policy_idx,
        )
        out = Float32MultiArray()
        out.data = obs.tolist()
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ObsAssemblerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
