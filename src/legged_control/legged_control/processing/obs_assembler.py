"""obs_assembler

Assembles the 46-dim SINGLE-FRAME observation for the B+C policy (no base
linear velocity; the actor is asymmetric, lin_vel was a privileged critic input
at training time). See docs/deployment_guide.md §2.

single-frame layout (obs_scales applied, matching training):
  [0:3]   base_ang_vel        * 0.25   from /state_estimate[3:6]
  [3:6]   projected_gravity   * 1.0    from /state_estimate[6:9]
  [6:9]   cmd (vx, vy, yaw)   * (2.0, 2.0, 0.25)   from /cmd_vel
  [9]     height command      * 1.0 (raw)          from /height_command
  [10:22] (q - q_default)     * 1.0    /joint_states_aggregated (yaml -> policy order)
  [22:34] dof_vel             * 0.05   /joint_states_aggregated (yaml -> policy order)
  [34:46] last_action         raw      /raw_policy_action (policy order)

Published on /observation. policy_node stacks 3 frames -> 138 before inference.
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
from std_msgs.msg import Float32, Float32MultiArray

SINGLE_OBS_DIM = 46

# obs_scales from legged_gym normalization.obs_scales (dog_urdf training config)
_LIN_VEL_SCALE = 2.0
_ANG_VEL_SCALE = 0.25
_DOF_POS_SCALE = 1.0
_DOF_VEL_SCALE = 0.05
# commands_scale = [lin_vel, lin_vel, ang_vel]; height command is appended unscaled
_CMD_SCALE = np.array([_LIN_VEL_SCALE, _LIN_VEL_SCALE, _ANG_VEL_SCALE], dtype=np.float32)

# Default height command before any /height_command arrives. Must stay inside the
# trained range [0.15, 0.28] (mid stance), else the policy sees an OOD height.
_DEFAULT_HEIGHT_CMD = 0.25

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

# dog_urdf DOF order (URDF joint declaration order = isaacgym DOF order).
# IMPORTANT: verify against hardware before first policy run.
_POLICY_JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]

_YAML_TO_POLICY = [_YAML_JOINT_NAMES.index(n) for n in _POLICY_JOINT_NAMES]


def reorder_yaml_to_policy(yaml_vec: np.ndarray) -> np.ndarray:
    return np.asarray(yaml_vec, dtype=np.float32)[_YAML_TO_POLICY]


def _assemble(
    state_estimate: np.ndarray,
    cmd_vel: tuple[float, float, float],
    height_cmd: float,
    joint_pos_urdf: np.ndarray,
    joint_vel_urdf: np.ndarray,
    q_default_urdf: np.ndarray,
    last_action: np.ndarray,
    sign_flip_policy_idx: list[int] | None = None,
) -> np.ndarray:
    state_estimate = np.asarray(state_estimate, dtype=np.float32)
    # state_estimate[0:3] is base_lin_vel — intentionally NOT used (B+C actor obs
    # excludes it; lin_vel was a privileged critic input at training time).
    ang_vel = state_estimate[3:6] * _ANG_VEL_SCALE
    proj_grav = state_estimate[6:9]
    cmd = np.asarray(cmd_vel, dtype=np.float32) * _CMD_SCALE

    joint_pos_rel = reorder_yaml_to_policy(
        np.asarray(joint_pos_urdf, dtype=np.float32) - np.asarray(q_default_urdf, dtype=np.float32)
    ) * _DOF_POS_SCALE
    joint_vel = reorder_yaml_to_policy(joint_vel_urdf) * _DOF_VEL_SCALE
    if sign_flip_policy_idx:
        for idx in sign_flip_policy_idx:
            joint_pos_rel[idx] *= -1.0
            joint_vel[idx] *= -1.0

    return np.clip(np.concatenate([
        ang_vel,
        proj_grav,
        cmd,
        np.array([height_cmd], dtype=np.float32),
        joint_pos_rel,
        joint_vel,
        np.asarray(last_action, dtype=np.float32),
    ]), -100.0, 100.0).astype(np.float32)


class ObsAssemblerNode(Node):
    def __init__(self) -> None:
        super().__init__("obs_assembler")

        self._q_default_urdf = self._load_q_default()
        self._sign_flip_policy_idx = self._load_sign_flip_policy_idx()
        self._state_estimate = np.zeros(9, dtype=np.float32)
        self._height_cmd = _DEFAULT_HEIGHT_CMD
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._joint_pos = np.zeros(12, dtype=np.float32)
        self._joint_vel = np.zeros(12, dtype=np.float32)
        self._last_action = np.zeros(12, dtype=np.float32)

        self._pub = self.create_publisher(Float32MultiArray, "/observation", 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32, "/height_command", self._on_height, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/raw_policy_action", self._on_raw_action, 10)

        self.create_timer(0.02, self._publish)
        self.get_logger().info(
            f"obs_assembler ready — /observation ({SINGLE_OBS_DIM} floats, single frame)")

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

    def _on_height(self, msg: Float32) -> None:
        self._height_cmd = float(msg.data)

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
            self._height_cmd,
            self._joint_pos,
            self._joint_vel,
            self._q_default_urdf,
            self._last_action,
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
