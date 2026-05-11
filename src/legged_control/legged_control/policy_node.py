"""policy_node — RL policy deployment node.

Runs a TorchScript policy at 50 Hz with the same PASSIVE->STANDUP->WAIT->POLICY->LIE_DOWN
state machine as gait_node. The POLICY phase replaces IK trot with neural-network inference.

Observation vector (373 dims):
  [0:3]   base_lin_vel     from /state_estimate[0:3]
  [3:6]   base_ang_vel     from /state_estimate[3:6]
  [6:9]   projected_gravity from /state_estimate[6:9]
  [9:12]  velocity_commands [vx, vy, omega_z] from /cmd_vel
  [12:24] joint_pos_rel    q_motor - q_default_motor (yaml order -> policy order)
  [24:36] joint_vel        dq_motor (yaml order -> policy order)
  [36:48] last_action      previous raw policy output (policy order)
  [48:373] height_scan     from /height_scan (325 floats)

Action: 12-dim (policy order) joint position residuals.
  q_target_urdf = q_default_urdf + sign_flip * action * scale
  q_target_motor = direction * (q_target_urdf - zero_offset)
"""

from __future__ import annotations

import numpy as np

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

_YAML_TO_POLICY = [
    _YAML_JOINT_NAMES.index(name) for name in _POLICY_JOINT_NAMES
]

_POLICY_TO_YAML = [
    _POLICY_JOINT_NAMES.index(name) for name in _YAML_JOINT_NAMES
]

_DEFAULT_HIP_SIGN_FLIP_POLICY_IDX = [
    _POLICY_JOINT_NAMES.index("FR_hip"),
    _POLICY_JOINT_NAMES.index("RL_hip"),
]


def _reorder_yaml_to_policy(yaml_vec: np.ndarray) -> np.ndarray:
    return yaml_vec[_YAML_TO_POLICY]


def _reorder_policy_to_yaml(policy_vec: np.ndarray) -> np.ndarray:
    return policy_vec[_POLICY_TO_YAML]


def _decode_action(
    action_policy: np.ndarray,
    q_default_urdf_yaml: np.ndarray,
    action_scale_yaml: np.ndarray,
    sign_flip_policy_idx: list[int],
    joint_cfg: dict[str, dict],
) -> np.ndarray:
    action = action_policy.copy()
    for idx in sign_flip_policy_idx:
        action[idx] *= -1.0

    action_yaml = _reorder_policy_to_yaml(action)
    q_target_urdf = q_default_urdf_yaml + action_yaml * action_scale_yaml

    q_motor = np.empty(12, dtype=np.float32)
    for i, name in enumerate(_YAML_JOINT_NAMES):
        cfg = joint_cfg[name]
        direction = float(cfg["direction"])
        zero_offset = float(cfg["zero_offset"])
        q_m = direction * (float(q_target_urdf[i]) - zero_offset)
        q_motor[i] = float(np.clip(q_m, float(cfg["q_min"]), float(cfg["q_max"])))

    return q_motor


def _assemble_obs(
    state_estimate: np.ndarray,
    cmd_vel: tuple[float, float, float],
    joint_pos_motor_yaml: np.ndarray,
    joint_vel_motor_yaml: np.ndarray,
    q_default_motor_yaml: np.ndarray,
    last_action_policy: np.ndarray,
    height_scan: np.ndarray,
) -> np.ndarray:
    joint_pos_rel_policy = _reorder_yaml_to_policy(
        joint_pos_motor_yaml - q_default_motor_yaml
    )
    joint_vel_policy = _reorder_yaml_to_policy(joint_vel_motor_yaml)

    obs = np.concatenate([
        state_estimate[:9],
        np.array(cmd_vel, dtype=np.float32),
        joint_pos_rel_policy,
        joint_vel_policy,
        last_action_policy,
        height_scan,
    ])
    return obs.astype(np.float32)
