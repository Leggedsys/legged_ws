import numpy as np
import pytest


def reorder_yaml_to_policy(yaml_vec):
    from legged_control.policy_node import _reorder_yaml_to_policy
    return _reorder_yaml_to_policy(np.array(yaml_vec, dtype=np.float32))


def reorder_policy_to_yaml(policy_vec):
    from legged_control.policy_node import _reorder_policy_to_yaml
    return _reorder_policy_to_yaml(np.array(policy_vec, dtype=np.float32))


def decode_action(action, q_default_urdf, action_scale, sign_flip_policy_idx, joint_cfg):
    from legged_control.policy_node import _decode_action
    return _decode_action(
        np.array(action, dtype=np.float32),
        np.array(q_default_urdf, dtype=np.float32),
        np.array(action_scale, dtype=np.float32),
        sign_flip_policy_idx,
        joint_cfg,
    )


_YAML_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]
_POLICY_NAMES = [
    "FL_hip", "FR_hip", "FL_thigh", "FR_thigh", "FL_calf", "FR_calf",
    "RL_hip", "RR_hip", "RL_thigh", "RR_thigh", "RL_calf", "RR_calf",
]


def test_reorder_yaml_to_policy_identity_check():
    yaml_vec = np.arange(12, dtype=np.float32)
    policy_vec = reorder_yaml_to_policy(yaml_vec)
    for pi, pname in enumerate(_POLICY_NAMES):
        yi = _YAML_NAMES.index(pname)
        assert policy_vec[pi] == yaml_vec[yi], (
            f"policy[{pi}]={pname}: expected yaml[{yi}]={yaml_vec[yi]}, got {policy_vec[pi]}"
        )


def test_reorder_policy_to_yaml_identity_check():
    original = np.random.rand(12).astype(np.float32)
    roundtrip = reorder_yaml_to_policy(reorder_policy_to_yaml(original))
    np.testing.assert_allclose(roundtrip, original, atol=1e-6)


def test_reorder_yaml_to_policy_shape():
    assert reorder_yaml_to_policy(np.zeros(12)).shape == (12,)


def _make_joint_cfg():
    return {
        "FR_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "FR_thigh": {"direction":  1, "zero_offset": -1.254, "q_min": -2.0, "q_max": 2.0},
        "FR_calf":  {"direction": -1, "zero_offset":  2.791, "q_min": -4.5, "q_max": 4.5},
        "FL_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "FL_thigh": {"direction":  1, "zero_offset":  1.254, "q_min": -2.0, "q_max": 2.0},
        "FL_calf":  {"direction": -1, "zero_offset": -2.791, "q_min": -4.5, "q_max": 4.5},
        "RR_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "RR_thigh": {"direction":  1, "zero_offset": -1.254, "q_min": -2.0, "q_max": 2.0},
        "RR_calf":  {"direction": -1, "zero_offset":  2.791, "q_min": -4.5, "q_max": 4.5},
        "RL_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "RL_thigh": {"direction":  1, "zero_offset":  1.254, "q_min": -2.0, "q_max": 2.0},
        "RL_calf":  {"direction": -1, "zero_offset": -2.791, "q_min": -4.5, "q_max": 4.5},
    }


def test_zero_action_gives_default_pose():
    cfg = _make_joint_cfg()
    q_def_urdf = np.array([0.0, 0.7, -1.2] * 4, dtype=np.float32)
    scale = np.array([0.15, 0.20, 0.15] * 4, dtype=np.float32)
    action_policy = np.zeros(12, dtype=np.float32)
    sign_flip = [1, 6]

    q_motor = decode_action(action_policy, q_def_urdf, scale, sign_flip, cfg)
    assert q_motor.shape == (12,)
    assert q_motor[0] == pytest.approx(0.0, abs=1e-5)


def test_sign_flip_applied_to_fr_hip():
    cfg = _make_joint_cfg()
    q_def_urdf = np.zeros(12, dtype=np.float32)
    scale = np.ones(12, dtype=np.float32)
    action_policy = np.zeros(12, dtype=np.float32)
    action_policy[1] = 0.5  # FR_hip in policy order = 1

    q_motor_flipped = decode_action(action_policy, q_def_urdf, scale, [1, 6], cfg)
    q_motor_plain = decode_action(action_policy, q_def_urdf, scale, [], cfg)

    assert q_motor_flipped[0] == pytest.approx(-0.5, abs=1e-5)
    assert q_motor_plain[0] == pytest.approx(0.5, abs=1e-5)


def test_output_clamped_to_q_limits():
    cfg = _make_joint_cfg()
    q_def_urdf = np.zeros(12, dtype=np.float32)
    scale = np.ones(12, dtype=np.float32) * 100.0
    action = np.zeros(12, dtype=np.float32)
    q_motor = decode_action(action, q_def_urdf, scale, [], cfg)
    for i, name in enumerate(["FR_hip", "FR_thigh", "FR_calf",
                               "FL_hip", "FL_thigh", "FL_calf",
                               "RR_hip", "RR_thigh", "RR_calf",
                               "RL_hip", "RL_thigh", "RL_calf"]):
        assert cfg[name]["q_min"] <= q_motor[i] <= cfg[name]["q_max"]
