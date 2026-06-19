import numpy as np
import pytest

from legged_control.policy_node import (
    _decode_action,
    _reorder_policy_to_yaml,
    _validate_obs,
)
from legged_control.processing.obs_assembler import _assemble


# Canonical orders (must match policy_node module)
_YAML_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]
_POLICY_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]


def decode_action(action, q_default_urdf, action_scale, sign_flip_policy_idx,
                  soft_q_min, soft_q_max):
    return _decode_action(
        np.array(action, dtype=np.float32),
        np.array(q_default_urdf, dtype=np.float32),
        np.array(action_scale, dtype=np.float32),
        sign_flip_policy_idx,
        np.array(soft_q_min, dtype=np.float32),
        np.array(soft_q_max, dtype=np.float32),
    )


# ── reorder ─────────────────────────────────────────────────────────────────

def test_reorder_policy_to_yaml_maps_by_name():
    policy_vec = np.arange(12, dtype=np.float32)
    yaml_vec = _reorder_policy_to_yaml(policy_vec)
    for yi, yname in enumerate(_YAML_NAMES):
        pi = _POLICY_NAMES.index(yname)
        assert yaml_vec[yi] == policy_vec[pi]


# ── action decode ─────────────────────────────────────────────────────────────

_WIDE = ([-9.0] * 12, [9.0] * 12)


def test_zero_action_gives_default_pose():
    q_def = np.array([0.1, 0.8, -1.5] * 4, dtype=np.float32)
    scale = np.full(12, 0.25, dtype=np.float32)
    q = decode_action(np.zeros(12), q_def, scale, [], *_WIDE)
    np.testing.assert_allclose(q, q_def, atol=1e-6)


def test_action_scaled_and_added_in_yaml_order():
    q_def = np.zeros(12, dtype=np.float32)
    scale = np.full(12, 0.25, dtype=np.float32)
    action_policy = np.zeros(12, dtype=np.float32)
    action_policy[0] = 4.0  # FL_hip (policy idx 0) -> yaml FL_hip
    q = decode_action(action_policy, q_def, scale, [], *_WIDE)
    yi = _YAML_NAMES.index("FL_hip")
    assert q[yi] == pytest.approx(1.0, abs=1e-5)  # 4.0 * 0.25


def test_sign_flip_negates_listed_policy_joint():
    q_def = np.zeros(12, dtype=np.float32)
    scale = np.ones(12, dtype=np.float32)
    action = np.zeros(12, dtype=np.float32)
    fr_hip_policy = _POLICY_NAMES.index("FR_hip")
    action[fr_hip_policy] = 0.5
    yi = _YAML_NAMES.index("FR_hip")
    q_flip = decode_action(action, q_def, scale, [fr_hip_policy], *_WIDE)
    q_plain = decode_action(action, q_def, scale, [], *_WIDE)
    assert q_flip[yi] == pytest.approx(-0.5, abs=1e-5)
    assert q_plain[yi] == pytest.approx(0.5, abs=1e-5)


def test_output_clamped_to_soft_limits():
    q_def = np.zeros(12, dtype=np.float32)
    scale = np.full(12, 100.0, dtype=np.float32)
    q_min = np.full(12, -0.4, dtype=np.float32)
    q_max = np.full(12, 0.4, dtype=np.float32)
    q = decode_action(np.ones(12), q_def, scale, [], q_min, q_max)
    assert np.all(q <= 0.4 + 1e-6) and np.all(q >= -0.4 - 1e-6)


# ── obs validation (49-dim) ───────────────────────────────────────────────────

def _nominal_obs():
    """A benign, in-range 49-dim obs built via the real assembler."""
    state = np.array([0, 0, 0, 0, 0, 0, 0.0, 0.0, -1.0], dtype=np.float32)
    return _assemble(
        state, (0.0, 0.0, 0.0), 0.22,
        np.zeros(12, dtype=np.float32), np.zeros(12, dtype=np.float32),
        np.zeros(12, dtype=np.float32), np.zeros(12, dtype=np.float32),
    )


def test_validate_obs_takes_single_49dim_arg_and_passes_nominal():
    obs = _nominal_obs()
    assert obs.shape == (49,)
    assert _validate_obs(obs) == []


def test_validate_obs_flags_out_of_range_velocity_command():
    obs = _nominal_obs()
    obs[9] = 50.0  # absurd scaled vx command
    assert _validate_obs(obs) != []


def test_validate_obs_flags_nan():
    obs = _nominal_obs()
    obs[3] = np.nan
    assert _validate_obs(obs) != []
