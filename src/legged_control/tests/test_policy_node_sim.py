"""Tests for the Gazebo sim policy node's 46-dim obs assembly and action decode.

Sim works in policy/sim order (no reorder, no hip sign flip). The obs layout and
scaling must match the real obs_assembler — verified by a cross-check.
"""

import numpy as np

from legged_control.processing.obs_assembler import _assemble, reorder_yaml_to_policy
from legged_control.sim.policy_node_sim import (
    _SIM_NAMES,
    _assemble_obs,
    _decode_action,
)

_WIDE = (np.full(12, -9.0, np.float32), np.full(12, 9.0, np.float32))


def _inputs():
    state = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0, 0.0, -1.0], dtype=np.float32)
    cmd = (0.5, -0.3, 0.8)
    height = 0.25
    q_sim = np.arange(12, dtype=np.float32)
    dq_sim = np.arange(12, dtype=np.float32) + 100.0
    q_default = np.zeros(12, dtype=np.float32)
    last_action = np.linspace(-3.0, 3.0, 12, dtype=np.float32)
    return state, cmd, height, q_sim, dq_sim, q_default, last_action


def test_sim_obs_is_46_dim():
    obs = _assemble_obs(*_inputs())
    assert obs.shape == (46,)
    assert obs.dtype == np.float32


def test_sim_obs_scaling_and_height():
    s, c, h, qs, dqs, qd, la = _inputs()
    obs = _assemble_obs(s, c, h, qs, dqs, qd, la)
    np.testing.assert_allclose(obs[0:3], s[3:6] * 0.25, rtol=1e-6)  # ang_vel first, no lin_vel
    np.testing.assert_allclose(obs[6:9], [0.5 * 2.0, -0.3 * 2.0, 0.8 * 0.25], rtol=1e-6)
    assert obs[9] == np.float32(0.25)
    np.testing.assert_allclose(obs[22:34], dqs * 0.05, rtol=1e-6)


def test_sim_matches_real_assembler():
    """Sim (policy order) and real (yaml order + reorder) must agree."""
    s = np.array([0.5, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, -1.0], dtype=np.float32)
    cmd = (0.4, 0.1, -0.2)
    h = 0.21
    q_yaml = np.linspace(-0.3, 0.3, 12).astype(np.float32)
    dq_yaml = np.linspace(-1.0, 1.0, 12).astype(np.float32)
    qd_yaml = np.full(12, 0.05, dtype=np.float32)
    last_action = np.zeros(12, dtype=np.float32)

    real = _assemble(s, cmd, h, q_yaml, dq_yaml, qd_yaml, last_action)
    sim = _assemble_obs(
        s, cmd, h,
        reorder_yaml_to_policy(q_yaml),
        reorder_yaml_to_policy(dq_yaml),
        reorder_yaml_to_policy(qd_yaml),
        last_action,
    )
    np.testing.assert_allclose(sim, real, rtol=1e-6, atol=1e-6)


def test_sim_decode_zero_action_gives_default():
    q_def = np.array([0.1, 0.8, -1.5] * 4, dtype=np.float32)
    scale = np.full(12, 0.25, dtype=np.float32)
    q = _decode_action(np.zeros(12, dtype=np.float32), q_def, scale, [], *_WIDE)
    np.testing.assert_allclose(q, q_def, atol=1e-6)


def test_sim_decode_clamps_to_soft_limits():
    q_def = np.zeros(12, dtype=np.float32)
    scale = np.full(12, 100.0, dtype=np.float32)
    q_min = np.full(12, -0.4, dtype=np.float32)
    q_max = np.full(12, 0.4, dtype=np.float32)
    q = _decode_action(np.ones(12, dtype=np.float32), q_def, scale, [], q_min, q_max)
    assert np.all(q <= 0.4 + 1e-6) and np.all(q >= -0.4 - 1e-6)


def test_sim_names_match_policy_order():
    # sim order must equal the real policy joint order (leg-grouped FL/FR/RL/RR)
    from legged_control.processing.obs_assembler import _POLICY_JOINT_NAMES
    assert _SIM_NAMES == _POLICY_JOINT_NAMES
