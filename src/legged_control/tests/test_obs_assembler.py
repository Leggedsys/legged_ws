"""Tests for the 46-dim single-frame observation (B+C policy, no base_lin_vel).

Single-frame layout (matches deployment_guide.md §2; obs_scales applied):
  [0:3]   base_ang_vel        * 0.25   from /state_estimate[3:6]
  [3:6]   projected_gravity   * 1.0    from /state_estimate[6:9]
  [6:9]   cmd (vx, vy, yaw)   * (2.0, 2.0, 0.25)
  [9]     height command      * 1.0 (raw)
  [10:22] (q - q_default)     * 1.0    (yaml -> policy order)
  [22:34] dof_vel             * 0.05   (yaml -> policy order)
  [34:46] last_action         (policy order, raw)

Frame stacking (single 46 -> 138) is done in policy_node, not here.
"""

import numpy as np

from legged_control.processing.obs_assembler import (
    SINGLE_OBS_DIM,
    _assemble,
    reorder_yaml_to_policy,
)


def _inputs():
    # state_estimate: [lin_vel(3, ignored), ang_vel(3), proj_grav(3)]
    state = np.array([9.0, 9.0, 9.0, 4.0, 5.0, 6.0, 0.1, 0.2, -0.97], dtype=np.float32)
    cmd = (0.5, -0.3, 0.8)
    height_cmd = 0.25
    joint_pos = np.arange(12, dtype=np.float32)
    joint_vel = np.arange(12, dtype=np.float32) + 100.0
    q_default = np.zeros(12, dtype=np.float32)
    last_action = np.linspace(-3.0, 3.0, 12, dtype=np.float32)  # realistic policy output
    return state, cmd, height_cmd, joint_pos, joint_vel, q_default, last_action


def test_single_obs_is_46_dim():
    obs = _assemble(*_inputs())
    assert SINGLE_OBS_DIM == 46
    assert obs.shape == (46,)
    assert obs.dtype == np.float32


def test_no_base_lin_vel_ang_vel_first():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    # block [0:3] must be ang_vel (state[3:6]) * 0.25, NOT lin_vel (state[0:3])
    np.testing.assert_allclose(obs[0:3], s[3:6] * 0.25, rtol=1e-6)


def test_gravity_unscaled():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[3:6], s[6:9], rtol=1e-6)


def test_command_block_then_height():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[6:9], [0.5 * 2.0, -0.3 * 2.0, 0.8 * 0.25], rtol=1e-6)
    assert obs[9] == np.float32(0.25)


def test_joint_blocks():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[10:22], reorder_yaml_to_policy(jp - qd) * 1.0, rtol=1e-6)
    np.testing.assert_allclose(obs[22:34], reorder_yaml_to_policy(jv) * 0.05, rtol=1e-6)


def test_last_action_block_raw():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[34:46], la, rtol=1e-6)


def test_obs_is_clipped_to_100():
    s, c, h, jp, jv, qd, la = _inputs()
    # ang_vel huge -> would exceed 100 after scaling? force via gravity slot
    s = s.copy()
    s[6] = 500.0  # proj_grav x (unscaled) -> clipped to 100
    obs = _assemble(s, c, h, jp, jv, qd, la)
    assert obs.max() <= 100.0 and obs.min() >= -100.0


def test_sign_flip_negates_pos_and_vel():
    s, c, h, jp, jv, qd, la = _inputs()
    plain = _assemble(s, c, h, jp, jv, qd, la, sign_flip_policy_idx=[])
    flipped = _assemble(s, c, h, jp, jv, qd, la, sign_flip_policy_idx=[0])
    assert flipped[10] == -plain[10]   # joint_pos_rel block
    assert flipped[22] == -plain[22]   # dof_vel block
