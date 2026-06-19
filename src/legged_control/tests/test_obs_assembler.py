"""Tests for the 49-dim observation assembly (dog_urdf blind policy).

obs layout (matches legged_gym compute_observations with include_lin_vel=True,
num_commands=5):
  [0:3]   base_lin_vel        * 2.0
  [3:6]   base_ang_vel        * 0.25
  [6:9]   projected_gravity   * 1.0
  [9:12]  cmd (vx, vy, yaw)   * (2.0, 2.0, 0.25)
  [12:13] height command      * 1.0 (raw)
  [13:25] (q - q_default)     * 1.0   (yaml -> policy order)
  [25:37] dof_vel             * 0.05  (yaml -> policy order)
  [37:49] last_action         (policy order, raw)
"""

import numpy as np

from legged_control.processing.obs_assembler import _assemble, reorder_yaml_to_policy


def _inputs():
    state = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.1, 0.2, -0.97], dtype=np.float32)
    cmd = (0.5, -0.3, 0.8)
    height_cmd = 0.22
    joint_pos = np.arange(12, dtype=np.float32)
    joint_vel = np.arange(12, dtype=np.float32) + 100.0
    q_default = np.zeros(12, dtype=np.float32)
    last_action = np.arange(12, dtype=np.float32) + 200.0
    return state, cmd, height_cmd, joint_pos, joint_vel, q_default, last_action


def test_obs_is_49_dim():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    assert obs.shape == (49,)
    assert obs.dtype == np.float32


def test_base_velocity_and_gravity_scaled():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[0:3], s[0:3] * 2.0, rtol=1e-6)
    np.testing.assert_allclose(obs[3:6], s[3:6] * 0.25, rtol=1e-6)
    np.testing.assert_allclose(obs[6:9], s[6:9], rtol=1e-6)  # gravity unscaled


def test_command_block_scaled_then_height_raw():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[9:12], [0.5 * 2.0, -0.3 * 2.0, 0.8 * 0.25], rtol=1e-6)
    assert obs[12] == np.float32(0.22)  # height command is not scaled


def test_joint_pos_rel_reordered_and_unscaled():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    expected = reorder_yaml_to_policy(jp - qd) * 1.0
    np.testing.assert_allclose(obs[13:25], expected, rtol=1e-6)


def test_joint_vel_reordered_and_scaled():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    expected = reorder_yaml_to_policy(jv) * 0.05
    np.testing.assert_allclose(obs[25:37], expected, rtol=1e-6)


def test_last_action_block_raw():
    s, c, h, jp, jv, qd, la = _inputs()
    obs = _assemble(s, c, h, jp, jv, qd, la)
    np.testing.assert_allclose(obs[37:49], la, rtol=1e-6)


def test_sign_flip_negates_pos_and_vel_for_listed_policy_indices():
    s, c, h, jp, jv, qd, la = _inputs()
    flip = [0]  # first policy joint
    plain = _assemble(s, c, h, jp, jv, qd, la, sign_flip_policy_idx=[])
    flipped = _assemble(s, c, h, jp, jv, qd, la, sign_flip_policy_idx=flip)
    assert flipped[13] == -plain[13]   # joint_pos_rel block
    assert flipped[25] == -plain[25]   # dof_vel block
