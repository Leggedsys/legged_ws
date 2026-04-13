# src/legged_control/tests/test_policy_node.py
"""Tests for policy_node pure helper functions (no ROS2 or torch required)."""
import numpy as np
import pytest


# ─── _quat_rotate_inverse ───────────────────────────────────────────────────

def test_quat_rotate_inverse_identity():
    """Identity quaternion: vector passes through unchanged."""
    from legged_control.policy_node import _quat_rotate_inverse
    q = np.array([1.0, 0.0, 0.0, 0.0])   # [w, x, y, z]
    v = np.array([0.0, 0.0, -1.0])
    np.testing.assert_allclose(_quat_rotate_inverse(q, v), v, atol=1e-6)


def test_quat_rotate_inverse_upright_gravity():
    """Upright robot (identity quat): g_body should be [0, 0, -1]."""
    from legged_control.policy_node import _quat_rotate_inverse
    q = np.array([1.0, 0.0, 0.0, 0.0])
    g_body = _quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0]))
    np.testing.assert_allclose(g_body, [0.0, 0.0, -1.0], atol=1e-6)


def test_quat_rotate_inverse_90deg_pitch():
    """90° forward pitch: gravity should shift to +x body axis."""
    from legged_control.policy_node import _quat_rotate_inverse
    import math
    c, s = math.cos(math.pi / 4), math.sin(math.pi / 4)
    q = np.array([c, 0.0, s, 0.0])
    g_body = _quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0]))
    np.testing.assert_allclose(g_body[0],  1.0, atol=1e-6)
    np.testing.assert_allclose(g_body[2],  0.0, atol=1e-6)


# ─── _motor_to_urdf ─────────────────────────────────────────────────────────

def test_motor_to_urdf_positive_direction_with_offset():
    from legged_control.policy_node import _motor_to_urdf
    directions   = np.array([1.0, 1.0])
    zero_offsets = np.array([0.0, 1.254])
    q_motor      = np.array([0.1, 0.0])
    dq_motor     = np.array([0.5, 0.2])
    q_urdf, dq_urdf = _motor_to_urdf(q_motor, dq_motor, directions, zero_offsets)
    np.testing.assert_allclose(q_urdf,  [0.1,  1.254], atol=1e-6)
    np.testing.assert_allclose(dq_urdf, [0.5,  0.2],   atol=1e-6)


def test_motor_to_urdf_negative_direction():
    from legged_control.policy_node import _motor_to_urdf
    directions   = np.array([-1.0])
    zero_offsets = np.array([-1.221])
    q_motor      = np.array([0.2])
    dq_motor     = np.array([1.0])
    q_urdf, dq_urdf = _motor_to_urdf(q_motor, dq_motor, directions, zero_offsets)
    np.testing.assert_allclose(q_urdf,  [-0.2 - 1.221], atol=1e-6)
    np.testing.assert_allclose(dq_urdf, [-1.0],          atol=1e-6)


# ─── _decode_action ─────────────────────────────────────────────────────────

def test_decode_action_zero_returns_default_motor():
    """Zero action → target_q_motor should equal default_q in motor frame (0.0)."""
    from legged_control.policy_node import _decode_action
    directions     = np.array([1.0, -1.0])
    zero_offsets   = np.array([0.0, -1.221])
    default_q_urdf = np.array([0.0, -1.221])   # = zero_offset (since default_q_motor=0)
    scales         = np.array([0.04, 0.03])
    # q_min/q_max are motor-frame values from robot.yaml
    q_mins_motor   = np.array([-1.0, 0.0])
    q_maxs_motor   = np.array([ 1.0, 5.554])
    action         = np.array([0.0, 0.0])

    result = _decode_action(action, default_q_urdf, scales, q_mins_motor, q_maxs_motor,
                            directions, zero_offsets)
    # target_q_urdf = [0, -1.221]
    # target_q_motor = ([0,-1.221] - [0,-1.221]) / [1,-1] = [0, 0]
    # clip([0,0], motor limits) = [0, 0]
    np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-6)


def test_decode_action_clips_in_motor_frame():
    """Large action clipped AFTER converting to motor frame (direction=+1 case)."""
    from legged_control.policy_node import _decode_action
    directions     = np.array([1.0])
    zero_offsets   = np.array([0.0])
    default_q_urdf = np.array([0.0])
    scales         = np.array([0.5])
    q_mins_motor   = np.array([-0.1])   # motor-frame limits
    q_maxs_motor   = np.array([ 0.1])
    action         = np.array([100.0])   # huge

    result = _decode_action(action, default_q_urdf, scales, q_mins_motor, q_maxs_motor,
                            directions, zero_offsets)
    # target_q_urdf = 0 + 0.5×100 = 50
    # target_q_motor = (50 - 0) / 1 = 50
    # clip(50, -0.1, 0.1) = 0.1
    np.testing.assert_allclose(result, [0.1], atol=1e-6)


def test_decode_action_negative_direction_clips_motor_frame():
    """Clip in motor frame for negative-direction joint."""
    from legged_control.policy_node import _decode_action
    directions     = np.array([-1.0])
    zero_offsets   = np.array([1.221])
    default_q_urdf = np.array([1.221])   # direction×0 + 1.221
    scales         = np.array([0.03])
    q_mins_motor   = np.array([0.0])    # motor-frame limits (from robot.yaml)
    q_maxs_motor   = np.array([5.554])
    action         = np.array([-100.0]) # large negative action

    result = _decode_action(action, default_q_urdf, scales, q_mins_motor, q_maxs_motor,
                            directions, zero_offsets)
    # target_q_urdf = 1.221 + 0.03×(-100) = -1.779
    # target_q_motor = (-1.779 - 1.221) / (-1) = 3.0
    # clip(3.0, 0.0, 5.554) = 3.0
    np.testing.assert_allclose(result, [3.0], atol=1e-6)


# ─── _build_joint_params ────────────────────────────────────────────────────

def test_build_joint_params_shape():
    """_build_joint_params should return six arrays of shape (12,)."""
    from legged_control.policy_node import _build_joint_params, JOINT_ORDER
    import os, yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    result = _build_joint_params(cfg)
    assert len(result) == 6
    for arr in result:
        assert arr.shape == (12,), f"Expected (12,), got {arr.shape}"


def test_build_joint_params_fr_hip_scale():
    """FR_hip (front/hip) should have scale 0.04."""
    from legged_control.policy_node import _build_joint_params, JOINT_ORDER
    import os, yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    _, _, _, _, _, scales = _build_joint_params(cfg)
    fr_hip_idx = JOINT_ORDER.index('FR_hip')
    assert scales[fr_hip_idx] == pytest.approx(0.04)


def test_build_joint_params_rr_calf_scale():
    """RR_calf (rear/calf) should have scale 0.04."""
    from legged_control.policy_node import _build_joint_params, JOINT_ORDER
    import os, yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    _, _, _, _, _, scales = _build_joint_params(cfg)
    rr_calf_idx = JOINT_ORDER.index('RR_calf')
    assert scales[rr_calf_idx] == pytest.approx(0.04)
