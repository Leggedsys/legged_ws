import sys
import os
import math

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legged_control.position_gait_node import (
    CALF_FORWARD_SIGN,
    HIP_OUTWARD_SIGN,
    THIGH_FORWARD_SIGN,
    _activity,
    _clamp_unit,
    _clip_targets,
    _command_with_timeout,
    _joint_targets,
    _normalized_drive,
)


def test_clamp_unit_limits_range():
    assert _clamp_unit(2.0) == 1.0
    assert _clamp_unit(-2.0) == -1.0
    assert _clamp_unit(0.25) == 0.25


def test_normalized_drive_uses_vx_and_yaw_only():
    vx, yaw = _normalized_drive(np.array([0.5, 99.0, -0.5]), max_vx=1.0, max_yaw=1.0)
    assert vx == 0.5
    assert yaw == -0.5


def test_activity_deadband():
    assert _activity(0.04, 0.03, min_command=0.05) == 0.0
    assert _activity(0.1, 0.0, min_command=0.05) == 0.1


def test_command_with_timeout_returns_zero_when_stale():
    cmd = np.array([0.3, 0.0, -0.2], dtype=np.float32)
    result = _command_with_timeout(cmd, last_cmd_t=1.0, now=1.6, timeout=0.5)
    np.testing.assert_allclose(result, np.zeros(3, dtype=np.float32), atol=1e-6)


def test_clip_targets_uses_joint_specific_ranges():
    names = ["FR_hip", "FR_thigh", "FR_calf"]
    targets = np.array([0.3, -0.7, 0.8], dtype=np.float32)
    safe_ranges = {
        "hip": (-0.2, 0.2),
        "thigh": (-0.52, 0.52),
        "calf": (-0.52, 0.52),
    }
    clipped = _clip_targets(targets, names, safe_ranges)
    np.testing.assert_allclose(clipped, [0.2, -0.52, 0.52], atol=1e-6)


def test_joint_targets_zero_command_stays_zero():
    names = ["FR_hip", "FR_thigh", "FR_calf"]
    targets = _joint_targets(0.0, 0.0, 0.0, names, 0.1, 0.2, 0.3)
    np.testing.assert_allclose(targets, [0.0, 0.0, 0.0], atol=1e-6)


def test_joint_targets_forward_stride_uses_measured_thigh_signs():
    names = ["FL_thigh", "RR_thigh"]
    targets = _joint_targets(math.pi / 2.0, 1.0, 0.0, names, 0.1, 0.2, 0.3)
    assert np.sign(targets[0]) == THIGH_FORWARD_SIGN["FL"]
    assert np.sign(targets[1]) == THIGH_FORWARD_SIGN["RR"]


def test_joint_targets_forward_stride_uses_measured_calf_signs():
    names = ["FL_calf", "RR_calf"]
    targets = _joint_targets(math.pi / 2.0, 1.0, 0.0, names, 0.1, 0.2, 0.3)
    assert np.sign(targets[0]) == CALF_FORWARD_SIGN["FL"]
    assert np.sign(targets[1]) == CALF_FORWARD_SIGN["RR"]


def test_joint_targets_turn_command_uses_measured_hip_signs():
    names = ["FL_hip", "RR_hip"]
    targets = _joint_targets(math.pi / 2.0, 0.0, 0.5, names, 0.1, 0.2, 0.3)
    assert np.sign(targets[0]) == -HIP_OUTWARD_SIGN["FL"]
    assert np.sign(targets[1]) == HIP_OUTWARD_SIGN["RR"]


def test_joint_targets_lift_term_is_positive_for_known_swing_legs():
    names = ["FL_thigh", "FL_calf", "RR_thigh", "RR_calf"]
    targets = _joint_targets(0.0, 1.0, 0.0, names, 0.1, 0.2, 0.3)
    assert np.all(targets > 0.0)
