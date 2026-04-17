import math

import pytest

from legged_control.gait_node import (
    _blend_targets,
    _body_to_hip,
    _clamp_cmd_vel,
    _command_is_fresh,
    _command_with_timeout,
    _foot_target,
    _is_effectively_zero_cmd,
    _leg_stride,
    _motor_targets_from_urdf,
    _phase_is_stance,
    _rate_limit_targets,
)


def test_command_with_timeout_returns_zero_when_stale():
    assert _command_with_timeout((0.1, 0.2, 0.3), 1.0, 2.1, 1.0) == (0.0, 0.0, 0.0)


def test_command_with_timeout_returns_latest_when_fresh():
    assert _command_with_timeout((0.1, 0.2, 0.3), 1.0, 1.5, 1.0) == (0.1, 0.2, 0.3)


def test_command_is_fresh():
    assert _command_is_fresh(1.0, 1.5, 1.0) is True
    assert _command_is_fresh(1.0, 2.1, 1.0) is False
    assert _command_is_fresh(None, 2.1, 1.0) is False


def test_phase_is_stance_boundary():
    assert _phase_is_stance(0.0) is True
    assert _phase_is_stance(math.pi - 1e-6) is True
    assert _phase_is_stance(math.pi) is False


def test_is_effectively_zero_cmd():
    assert _is_effectively_zero_cmd((0.0, 0.0, 0.0)) is True
    assert _is_effectively_zero_cmd((1e-7, -1e-7, 1e-7)) is True
    assert _is_effectively_zero_cmd((0.0, 0.0, 1e-3)) is False


def test_body_to_hip_fl_nominal_vertical_drop():
    x, y, z = _body_to_hip("FL", (0.1426, 0.1592, -0.28))
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.11266, abs=1e-4)
    assert z == pytest.approx(-0.28)


def test_leg_stride_yaw_creates_opposite_lateral_motion_front_legs():
    fl = _leg_stride((0.14, 0.16), "FL", 0.0, 0.0, 0.5, 2.0, 0.08, 0.04)
    fr = _leg_stride((0.14, -0.16), "FR", 0.0, 0.0, 0.5, 2.0, 0.08, 0.04)
    assert fl[0] < 0.0
    assert fr[0] > 0.0
    assert fl[1] > 0.0
    assert fr[1] > 0.0


def test_foot_target_uses_stride_y_in_stance():
    x, y, z = _foot_target((0.1426, 0.1592, -0.28), "FL", 0.0, 0.0, 0.06, 0.04, 0.02)
    assert x == pytest.approx(0.1426 + 0.02, abs=1e-4)
    assert y == pytest.approx(0.1592 + 0.01, abs=1e-4)
    assert z == pytest.approx(-0.28)


def test_clamp_cmd_vel_limits_each_axis():
    result = _clamp_cmd_vel((0.4, -0.2, 1.0), 0.15, 0.08, 0.4)
    assert result == pytest.approx((0.15, -0.08, 0.4))


def test_rate_limit_targets_caps_step_change():
    result = _rate_limit_targets([0.0, 1.0], [1.0, -1.0], 0.2)
    assert result == pytest.approx([0.2, 0.8])


def test_rate_limit_targets_passes_through_without_history():
    result = _rate_limit_targets(None, [0.1, -0.2], 0.05)
    assert result == pytest.approx([0.1, -0.2])


def test_blend_targets_interpolates_lists():
    result = _blend_targets([0.0, 1.0], [1.0, -1.0], 0.25)
    assert result == pytest.approx([0.25, 0.5])


def test_motor_targets_from_urdf_applies_conversion_and_clipping():
    joints = [
        {"direction": 1, "zero_offset": 0.0, "q_min": -0.5, "q_max": 0.5},
        {"direction": 1, "zero_offset": 1.0, "q_min": -0.2, "q_max": 0.2},
        {"direction": -1, "zero_offset": 2.0, "q_min": 0.0, "q_max": 1.0},
    ]
    targets, clipped = _motor_targets_from_urdf(joints, (0.1, 1.5, 0.5))
    assert targets == pytest.approx([0.1, 0.2, 1.0])
    assert clipped is True
