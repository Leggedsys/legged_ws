import math

import pytest

from legged_control.gait_node import (
    _blend_targets,
    _body_to_hip,
    _clamp_cmd_vel,
    _clamp_height,
    _command_is_fresh,
    _command_with_timeout,
    _foot_target,
    _has_motion_command,
    _ik_derived_targets,
    _is_effectively_zero_cmd,
    _leg_stride,
    _motor_targets_from_urdf,
    _phase_is_stance,
    _rate_limit_targets,
    GaitNode,
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


def test_has_motion_command_is_inverse_of_zero_check():
    assert _has_motion_command((0.0, 0.0, 0.0)) is False
    assert _has_motion_command((0.01, 0.0, 0.0)) is True


def test_body_to_hip_fl_nominal_vertical_drop():
    x, y, z = _body_to_hip("FL", (0.1426, 0.1592, -0.28))
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.11266, abs=1e-4)
    assert z == pytest.approx(-0.28)


def test_leg_stride_yaw_creates_opposite_lateral_motion_front_legs():
    fl = _leg_stride((0.14, 0.16), "FL", 0.0, 0.0, 0.5, 2.0, 0.08, 0.04, 1.0)
    fr = _leg_stride((0.14, -0.16), "FR", 0.0, 0.0, 0.5, 2.0, 0.08, 0.04, 1.0)
    assert fl[0] < 0.0
    assert fr[0] > 0.0
    assert fl[1] > 0.0
    assert fr[1] > 0.0


def test_leg_stride_yaw_scale_increases_turning_stride():
    base = _leg_stride((0.14, 0.16), "FL", 0.0, 0.0, 0.5, 2.0, 0.08, 0.04, 1.0)
    scaled = _leg_stride((0.14, 0.16), "FL", 0.0, 0.0, 0.5, 2.0, 0.08, 0.04, 1.8)
    assert abs(scaled[0]) > abs(base[0])
    assert abs(scaled[1]) > abs(base[1])


def test_foot_target_uses_stride_y_in_stance():
    x, y, z = _foot_target((0.1426, 0.1592, -0.28), "FL", 0.0, 0.28, 0.06, 0.04, 0.02)
    assert x == pytest.approx(0.1426 + 0.02, abs=1e-4)
    assert y == pytest.approx(0.1592 + 0.01, abs=1e-4)
    assert z == pytest.approx(-0.28)


def test_foot_target_swing_midpoint_returns_to_nominal_xy_and_peak_height():
    x, y, z = _foot_target(
        (0.1426, 0.1592, -0.28), "FL", 1.5 * math.pi, 0.28, 0.06, 0.04, 0.02
    )
    assert x == pytest.approx(0.1426, abs=1e-4)
    assert y == pytest.approx(0.1592, abs=1e-4)
    assert z == pytest.approx(-0.22, abs=1e-4)


def test_foot_target_swing_endpoints_return_to_stance_height():
    for phase in (math.pi, 2.0 * math.pi):
        _, _, z = _foot_target(
            (0.1426, 0.1592, -0.28), "FL", phase, 0.28, 0.06, 0.04, 0.02
        )
        assert z == pytest.approx(-0.28, abs=1e-4)


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


# ── _ik_derived_targets ──────────────────────────────────────────────────────

_FL_JOINTS = [
    {"direction": 1, "zero_offset": 0.0, "q_min": -0.533, "q_max": 0.467},
    {"direction": 1, "zero_offset": 1.254, "q_min": -3.133, "q_max": 0.663},
    {"direction": -1, "zero_offset": -2.791, "q_min": -5.554, "q_max": 0.000},
]
# q_urdf = direction * default_q + zero_offset
_FL_Q_URDF = (0.0, 1 * (-0.37373) + 1.254, -1 * (-1.488672) + (-2.791))
# = (0.0, 0.88027, -1.302328)
_FL_NOMINAL_XY = (0.1426, 0.1592)  # body-frame x,y from FK
_FL_NOMINAL_H = 0.280  # approximate FK stance height


def test_ik_derived_targets_fl_roundtrip():
    """IK at the nominal FK height should recover the original motor angles."""
    targets = _ik_derived_targets(
        "FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, _FL_NOMINAL_H
    )
    assert targets is not None
    assert targets[0] == pytest.approx(0.0, abs=1e-2)
    assert targets[1] == pytest.approx(-0.37373, abs=1e-2)
    assert targets[2] == pytest.approx(-1.488672, abs=1e-2)


def test_ik_derived_targets_lower_height_changes_calf():
    """Lowering stance height bends the knee; calf motor angle must change."""
    targets_nom = _ik_derived_targets(
        "FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, 0.280
    )
    targets_lower = _ik_derived_targets(
        "FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, 0.220
    )
    assert targets_nom is not None
    assert targets_lower is not None
    assert not math.isclose(targets_nom[2], targets_lower[2], abs_tol=0.01)
    assert not math.isclose(targets_nom[1], targets_lower[1], abs_tol=0.01)


def test_ik_derived_targets_returns_none_when_unreachable():
    """h=0.40 m exceeds leg reach -> IK returns None."""
    targets = _ik_derived_targets("FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, 0.40)
    assert targets is None


# ── _clamp_height ────────────────────────────────────────────────────────────


def test_clamp_height_no_clamp():
    assert _clamp_height(0.28, 0.03, 0.02, 0.20, 0.35) == pytest.approx(0.2806)


def test_clamp_height_clamps_max():
    # 0.345 + 0.03 * 1.0 = 0.375 > h_max=0.35 → clamped to 0.35
    assert _clamp_height(0.345, 0.03, 1.0, 0.20, 0.35) == pytest.approx(0.35)


def test_clamp_height_clamps_min():
    # 0.205 + (-0.03) * 1.0 = 0.175 < h_min=0.20 → clamped to 0.20
    assert _clamp_height(0.205, -0.03, 1.0, 0.20, 0.35) == pytest.approx(0.20)


def test_tracking_error_exceeded_uses_joint_feedback_against_targets():
    node = GaitNode.__new__(GaitNode)
    node._joint_names = ["FR_hip", "FR_thigh"]
    node._latest_joint_pos = {"FR_hip": 0.0, "FR_thigh": 0.25}
    assert node._tracking_error_exceeded([0.0, 0.0], 0.2) is True
    assert node._tracking_error_exceeded([0.0, 0.1], 0.2) is False
