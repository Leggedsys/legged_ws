from legged_control.standup_node import (
    _clamp_error_targets,
    _progress_fraction,
)


def test_clamp_error_targets_limits_per_joint_error():
    result = _clamp_error_targets([0.0, 1.0], [0.5, -1.0], 0.2)
    assert result == [0.2, 0.8]


def test_progress_fraction_reaches_one_at_target():
    assert _progress_fraction([0.5, -0.5], [0.5, -0.5]) == 1.0


def test_progress_fraction_is_zero_at_origin_for_nonzero_target():
    assert _progress_fraction([0.0, 0.0], [1.0, -1.0]) == 0.0
