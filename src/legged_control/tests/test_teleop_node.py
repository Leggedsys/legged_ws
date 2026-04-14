import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legged_control.teleop_node import (
    _apply_deadzone,
    _scale_axis,
    _toggle_latched_estop,
)


class TestApplyDeadzone:
    def test_inside_deadzone_returns_zero(self):
        # 0.04 < deadzone 0.05 → zero
        assert _apply_deadzone(0.04, 0.05) == 0.0

    def test_negative_inside_deadzone_returns_zero(self):
        assert _apply_deadzone(-0.03, 0.05) == 0.0

    def test_zero_returns_zero(self):
        assert _apply_deadzone(0.0, 0.05) == 0.0

    def test_full_positive_returns_one(self):
        # (1.0 - 0.05) / (1 - 0.05) = 1.0
        assert abs(_apply_deadzone(1.0, 0.05) - 1.0) < 1e-9

    def test_full_negative_returns_minus_one(self):
        assert abs(_apply_deadzone(-1.0, 0.05) - (-1.0)) < 1e-9

    def test_midrange_value(self):
        # raw=0.55, dz=0.05 → (0.55-0.05)/(1-0.05) = 0.50/0.95
        expected = 0.50 / 0.95
        assert abs(_apply_deadzone(0.55, 0.05) - expected) < 1e-9

    def test_continuous_at_deadzone_boundary(self):
        # Exactly at boundary: (0.05-0.05)/(1-0.05) = 0.0, not a jump
        assert abs(_apply_deadzone(0.05, 0.05) - 0.0) < 1e-9

    def test_deadzone_at_one_returns_zero(self):
        # deadzone=1.0 means nothing can move; guard prevents division by zero
        assert _apply_deadzone(1.0, 1.0) == 0.0


class TestScaleAxis:
    def test_inside_deadzone_returns_zero(self):
        assert _scale_axis(0.0, 0.05, 1.0, False) == 0.0

    def test_full_positive_no_invert(self):
        assert abs(_scale_axis(1.0, 0.05, 1.0, False) - 1.0) < 1e-9

    def test_full_positive_with_invert(self):
        assert abs(_scale_axis(1.0, 0.05, 1.0, True) - (-1.0)) < 1e-9

    def test_scales_by_max_vel(self):
        # max_vel=0.5 halves the output
        assert abs(_scale_axis(1.0, 0.05, 0.5, False) - 0.5) < 1e-9

    def test_negative_input_no_invert(self):
        assert abs(_scale_axis(-1.0, 0.05, 1.0, False) - (-1.0)) < 1e-9

    def test_negative_input_with_invert(self):
        assert abs(_scale_axis(-1.0, 0.05, 1.0, True) - 1.0) < 1e-9


class TestToggleLatchedEstop:
    def test_no_button_config_keeps_unlatched(self):
        pressed, latched = _toggle_latched_estop(False, False, [1, 0], -1)
        assert pressed is False
        assert latched is False

    def test_rising_edge_latches_estop(self):
        pressed, latched = _toggle_latched_estop(False, False, [1, 0], 0)
        assert pressed is True
        assert latched is True

    def test_holding_button_does_not_toggle_twice(self):
        pressed, latched = _toggle_latched_estop(True, True, [1, 0], 0)
        assert pressed is True
        assert latched is True

    def test_second_press_releases_latch(self):
        pressed, latched = _toggle_latched_estop(False, True, [1, 0], 0)
        assert pressed is True
        assert latched is False
