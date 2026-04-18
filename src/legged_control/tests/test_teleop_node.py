import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legged_control.teleop_node import (
    _apply_deadzone,
    _button_is_rising_edge,
    _normalize_trigger_axis,
    _scale_axis,
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


class TestButtonIsRisingEdge:
    def test_detects_button_press_edge(self):
        assert _button_is_rising_edge(0, 1) is True

    def test_ignores_button_hold(self):
        assert _button_is_rising_edge(1, 1) is False

    def test_ignores_button_release(self):
        assert _button_is_rising_edge(1, 0) is False


# ---------------------------------------------------------------------------
# Trigger (LT/RT) height-rate tests
# ---------------------------------------------------------------------------


def _dz(
    lt: float,
    rt: float,
    deadzone: float = 0.05,
    max_dz: float = 0.03,
    lt_released: float = 0.0,
    rt_released: float = 0.0,
) -> float:
    """Compute height rate from trigger values (mirrors teleop_node implementation)."""
    return _scale_axis(
        _normalize_trigger_axis(rt, rt_released), deadzone, max_dz, invert=False
    ) - _scale_axis(
        _normalize_trigger_axis(lt, lt_released), deadzone, max_dz, invert=False
    )


class TestNormalizeTriggerAxis:
    def test_keeps_zero_to_one_convention(self):
        assert _normalize_trigger_axis(0.0, 0.0) == pytest.approx(0.0)
        assert _normalize_trigger_axis(0.4, 0.0) == pytest.approx(0.4)
        assert _normalize_trigger_axis(1.0, 0.0) == pytest.approx(1.0)

    def test_maps_negative_one_to_one_convention(self):
        assert _normalize_trigger_axis(-1.0, -1.0) == pytest.approx(0.0)
        assert _normalize_trigger_axis(0.0, -1.0) == pytest.approx(0.5)
        assert _normalize_trigger_axis(1.0, -1.0) == pytest.approx(1.0)

    def test_maps_positive_one_to_negative_one_convention(self):
        assert _normalize_trigger_axis(1.0, 1.0) == pytest.approx(0.0)
        assert _normalize_trigger_axis(0.0, 1.0) == pytest.approx(0.5)
        assert _normalize_trigger_axis(-1.0, 1.0) == pytest.approx(1.0)

    def test_clamps_out_of_range_values(self):
        assert _normalize_trigger_axis(-2.0, -1.0) == pytest.approx(0.0)
        assert _normalize_trigger_axis(2.0, 0.0) == pytest.approx(1.0)


def test_trigger_both_released_gives_zero():
    assert _dz(lt=0.0, rt=0.0) == pytest.approx(0.0)


def test_trigger_rt_fully_pressed_gives_max_dz():
    assert _dz(lt=0.0, rt=1.0) == pytest.approx(0.03)


def test_trigger_lt_fully_pressed_gives_neg_max_dz():
    assert _dz(lt=1.0, rt=0.0) == pytest.approx(-0.03)


def test_trigger_within_deadzone_gives_zero():
    assert _dz(lt=0.0, rt=0.03) == pytest.approx(0.0)  # 0.03 < deadzone 0.05


def test_trigger_partial_differential():
    """Both triggers partially pressed → dz is scaled difference."""
    deadzone, max_dz = 0.05, 0.03
    # _apply_deadzone(0.7) = (0.7 - 0.05) / 0.95 = 0.6842
    # _apply_deadzone(0.3) = (0.3 - 0.05) / 0.95 = 0.2632
    expected = (0.6842 - 0.2632) * max_dz
    assert _dz(lt=0.3, rt=0.7) == pytest.approx(expected, abs=1e-4)


def test_trigger_both_fully_pressed_cancels():
    assert _dz(lt=1.0, rt=1.0) == pytest.approx(0.0)


def test_negative_one_to_one_trigger_convention_released_is_zero():
    assert _dz(lt=-1.0, rt=-1.0, lt_released=-1.0, rt_released=-1.0) == pytest.approx(0.0)


def test_negative_one_to_one_trigger_convention_pressed_scales_correctly():
    assert _dz(lt=-1.0, rt=1.0, lt_released=-1.0, rt_released=-1.0) == pytest.approx(0.03)
    assert _dz(lt=1.0, rt=-1.0, lt_released=-1.0, rt_released=-1.0) == pytest.approx(-0.03)


def test_positive_one_to_negative_one_trigger_convention_released_is_zero():
    assert _dz(lt=1.0, rt=1.0, lt_released=1.0, rt_released=1.0) == pytest.approx(0.0)
