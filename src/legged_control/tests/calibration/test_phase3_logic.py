from legged_control.calibration.phases.phase3 import (
    suggest_kd_from_overshoot,
    KP_MAX,
)

def test_suggest_kd_increase_on_overshoot():
    # frac = 0.25/0.1 = 2.5 > 1.0 → increase kd
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.25, step=0.1)
    assert result > current_kd

def test_suggest_kd_decrease_on_underdamp():
    # frac = 0.01/0.1 = 0.1 < 0.20 → decrease kd (overdamped)
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.01, step=0.1)
    assert result < current_kd

def test_suggest_kd_unchanged_in_good_range():
    # frac = 0.08/0.1 = 0.8, in range [0.20, 1.0] → unchanged
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.08, step=0.1)
    assert result == current_kd

def test_suggest_kd_unchanged_at_upper_boundary():
    # frac = 1.0 exactly → unchanged (boundary is exclusive: frac > 1.0 triggers increase)
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.1, step=0.1)
    assert result == current_kd

def test_suggest_kd_zero_step_guard():
    # step=0 → frac=0.0 → triggers decrease branch (documented safe-side behavior)
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.5, step=0.0)
    assert result < current_kd

def test_kp_max_constant():
    assert KP_MAX == 40
