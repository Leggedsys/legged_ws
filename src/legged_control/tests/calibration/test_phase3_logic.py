from legged_control.calibration.phases.phase3 import (
    suggest_kd_from_overshoot,
    KP_MAX,
)

def test_suggest_kd_increase_on_overshoot():
    # overshoot > 20% of step size → increase kd
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.25, step=0.1)
    assert result > current_kd

def test_suggest_kd_decrease_on_underdamp():
    # overshoot < 5% of step size → decrease kd (overdamped)
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.01, step=0.1)
    assert result < current_kd

def test_suggest_kd_unchanged_in_good_range():
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.08, step=0.1)
    assert result == current_kd

def test_kp_max_constant():
    assert KP_MAX == 40
