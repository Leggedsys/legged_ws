from legged_control.calibration.oscillation import OscillationDetector

def test_detects_rapid_reversals():
    d = OscillationDetector(window_sec=1.0, threshold=3)
    for t, v in [(0.0, 1.0), (0.1, -1.0), (0.2, 1.0), (0.3, -1.0), (0.4, 1.0)]:
        d.update(t, v)
    assert d.is_oscillating()

def test_stable_motion_not_oscillating():
    d = OscillationDetector(window_sec=1.0, threshold=3)
    for t, v in [(0.0, 1.0), (0.2, 0.8), (0.4, 0.3), (0.6, 0.0)]:
        d.update(t, v)
    assert not d.is_oscillating()

def test_old_samples_expire():
    d = OscillationDetector(window_sec=0.5, threshold=3)
    for t, v in [(0.0, 1.0), (0.1, -1.0), (0.2, 1.0), (0.3, -1.0)]:
        d.update(t, v)
    d.update(1.0, 0.1)
    assert not d.is_oscillating()

def test_reset_clears_history():
    d = OscillationDetector(window_sec=1.0, threshold=3)
    for t, v in [(0.0, 1.0), (0.1, -1.0), (0.2, 1.0), (0.3, -1.0), (0.4, 1.0)]:
        d.update(t, v)
    assert d.is_oscillating()
    d.reset()
    assert not d.is_oscillating()
