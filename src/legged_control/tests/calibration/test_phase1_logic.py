from legged_control.calibration.phases.phase1 import (
    detect_moved_joint,
    validate_mapping,
)

def test_detect_most_moved():
    baseline = {'FR_hip': 0.0, 'FR_thigh': 0.8, 'FR_calf': -1.5}
    current  = {'FR_hip': 0.1, 'FR_thigh': 0.8, 'FR_calf': -2.8}
    name, delta = detect_moved_joint(baseline, current, threshold=0.3)
    assert name == 'FR_calf'
    assert abs(delta - 1.3) < 1e-6

def test_detect_below_threshold_returns_none():
    baseline = {'FR_hip': 0.0}
    current  = {'FR_hip': 0.05}
    assert detect_moved_joint(baseline, current, threshold=0.3) is None

def test_validate_mapping_ok():
    joint_names = ['FR_hip', 'FR_thigh']
    mapping = {'FR_hip': 0, 'FR_thigh': 1}
    gaps, dupes = validate_mapping(joint_names, mapping)
    assert gaps == []
    assert dupes == []

def test_validate_mapping_gap():
    joint_names = ['FR_hip', 'FR_thigh']
    mapping = {'FR_hip': 0}
    gaps, dupes = validate_mapping(joint_names, mapping)
    assert 'FR_thigh' in gaps

def test_validate_mapping_duplicate():
    joint_names = ['FR_hip', 'FR_thigh']
    mapping = {'FR_hip': 0, 'FR_thigh': 0}
    _, dupes = validate_mapping(joint_names, mapping)
    assert len(dupes) > 0
