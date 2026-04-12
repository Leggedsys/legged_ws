import math
from legged_control.passive_monitor_node import _ns_from_joint_name, _format_display


def test_ns_from_joint_name_hip():
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_from_joint_name_calf():
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_from_joint_name_thigh():
    assert _ns_from_joint_name('FL_thigh') == 'fl/thigh'


def test_format_display_structure():
    positions = {
        'FR_hip': 0.312, 'FR_thigh': 0.841, 'FR_calf': -1.502,
        'FL_hip': 0.000, 'FL_thigh': 0.823, 'FL_calf': -1.499,
        'RR_hip': -0.015, 'RR_thigh': 0.810, 'RR_calf': -1.510,
        'RL_hip': 0.003, 'RL_thigh': 0.834, 'RL_calf': -1.495,
    }
    text = _format_display(positions)
    lines = text.splitlines()
    assert len(lines) == 4
    assert '[passive]' in lines[0]
    assert 'FR' in lines[0]
    assert 'FL' in lines[1]
    assert 'RR' in lines[2]
    assert 'RL' in lines[3]


def test_format_display_values():
    positions = {
        'FR_hip': 0.312, 'FR_thigh': 0.841, 'FR_calf': -1.502,
        'FL_hip': 0.0, 'FL_thigh': 0.823, 'FL_calf': -1.499,
        'RR_hip': -0.015, 'RR_thigh': 0.810, 'RR_calf': -1.510,
        'RL_hip': 0.003, 'RL_thigh': 0.834, 'RL_calf': -1.495,
    }
    text = _format_display(positions)
    assert ' 0.312' in text
    assert '-1.502' in text
    assert '-0.015' in text


def test_format_display_nan_for_missing():
    text = _format_display({})
    assert 'nan' in text
