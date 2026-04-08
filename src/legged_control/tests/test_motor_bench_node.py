import math
import pytest
from legged_control.motor_bench_node import (
    _apply_deadzone,
    _map_to_range,
    _select_joints,
)

# --- _apply_deadzone ---

def test_deadzone_zero_input():
    assert _apply_deadzone(0.0) == 0.0

def test_deadzone_inside_zone():
    assert _apply_deadzone(0.04) == 0.0
    assert _apply_deadzone(-0.04) == 0.0

def test_deadzone_at_boundary():
    assert _apply_deadzone(0.05) == 0.0   # exactly at edge → still zero

def test_deadzone_full_positive():
    result = _apply_deadzone(1.0)
    assert math.isclose(result, 1.0, rel_tol=1e-6)

def test_deadzone_full_negative():
    result = _apply_deadzone(-1.0)
    assert math.isclose(result, -1.0, rel_tol=1e-6)

def test_deadzone_rescales_above_boundary():
    # Just above deadzone (0.05) should give a small positive value, not jump to 1
    result = _apply_deadzone(0.10)
    assert 0.0 < result < 0.1

def test_deadzone_custom_dz():
    assert _apply_deadzone(0.09, dz=0.10) == 0.0
    result = _apply_deadzone(0.20, dz=0.10)
    assert result > 0.0

# --- _map_to_range ---

def test_map_range_min():
    assert math.isclose(_map_to_range(-1.0, -1.047, 1.047), -1.047, rel_tol=1e-6)

def test_map_range_max():
    assert math.isclose(_map_to_range(1.0, -1.047, 1.047), 1.047, rel_tol=1e-6)

def test_map_range_midpoint():
    result = _map_to_range(0.0, -1.047, 1.047)
    assert math.isclose(result, 0.0, abs_tol=1e-6)

def test_map_range_asymmetric():
    # calf: q_min=-2.723, q_max=-0.837
    result = _map_to_range(-1.0, -2.723, -0.837)
    assert math.isclose(result, -2.723, rel_tol=1e-6)
    result = _map_to_range(1.0, -2.723, -0.837)
    assert math.isclose(result, -0.837, rel_tol=1e-6)

# --- _select_joints ---

JOINTS_CFG = [
    {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'FR_calf',  'motor_id': 2, 'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
    {'name': 'FL_hip',   'motor_id': 3, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'FL_thigh', 'motor_id': 4, 'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'FL_calf',  'motor_id': 5, 'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
    {'name': 'RR_hip',   'motor_id': 6, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'RR_thigh', 'motor_id': 7, 'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'RR_calf',  'motor_id': 8, 'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
    {'name': 'RL_hip',   'motor_id': 9, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'RL_thigh', 'motor_id': 10,'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'RL_calf',  'motor_id': 11,'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
]

def test_select_joints_fr_uppercase():
    result = _select_joints(JOINTS_CFG, 'FR')
    assert len(result) == 3
    assert [j['name'] for j in result] == ['FR_hip', 'FR_thigh', 'FR_calf']

def test_select_joints_fr_lowercase():
    result = _select_joints(JOINTS_CFG, 'fr')
    assert len(result) == 3
    assert result[0]['name'] == 'FR_hip'

def test_select_joints_rl():
    result = _select_joints(JOINTS_CFG, 'RL')
    assert [j['name'] for j in result] == ['RL_hip', 'RL_thigh', 'RL_calf']

def test_select_joints_unknown_leg():
    result = _select_joints(JOINTS_CFG, 'XX')
    assert result == []
