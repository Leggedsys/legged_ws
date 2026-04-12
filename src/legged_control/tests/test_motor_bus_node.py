# src/legged_control/tests/test_motor_bus_node.py
from legged_control.motor_bus_node import _ns_from_joint_name, _filter_joints


def test_ns_hip():
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_calf():
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_thigh():
    assert _ns_from_joint_name('RR_thigh') == 'rr/thigh'


def test_filter_joints_subset():
    joints = [
        {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0},
        {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8},
        {'name': 'RR_hip',   'motor_id': 6, 'default_q': 0.0},
    ]
    result = _filter_joints(joints, ['FR_hip', 'FR_thigh'])
    assert len(result) == 2
    assert result[0]['name'] == 'FR_hip'
    assert result[1]['name'] == 'FR_thigh'


def test_filter_joints_empty_names():
    joints = [{'name': 'FR_hip', 'motor_id': 0, 'default_q': 0.0}]
    assert _filter_joints(joints, []) == []


def test_filter_joints_unknown_name():
    joints = [{'name': 'FR_hip', 'motor_id': 0, 'default_q': 0.0}]
    assert _filter_joints(joints, ['XX_hip']) == []


def test_filter_joints_preserves_yaml_order():
    joints = [
        {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0},
        {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8},
        {'name': 'FR_calf',  'motor_id': 2, 'default_q': -1.5},
    ]
    result = _filter_joints(joints, ['FR_calf', 'FR_hip'])
    assert [j['name'] for j in result] == ['FR_hip', 'FR_calf']
