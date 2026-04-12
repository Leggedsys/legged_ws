# src/legged_control/tests/test_robot_launch.py
import importlib.util, os

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'robot.launch.py')
_spec   = importlib.util.spec_from_file_location('robot_launch', os.path.abspath(_LAUNCH))
_mod    = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
_leg_group = _mod._leg_group


def test_leg_group_front():
    assert _leg_group('FR_hip')   == 'front'
    assert _leg_group('FL_thigh') == 'front'
    assert _leg_group('FL_calf')  == 'front'


def test_leg_group_rear():
    assert _leg_group('RR_hip')   == 'rear'
    assert _leg_group('RL_thigh') == 'rear'
    assert _leg_group('RL_calf')  == 'rear'
