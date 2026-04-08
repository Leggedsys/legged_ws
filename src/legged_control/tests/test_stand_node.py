from legged_control.stand_node import _load_joint_defaults, _motor_node_name


def test_load_joint_defaults_names():
    joints_cfg = [
        {'name': 'FR_hip',   'default_q':  0.0},
        {'name': 'FR_thigh', 'default_q':  0.8},
        {'name': 'FR_calf',  'default_q': -1.5},
    ]
    names, _ = _load_joint_defaults(joints_cfg)
    assert names == ['FR_hip', 'FR_thigh', 'FR_calf']


def test_load_joint_defaults_values():
    joints_cfg = [
        {'name': 'FR_hip',   'default_q':  0.0},
        {'name': 'FR_thigh', 'default_q':  0.8},
        {'name': 'FR_calf',  'default_q': -1.5},
    ]
    _, defaults = _load_joint_defaults(joints_cfg)
    assert defaults == [0.0, 0.8, -1.5]


def test_load_joint_defaults_coerces_to_float():
    joints_cfg = [{'name': 'FR_hip', 'default_q': '0'}]
    _, defaults = _load_joint_defaults(joints_cfg)
    assert isinstance(defaults[0], float)


def test_load_joint_defaults_12_joints():
    joints_cfg = [
        {'name': f'J{i}', 'default_q': float(i)} for i in range(12)
    ]
    names, defaults = _load_joint_defaults(joints_cfg)
    assert len(names) == 12
    assert len(defaults) == 12


def test_motor_node_name_hip():
    assert _motor_node_name('FR_hip') == '/fr/hip/motor'


def test_motor_node_name_calf():
    assert _motor_node_name('RL_calf') == '/rl/calf/motor'


def test_motor_node_name_thigh():
    assert _motor_node_name('RR_thigh') == '/rr/thigh/motor'
