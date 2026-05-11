import math

import numpy as np

import legged_control.passive_monitor_node as _mod


def test_yaml_joint_names_count():
    assert len(_mod._YAML_JOINT_NAMES) == 12


def test_yaml_joint_names_order():
    names = _mod._YAML_JOINT_NAMES
    assert names[0] == "FR_hip"
    assert names[2] == "FR_calf"
    assert names[3] == "FL_hip"
    assert names[9] == "RL_hip"
    assert names[11] == "RL_calf"


def test_yaml_joint_names_all_legs():
    names = _mod._YAML_JOINT_NAMES
    for leg in ("FR", "FL", "RR", "RL"):
        for slot in ("hip", "thigh", "calf"):
            assert f"{leg}_{slot}" in names
