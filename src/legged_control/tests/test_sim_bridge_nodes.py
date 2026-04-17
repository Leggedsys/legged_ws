import pytest

from legged_control.fake_motor_bus_node import (
    _ns_from_joint_name,
    _targets_for_joint_names,
)
from legged_control.urdf_joint_state_bridge import (
    _joint_name_to_urdf_joint,
    _motor_to_urdf_joint_state,
)


def test_fake_motor_bus_namespace_mapping():
    assert _ns_from_joint_name("FR_hip") == "fr/hip"


def test_fake_motor_bus_filters_targets():
    result = _targets_for_joint_names(
        ["FR_hip", "FL_hip"], ["FR_hip", "RR_hip"], [0.1, 0.2]
    )
    assert result == {"FR_hip": 0.1, "FL_hip": 0.0}


def test_joint_name_to_urdf_joint():
    assert _joint_name_to_urdf_joint("FL_thigh") == "FL_thigh_joint"


def test_motor_to_urdf_joint_state_conversion():
    cfg = {
        "FR_hip": {"direction": 1, "zero_offset": 0.0},
        "FR_calf": {"direction": -1, "zero_offset": 2.791},
    }
    names, pos, vel = _motor_to_urdf_joint_state(
        ["FR_hip", "FR_calf"],
        [-0.14, 3.511],
        [0.5, -1.0],
        cfg,
    )
    assert names == ["FR_hip_joint", "FR_calf_joint"]
    assert pos == pytest.approx([-0.14, -0.72])
    assert vel == pytest.approx([0.5, 1.0])
