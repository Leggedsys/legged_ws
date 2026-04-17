import pytest

from legged_control.gazebo_control_bridge import (
    URDF_JOINT_ORDER,
    _motor_commands_to_urdf_positions,
    _urdf_joint_state_to_motor_aggregated,
)


def test_motor_commands_to_urdf_positions_uses_defaults_and_conversion():
    cfg = {
        "FL_hip": {"default_q": 0.0, "direction": 1, "zero_offset": 0.0},
        "FL_thigh": {"default_q": -0.5, "direction": 1, "zero_offset": 1.0},
        "FL_calf": {"default_q": -1.0, "direction": -1, "zero_offset": -2.0},
    }
    order = ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"]
    result = _motor_commands_to_urdf_positions(
        ["FL_hip", "FL_calf"], [0.2, -1.5], cfg, order
    )
    assert result == pytest.approx([0.2, 0.5, -0.5])


def test_urdf_joint_state_to_motor_aggregated_converts_back():
    cfg = {
        "FL_hip": {"direction": 1, "zero_offset": 0.0},
        "FR_calf": {"direction": -1, "zero_offset": 2.791},
    }
    names, pos, vel = _urdf_joint_state_to_motor_aggregated(
        ["FL_hip_joint", "FR_calf_joint"],
        [0.12, -0.72],
        [0.5, 1.0],
        cfg,
        ["FL_hip", "FR_calf"],
    )
    assert names == ["FL_hip", "FR_calf"]
    assert pos == pytest.approx([0.12, 3.511])
    assert vel == pytest.approx([0.5, -1.0])


def test_full_urdf_joint_order_contains_12_entries():
    assert len(URDF_JOINT_ORDER) == 12
