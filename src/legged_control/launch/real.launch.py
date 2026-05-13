"""real.launch.py — hardware interface layer.

Starts ALL hardware data-source nodes:
  - motor_bus_node ×2 (front/rear)  : RS485 motor I/O
  - joint_aggregator                 : merge + motor→URDF
  - motor_command_bridge             : URDF cmd → motor frame
  - urdf_joint_state_bridge          : → /joint_states (TF tree)
  - host_sdk_sample + imu_filter    : Odin1 IMU
  - realsense2_camera_node           : depth camera
  - joy_node                         : gamepad
  - odin static TF                   : connect odin→URDF TF trees

Published topics:
  /joint_states_aggregated     (URDF frame)
  /joint_states                (TF tree)
  /joint_commands_motor        (motor frame, for motor_bus)
  odin1/imu                    (raw IMU)
  odin1/imu/filtered           (filtered IMU)
  /camera/depth/image_rect_raw
  /camera/depth/camera_info
  /joy

Launch args:
  serial_port_front  [/dev/ttyUSB0]
  serial_port_rear   [/dev/ttyUSB1]
  legs               [all]
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from legged_control.launch_common import (
    make_imu_filter,
    make_joy,
    make_joint_aggregator,
    make_motor_command_bridge,
    make_odin1_node,
    make_odin_tf,
    make_realsense,
    make_urdf_joint_state_bridge,
)

_VALID_LEGS = {"FR", "FL", "RR", "RL"}
_YAML_SENTINEL = "__from_yaml__"


def _load_config() -> dict:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _leg_group(joint_name: str) -> str:
    return "front" if joint_name.split("_")[0] in ("FR", "FL") else "rear"


def _parse_legs(legs_arg: str) -> set:
    if legs_arg.upper() == "ALL":
        return _VALID_LEGS
    selected = {leg.strip().upper() for leg in legs_arg.split(",")}
    invalid = selected - _VALID_LEGS
    if invalid:
        raise RuntimeError(
            f"Unknown leg(s): {', '.join(sorted(invalid))}. "
            f"Valid: FR, FL, RR, RL, all"
        )
    return selected


def _launch_setup(context, *args, **kwargs):
    cfg = _load_config()
    control = cfg["control"]

    sp_front = LaunchConfiguration("serial_port_front").perform(context)
    sp_rear  = LaunchConfiguration("serial_port_rear").perform(context)
    legs_arg = LaunchConfiguration("legs").perform(context)

    if sp_front == _YAML_SENTINEL:
        sp_front = control["serial_port_front"]
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control["serial_port_rear"]

    motor_hz = float(control["motor_hz"])
    kp = float(control["kp"])
    kd = float(control["kd"])

    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg["joints"] if j["name"].split("_")[0] in active_legs]

    port_map = {"front": sp_front, "rear": sp_rear}
    groups = {
        "front": ("motor_bus_front", port_map["front"]),
        "rear":  ("motor_bus_rear",  port_map["rear"]),
    }

    nodes = []

    # motor_bus_node ×2
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package="legged_control",
            executable="motor_bus_node",
            name=node_name,
            parameters=[{
                "serial_port":  port,
                "joint_names":  [j["name"] for j in group_joints],
                "kp":           kp,
                "kd":           kd,
                "loop_hz":      motor_hz,
            }],
            remappings=[("/joint_commands", "/joint_commands_motor")],
            output="log",
        ))

    # real-layer convertors
    nodes.append(make_joint_aggregator())
    nodes.append(make_motor_command_bridge())
    nodes.append(make_urdf_joint_state_bridge())

    # external sensors
    nodes.append(make_odin1_node())
    nodes.append(make_imu_filter())
    nodes.append(make_realsense())
    nodes.append(make_joy())
    nodes.append(make_odin_tf())

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port_front", default_value=_YAML_SENTINEL,
                              description="Serial port for FR/FL motors"),
        DeclareLaunchArgument("serial_port_rear", default_value=_YAML_SENTINEL,
                              description="Serial port for RR/RL motors"),
        DeclareLaunchArgument("legs", default_value="all",
                              description="Legs: all | FR | FL | RR | RL | comma-separated"),
        OpaqueFunction(function=_launch_setup),
    ])
