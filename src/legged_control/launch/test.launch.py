"""test.launch.py — data-link verification with full visualization."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from legged_control.launch_common import (
    make_joy, make_imu_filter, make_odin1_node, make_realsense,
    make_joint_aggregator, make_urdf_joint_state_bridge, make_motor_command_bridge,
    make_state_estimator, make_height_scan, make_teleop, make_obs_assembler,
    make_robot_state_publisher, make_obs_monitor, make_vel_viz, make_rviz2,
    make_odin_tf,
)

_YAML_SENTINEL = "__from_yaml__"
_VALID_LEGS = {"FR", "FL", "RR", "RL"}


def _load_config() -> dict:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _leg_group(joint_name: str) -> str:
    return "front" if joint_name.split("_")[0] in ("FR", "FL") else "rear"


def _parse_legs(legs_arg: str) -> set:
    if legs_arg.upper() == "ALL":
        return _VALID_LEGS
    selected = {l.strip().upper() for l in legs_arg.split(",")}
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
    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg["joints"] if j["name"].split("_")[0] in active_legs]

    nodes = []

    # ── real hardware ──────────────────────────────────────────────────
    groups = {"front": ("motor_bus_front", sp_front),
              "rear":  ("motor_bus_rear",  sp_rear)}
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package="legged_control",
            executable="motor_bus_node",
            name=node_name,
            parameters=[{"serial_port": port,
                         "joint_names": [j["name"] for j in group_joints],
                         "kp": 0.0, "kd": 0.0, "loop_hz": motor_hz}],
            remappings=[("/joint_commands", "/joint_commands_motor")],
            output="log",
        ))

    nodes.append(make_joint_aggregator())
    nodes.append(make_motor_command_bridge())
    nodes.append(make_urdf_joint_state_bridge())

    # ── external sensors ───────────────────────────────────────────────
    nodes.append(make_odin1_node())
    nodes.append(make_imu_filter())
    nodes.append(make_realsense())
    nodes.append(make_joy())

    # ── processing ─────────────────────────────────────────────────────
    nodes.append(make_state_estimator())
    nodes.append(make_height_scan())
    nodes.append(make_teleop())
    nodes.append(make_obs_assembler())

    # ── TF glue ────────────────────────────────────────────────────────
    nodes.append(make_odin_tf())
    rsp = make_robot_state_publisher()
    if rsp is not None:
        nodes.append(rsp)

    # ── test / visualization ───────────────────────────────────────────
    nodes.append(make_obs_monitor())
    nodes.append(make_vel_viz())

    share = get_package_share_directory("legged_control")
    rviz_cfg = os.path.join(share, "config", "test.rviz")
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="log",
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port_front", default_value=_YAML_SENTINEL,
                              description="Serial port for FR/FL motors"),
        DeclareLaunchArgument("serial_port_rear", default_value=_YAML_SENTINEL,
                              description="Serial port for RR/RL motors"),
        DeclareLaunchArgument("legs", default_value="all",
                              description="Legs: all | FR,FL,..."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz2"),
        OpaqueFunction(function=_launch_setup),
    ])
