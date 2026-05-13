"""robot.launch.py — real robot with modes.

Launch args:
  mode          [passive]           passive | policy
  legs          [all]
  serial_port_front   [from robot.yaml]
  serial_port_rear    [from robot.yaml]
  model_path    []                  Path to TorchScript .pt policy file
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from legged_control.launch_common import (
    make_imu_filter,
    make_joy,
    make_joint_aggregator,
    make_motor_command_bridge,
    make_obs_assembler,
    make_odin1_node,
    make_odin_tf,
    make_realsense,
    make_robot_state_publisher,
    make_state_estimator,
    make_height_scan,
    make_teleop,
    make_urdf_joint_state_bridge,
    make_obs_monitor,
    make_vel_viz,
    make_rviz2,
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


def _bus_nodes(joints: list, port_map: dict, motor_hz: float, kp: float, kd: float) -> list:
    groups = {"front": ("motor_bus_front", port_map["front"]),
              "rear":  ("motor_bus_rear",  port_map["rear"])}
    nodes = []
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package="legged_control", executable="motor_bus_node", name=node_name,
            parameters=[{"serial_port": port, "joint_names": [j["name"] for j in group_joints],
                         "kp": kp, "kd": kd, "loop_hz": motor_hz}],
            remappings=[("/joint_commands", "/joint_commands_motor")],
            output="log",
        ))
    return nodes


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()
    legs_arg = LaunchConfiguration("legs").perform(context)
    sp_front = LaunchConfiguration("serial_port_front").perform(context)
    sp_rear = LaunchConfiguration("serial_port_rear").perform(context)

    cfg = _load_config()
    control = cfg["control"]
    if sp_front == _YAML_SENTINEL:
        sp_front = control["serial_port_front"]
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control["serial_port_rear"]
    port_map = {"front": sp_front, "rear": sp_rear}
    motor_hz = float(control["motor_hz"])

    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg["joints"] if j["name"].split("_")[0] in active_legs]

    # ── real layer (always) ──────────────────────────────────────────────
    if mode == "passive":
        motors = _bus_nodes(joints, port_map, motor_hz, kp=0.0, kd=0.0)
    elif mode == "policy":
        motors = _bus_nodes(joints, port_map, motor_hz,
                            kp=float(control["kp"]), kd=float(control["kd"]))
    else:
        raise RuntimeError(f"Unknown mode '{mode}'. Valid: passive, policy")

    nodes = (
        motors
        + [make_joint_aggregator(), make_motor_command_bridge(),
           make_urdf_joint_state_bridge(),
           make_odin1_node(), make_imu_filter(), make_realsense(), make_joy(),
           make_odin_tf()]
    )

    # ── processing (always) ──────────────────────────────────────────────
    nodes += [make_state_estimator(), make_height_scan(), make_teleop(),
              make_obs_assembler()]

    # ── mode-specific extras ─────────────────────────────────────────────
    rsp = make_robot_state_publisher()
    if rsp is not None:
        nodes.append(rsp)

    if mode == "passive":
        share = get_package_share_directory("legged_control")
        rviz_cfg = os.path.join(share, "config", "passive_mode.rviz")
        nodes += [
            Node(package="rviz2", executable="rviz2", name="rviz2",
                 arguments=["-d", rviz_cfg], output="log"),
            make_obs_monitor(),
        ]

    if mode == "policy":
        nodes.append(Node(
            package="legged_control", executable="policy_node",
            name="policy_node",
            parameters=[{"model_path": LaunchConfiguration("model_path")}],
            output="screen",
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="passive",
                              description="passive | policy"),
        DeclareLaunchArgument("legs", default_value="all"),
        DeclareLaunchArgument("serial_port_front", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("serial_port_rear", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("model_path", default_value="",
                              description="Path to TorchScript .pt policy file"),
        OpaqueFunction(function=_launch_setup),
    ])
