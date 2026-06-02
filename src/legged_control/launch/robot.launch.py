"""robot.launch.py — real robot with modes.

Includes real.launch.py for all hardware, adds processing + policy.
kp/kd come from robot.yaml (policy mode) or overridden to 0 (passive mode).

Launch args:
  mode          [passive]           passive | policy
  legs          [all]
  serial_port_front   [from robot.yaml]
  serial_port_rear    [from robot.yaml]
  model_path    []                  Path to TorchScript .pt policy file
#   dry_run       [false]             true = log motor cmds to file, motors passive
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from legged_control.launch_common import (
    make_state_estimator, make_height_scan, make_teleop,
    make_obs_assembler, make_robot_state_publisher,
    make_vel_viz, make_rviz2,
)

_YAML_SENTINEL = "__from_yaml__"


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()
    share = get_package_share_directory("legged_control")
    real_launch = os.path.join(share, "launch", "real.launch.py")

    if mode == "passive":
        kp_override = "0.0"
        kd_override = "0.0"
    elif mode == "policy":
        kp_override = "-1"  # use config value
        kd_override = "-1"
    else:
        raise RuntimeError(f"Unknown mode '{mode}'. Valid: passive, policy")

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "serial_port_front": LaunchConfiguration("serial_port_front"),
                "serial_port_rear":  LaunchConfiguration("serial_port_rear"),
                "legs":              LaunchConfiguration("legs"),
                "kp_override":       kp_override,
                "kd_override":       kd_override,
                "dry_run":           LaunchConfiguration("dry_run"),
            }.items(),
        ),
    ]

    # ── processing + obs ─────────────────────────────────────────────────
    nodes += [
        make_state_estimator(), make_height_scan(), make_teleop(),
        make_obs_assembler(),
    ]

    # ── TF + visualization ───────────────────────────────────────────────
    rsp = make_robot_state_publisher()
    if rsp is not None:
        nodes.append(rsp)
    nodes += [make_vel_viz()]

    if mode == "passive":
        nodes.append(make_rviz2())
    elif mode == "policy":
        nodes += [
            Node(package="legged_control", executable="policy_node",
                 name="policy_node",
                 parameters=[{"model_path": LaunchConfiguration("model_path")}],
                 output="screen"),
            make_rviz2(),
        ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="passive"),
        DeclareLaunchArgument("legs", default_value="all"),
        DeclareLaunchArgument("serial_port_front", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("serial_port_rear", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("dry_run", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
