"""test.launch.py — data-link verification with full visualization.

Includes real.launch.py (kp=kd=0, passive motors) + processing + RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from legged_control.launch_common import (
    make_state_estimator, make_height_scan, make_teleop,
    make_obs_assembler, make_robot_state_publisher,
    make_obs_monitor, make_monitor_node, make_vel_viz, make_rviz2,
)

_YAML_SENTINEL = "__from_yaml__"


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    real_launch = os.path.join(share, "launch", "real.launch.py")

    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "serial_port_front": LaunchConfiguration("serial_port_front"),
                "serial_port_rear":  LaunchConfiguration("serial_port_rear"),
                "legs":              LaunchConfiguration("legs"),
                "kp_override":       "0.0",
                "kd_override":       "0.0",
            }.items(),
        ),
    ]

    # ── processing ──────────────────────────────────────────────────────
    nodes += [
        make_state_estimator(), make_height_scan(), make_teleop(),
        make_obs_assembler(),
    ]

    # ── TF + visualization ───────────────────────────────────────────────
    rsp = make_robot_state_publisher()
    if rsp is not None:
        nodes.append(rsp)
    nodes += [make_obs_monitor(), make_monitor_node(), make_vel_viz()]

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
        DeclareLaunchArgument("serial_port_front", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("serial_port_rear", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("legs", default_value="all"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
