"""gazebo_sim.launch.py — full simulation stack with policy.

Launch args:
  urdf_path    [auto]      Path to simulation URDF
  spawn_z      [0.50]      Robot spawn height
  model_path   []          Path to TorchScript .pt policy file
  rviz         [true]      Launch RViz2
  gui          [true]      Launch Gazebo GUI
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
    make_imu_filter,
    make_joy,
    make_obs_assembler,
    make_robot_state_publisher,
    make_state_estimator,
    make_height_scan,
    make_teleop,
    make_obs_monitor,
    make_vel_viz,
    make_rviz2,
)

try:
    _DOG_URDF_SHARE = get_package_share_directory("dog_urdf")
except Exception:
    _DOG_URDF_SHARE = ""


def _default_urdf_path():
    if _DOG_URDF_SHARE:
        return os.path.join(_DOG_URDF_SHARE, "urdf", "dog_urdf.urdf")
    return ""


def _read_urdf(path: str) -> str:
    with open(path) as f:
        return f.read()


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    model_path = LaunchConfiguration("model_path").perform(context)

    physics_launch = os.path.join(share, "launch", "gazebo_physics.launch.py")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physics_launch),
            launch_arguments={
                "urdf_path": LaunchConfiguration("urdf_path"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "gui": LaunchConfiguration("gui"),
            }.items(),
        ),
        # ── sim data source ──────────────────────────────────────────────
        Node(
            package="legged_control",
            executable="gazebo_control_bridge",
            name="gazebo_control_bridge",
            output="screen",
        ),
        make_imu_filter(),
        # ── processing (shared with real) ────────────────────────────────
        make_state_estimator(),
        make_height_scan(),
        make_teleop(),
        make_obs_assembler(),
        # ── teleop ───────────────────────────────────────────────────────
        make_joy(),
        # ── policy ───────────────────────────────────────────────────────
        Node(
            package="legged_control",
            executable="policy_node",
            name="policy_node",
            parameters=[{"model_path": model_path}],
            output="screen",
        ),
        # ── visualization ────────────────────────────────────────────────
        make_robot_state_publisher(),
        make_obs_monitor(),
        make_vel_viz(),
        make_rviz2(),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=_default_urdf_path()),
        DeclareLaunchArgument("spawn_z", default_value="0.50"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="Path to TorchScript .pt policy file"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
