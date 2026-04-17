"""Run a stand-only controller against the Gazebo physics stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_SIM_URDF = (
    "/home/grayerd/Desktop/Projects/rc/塞北箭4urdf/urdf/塞北箭4_sim.urdf"
)


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    sim_config = os.path.join(share, "config", "robot_sim.yaml")
    physics_launch = os.path.join(share, "launch", "gazebo_physics.launch.py")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physics_launch),
            launch_arguments={
                "urdf_path": LaunchConfiguration("urdf_path"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "spawn_roll": LaunchConfiguration("spawn_roll"),
                "spawn_pitch": LaunchConfiguration("spawn_pitch"),
                "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                "start_paused": LaunchConfiguration("start_paused"),
                "unpause_delay": LaunchConfiguration("unpause_delay"),
            }.items(),
        ),
        Node(
            package="legged_control",
            executable="gazebo_control_bridge",
            name="gazebo_control_bridge",
            parameters=[{"config_path": sim_config}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="stand_node",
            name="stand_node",
            parameters=[{"config_path": sim_config}],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=_DEFAULT_SIM_URDF,
                description="Absolute path to Gazebo-friendly simulation URDF",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.38",
                description="Initial robot spawn height above ground",
            ),
            DeclareLaunchArgument(
                "spawn_roll",
                default_value="3.14159",
                description="Initial robot roll in radians",
            ),
            DeclareLaunchArgument(
                "spawn_pitch",
                default_value="0.0",
                description="Initial robot pitch in radians",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
                description="Initial robot yaw in radians",
            ),
            DeclareLaunchArgument(
                "start_paused",
                default_value="true",
                description="Start Gazebo paused before releasing physics",
            ),
            DeclareLaunchArgument(
                "unpause_delay",
                default_value="3.0",
                description="Seconds to wait before unpausing physics",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
