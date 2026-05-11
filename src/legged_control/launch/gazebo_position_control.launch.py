"""Run position control against the Gazebo physics stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


try:
    _DOG_URDF_SHARE = get_package_share_directory("dog_urdf")
except Exception:
    _DOG_URDF_SHARE = ""


def _default_urdf_path():
    if _DOG_URDF_SHARE:
        return os.path.join(_DOG_URDF_SHARE, "urdf", "dog_urdf.urdf")
    return ""

def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    config_path = os.path.join(share, "config", "robot.yaml")
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
                "start_paused": "true",
            }.items(),
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="teleop_node",
            name="teleop_node",
            parameters=[{"config_path": config_path}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="gazebo_control_bridge",
            name="gazebo_control_bridge",
            parameters=[{"config_path": config_path}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="gait_node",
            name="gait_node",
            parameters=[
                {
                    "config_path": config_path,
                    "hold_duration": 0.0,
                    "ramp_duration": 4.0,
                }
            ],
            output="screen",
        ),
        TimerAction(period=0.5, actions=[
            ExecuteProcess(
                cmd=["zsh", "-lc",
                     "source /opt/ros/humble/setup.zsh && "
                     "ros2 topic pub --once /posture_command std_msgs/msg/Bool '{data: true}'"],
                output="screen",
            )
        ]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=_default_urdf_path(),
                description="Absolute path to Gazebo-friendly simulation URDF",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.50",
                description="Spawn height — robot free-falls ~0.6s before landing",
            ),
            DeclareLaunchArgument(
                "spawn_roll",
                default_value="0.0",
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
