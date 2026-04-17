"""Minimal Gazebo physics launch for the quadruped simulation URDF."""

import os

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


_DEFAULT_SIM_URDF = (
    "/home/grayerd/Desktop/Projects/rc/塞北箭4urdf/urdf/塞北箭4_sim.urdf"
)
_DEFAULT_CONTROLLER_YAML = (
    "/home/grayerd/Desktop/Projects/rc/塞北箭4urdf/config/gazebo_ros2_controllers.yaml"
)


def _read_text(path: str) -> str:
    with open(path) as f:
        return f.read()


def _launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    controller_yaml = LaunchConfiguration("controller_yaml").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_roll = LaunchConfiguration("spawn_roll").perform(context)
    spawn_pitch = LaunchConfiguration("spawn_pitch").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    start_paused = LaunchConfiguration("start_paused").perform(context)
    unpause_delay = float(LaunchConfiguration("unpause_delay").perform(context))
    gazebo_ros_share = "/opt/ros/humble/share/gazebo_ros"

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
            ),
            launch_arguments={"pause": start_paused}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": _read_text(urdf_path)},
                controller_yaml,
            ],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_quadruped",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                "saibeijian4_sim",
                "-R",
                spawn_roll,
                "-P",
                spawn_pitch,
                "-Y",
                spawn_yaw,
                "-z",
                spawn_z,
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawn_joint_state_broadcaster",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawn_gait_position_controller",
            arguments=[
                "gait_position_controller",
                "--controller-manager",
                "/controller_manager",
            ],
            output="screen",
        ),
    ]

    if start_paused.lower() == "true":
        actions.append(
            TimerAction(
                period=unpause_delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "zsh",
                            "-lc",
                            "source /opt/ros/humble/setup.zsh && ros2 service call /unpause_physics std_srvs/srv/Empty '{}'",
                        ],
                        output="screen",
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=_DEFAULT_SIM_URDF,
                description="Absolute path to the Gazebo-friendly simulation URDF",
            ),
            DeclareLaunchArgument(
                "controller_yaml",
                default_value=_DEFAULT_CONTROLLER_YAML,
                description="ros2_control controller configuration for Gazebo",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.50",
                description="Initial robot spawn height above ground",
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
