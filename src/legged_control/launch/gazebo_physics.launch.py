"""Minimal Gazebo physics launch for the quadruped simulation URDF."""

import os
import tempfile

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


def _default_controller_yaml():
    if _DOG_URDF_SHARE:
        return os.path.join(_DOG_URDF_SHARE, "config", "gazebo_ros2_controllers.yaml")
    return ""


def _read_text(path: str) -> str:
    with open(path) as f:
        return f.read()


def _prepare_urdf(urdf_path: str, controller_yaml: str) -> str:
    """Write URDF with injected paths to a temp file, return the temp path."""
    content = _read_text(urdf_path)
    content = content.replace("__CONTROLLER_YAML__", controller_yaml)
    content = content.replace("package://dog_urdf/", f"file://{_DOG_URDF_SHARE}/")
    fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix="dog_urdf_sim_")
    with os.fdopen(fd, "w") as f:
        f.write(content)
    return tmp_path


def _launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    controller_yaml = LaunchConfiguration("controller_yaml").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_roll = LaunchConfiguration("spawn_roll").perform(context)
    spawn_pitch = LaunchConfiguration("spawn_pitch").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    start_paused = LaunchConfiguration("start_paused").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    unpause_delay = float(LaunchConfiguration("unpause_delay").perform(context))
    gazebo_ros_share = "/opt/ros/humble/share/gazebo_ros"

    sim_urdf_path = _prepare_urdf(urdf_path, controller_yaml)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
            ),
            launch_arguments={"pause": start_paused, "gui": gui, "force_system": "false"}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": _read_text(sim_urdf_path)},
            ],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_quadruped",
            arguments=[
                "-file",
                sim_urdf_path,
                "-entity",
                "dog_urdf",
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
                default_value=_default_urdf_path(),
                description="Absolute path to the Gazebo-friendly simulation URDF",
            ),
            DeclareLaunchArgument(
                "controller_yaml",
                default_value=_default_controller_yaml(),
                description="ros2_control controller configuration for Gazebo",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.40",
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
                "gui",
                default_value="true",
                description="Launch Gazebo GUI (set to false for headless)",
            ),
            DeclareLaunchArgument(
                "unpause_delay",
                default_value="3.0",
                description="Seconds to wait before unpausing physics",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
