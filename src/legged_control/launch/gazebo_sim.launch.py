"""gazebo_sim.launch.py — full simulation stack with policy.

Launches:
  gazebo_physics.launch.py    — Gazebo + robot model + ros2_control
  gazebo_control_bridge        — Gazebo joints ↔ /joint_states_aggregated
  imu_filter_madgwick          — /odin1/imu → /odin1/imu/filtered
  processing nodes             — state_estimator, height_scan, teleop
  policy_node                  — reads obs topics → /joint_commands
  test nodes                   — obs_monitor, vel_viz
  rviz2                        — visualization

Launch args:
  urdf_path         [auto]          Path to simulation URDF
  spawn_z           [0.50]          Robot spawn height
  model_path        []              Path to TorchScript .pt policy file
  rviz              [true]          Launch RViz2
  gui               [true]          Launch Gazebo GUI
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
    robot_cfg = os.path.join(share, "config", "robot_sim.yaml")
    model_path = LaunchConfiguration("model_path").perform(context)

    physics_launch = os.path.join(share, "launch", "gazebo_physics.launch.py")
    rviz_cfg = os.path.join(share, "config", "test.rviz")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physics_launch),
            launch_arguments={
                "urdf_path": LaunchConfiguration("urdf_path"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "gui": LaunchConfiguration("gui"),
            }.items(),
        ),
        Node(
            package="legged_control",
            executable="gazebo_control_bridge",
            name="gazebo_control_bridge",
            output="screen",
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            parameters=[{
                "use_mag": False,
                "publish_tf": False,
                "fixed_frame": "base_link",
                "world_frame": "enu",
            }],
            remappings=[
                ("imu/data_raw", "odin1/imu"),
                ("imu/data", "odin1/imu/filtered"),
            ],
            output="log",
        ),
        Node(
            package="legged_control",
            executable="joint_aggregator",
            name="joint_aggregator",
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="urdf_joint_state_bridge",
            name="urdf_joint_state_bridge",
            output="log",
        ),
        Node(
            package="legged_control",
            executable="state_estimator_node",
            name="state_estimator_node",
            parameters=[{"config_path": robot_cfg}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="height_scan_node",
            name="height_scan_node",
            output="log",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="log",
        ),
        Node(
            package="legged_control",
            executable="teleop_node",
            name="teleop_node",
            output="log",
        ),
        Node(
            package="legged_control",
            executable="policy_node",
            name="policy_node",
            parameters=[{
                "model_path": model_path,
                "config_path": robot_cfg,
            }],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": open(_default_urdf_path()).read(),
            }],
            output="log",
        ),
        Node(
            package="legged_control",
            executable="obs_monitor_node",
            name="obs_monitor_node",
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="vel_viz_node",
            name="vel_viz_node",
            output="log",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=_default_urdf_path(),
        ),
        DeclareLaunchArgument("spawn_z", default_value="0.50"),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="Path to TorchScript .pt policy file",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
