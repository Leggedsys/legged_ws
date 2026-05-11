"""Run RL policy against the Gazebo physics simulation.

The URDF must include:
  - IMU sensor plugin publishing sensor_msgs/Imu on 'odin1/imu'
  - Depth camera plugin publishing sensor_msgs/Image on /camera/depth/image_rect_raw
    and sensor_msgs/CameraInfo on /camera/depth/camera_info

Same software stack as real-hardware policy mode:
  state_estimator_node + height_scan_node + policy_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DOG_URDF_SHARE = ""
try:
    _DOG_URDF_SHARE = get_package_share_directory("dog_urdf")
except Exception:
    pass

_DEFAULT_SIM_URDF = os.path.join(_DOG_URDF_SHARE, "urdf", "dog_urdf.urdf")
_POSTURE_CMD_DELAY = 2.0


def _ros2_pub_once(topic, msg_type, data):
    return ["zsh", "-lc",
            f"source /opt/ros/humble/setup.zsh && "
            f"ros2 topic pub --once {topic} {msg_type} '{data}'"]


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    config_path = os.path.join(share, "config", "robot.yaml")
    physics_launch = os.path.join(share, "launch", "gazebo_physics.launch.py")
    model_path = LaunchConfiguration("model_path").perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physics_launch),
            launch_arguments={
                "urdf_path": LaunchConfiguration("urdf_path"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "start_paused": "false",
            }.items(),
        ),
        Node(package="legged_control", executable="gazebo_control_bridge",
             name="gazebo_control_bridge",
             parameters=[{"config_path": config_path}], output="screen"),
        Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
             name="imu_filter_madgwick",
             parameters=[{"use_mag": False, "publish_tf": False,
                          "fixed_frame": "base_link", "world_frame": "enu"}],
             remappings=[("imu/data_raw", "odin1/imu"),
                         ("imu/data", "odin1/imu/filtered")],
             output="log"),
        Node(package="legged_control", executable="state_estimator_node",
             name="state_estimator_node",
             parameters=[{"config_path": config_path}], output="screen"),
        Node(package="legged_control", executable="height_scan_node",
             name="height_scan_node", output="screen"),
        Node(package="joy", executable="joy_node", name="joy_node", output="log"),
        Node(package="legged_control", executable="teleop_node",
             name="teleop_node", output="screen"),
        Node(package="legged_control", executable="policy_node",
             name="policy_node",
             parameters=[{"config_path": config_path, "model_path": model_path}],
             output="screen"),
        TimerAction(period=_POSTURE_CMD_DELAY, actions=[
            ExecuteProcess(
                cmd=_ros2_pub_once("/posture_command", "std_msgs/msg/Bool", "{data: true}"),
                output="screen",
            )
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=_DEFAULT_SIM_URDF),
        DeclareLaunchArgument("spawn_z", default_value="1.80"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="Path to TorchScript .pt policy file"),
        OpaqueFunction(function=_launch_setup),
    ])
