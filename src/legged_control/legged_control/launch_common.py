"""Shared node factories for launch files.

Every launch file that needs the same sensor/processing/test setup
imports these helpers so configuration is defined once.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

_LEGGED_SHARE = get_package_share_directory("legged_control")


def _robot_description() -> str:
    try:
        dog_share = get_package_share_directory("dog_urdf")
    except Exception:
        return ""
    urdf_path = os.path.join(dog_share, "urdf", "dog_urdf.urdf")
    with open(urdf_path) as f:
        content = f.read()
    content = content.replace("__CONTROLLER_YAML__", "")
    content = content.replace("package://dog_urdf/", f"file://{dog_share}/")
    return content


# ── data source nodes ──────────────────────────────────────────────────────────

def make_odin1_node():
    """Odin1 UVDAR driver (IMU + point cloud)."""
    return Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="odin1_node",
        output="log",
    )


def make_imu_filter():
    """Madgwick filter: /odin1/imu → /odin1/imu/filtered."""
    return Node(
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
    )


def make_realsense():
    """RealSense D435 depth camera with remaps to standard topics."""
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        remappings=[
            ("/camera/camera/depth/camera_info", "/camera/depth/camera_info"),
            ("/camera/camera/depth/image_rect_raw", "/camera/depth/image_rect_raw"),
        ],
        output="log",
    )


def make_joy():
    return Node(package="joy", executable="joy_node", name="joy_node", output="log")


def make_odin_tf():
    """Connect odin1_base_link (from odin driver's odom→odin1_base_link)
    to the URDF root (base_link)."""
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odin_base_tf",
        arguments=["-0.10", "0.0", "-0.08", "0.0", "0.0", "0.0",
                   "odin1_base_link", "base_link"],
        output="log",
    )


# ── real layer nodes ───────────────────────────────────────────────────────────

def make_joint_aggregator():
    return Node(
        package="legged_control",
        executable="joint_aggregator",
        name="joint_aggregator",
        output="screen",
    )


def make_motor_command_bridge():
    return Node(
        package="legged_control",
        executable="motor_command_bridge",
        name="motor_command_bridge",
        output="log",
    )


def make_urdf_joint_state_bridge():
    return Node(
        package="legged_control",
        executable="urdf_joint_state_bridge",
        name="urdf_joint_state_bridge",
        output="log",
    )


# ── processing layer nodes ─────────────────────────────────────────────────────

def make_state_estimator(config_path: str = ""):
    params = {}
    if config_path:
        params["config_path"] = config_path
    return Node(
        package="legged_control",
        executable="state_estimator_node",
        name="state_estimator_node",
        parameters=[params] if params else None,
        output="screen",
    )


def make_height_scan():
    return Node(
        package="legged_control",
        executable="height_scan_node",
        name="height_scan_node",
        output="log",
    )


def make_teleop():
    return Node(
        package="legged_control",
        executable="teleop_node",
        name="teleop_node",
        output="log",
    )


def make_obs_assembler():
    return Node(
        package="legged_control",
        executable="obs_assembler",
        name="obs_assembler",
        output="screen",
    )


# ── test / visualization ───────────────────────────────────────────────────────

_ROBOT_DESC = _robot_description()  # cached


def make_robot_state_publisher():
    if not _ROBOT_DESC:
        return None
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": _ROBOT_DESC}],
        output="log",
    )


def make_obs_monitor():
    return Node(
        package="legged_control",
        executable="obs_monitor_node",
        name="obs_monitor_node",
        output="screen",
    )


def make_vel_viz():
    return Node(
        package="legged_control",
        executable="vel_viz_node",
        name="vel_viz_node",
        output="log",
    )


def make_rviz2():
    cfg = os.path.join(_LEGGED_SHARE, "config", "test.rviz")
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", cfg],
        output="log",
    )
