"""Shared launch helpers — processing nodes + visualization only.

These nodes read standard topics and produce obs/visualization data.
They work identically whether the data comes from real hardware or Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

_LEGGED_SHARE = get_package_share_directory("legged_control")

# ── processing layer ──────────────────────────────────────────────────────────

def make_state_estimator(config_path: str = ""):
    params = {"config_path": config_path} if config_path else {}
    return Node(
        package="legged_control", executable="state_estimator_node",
        name="state_estimator_node", parameters=[params] if params else None,
        output="screen",
    )


def make_height_scan():
    return Node(
        package="legged_control", executable="height_scan_node",
        name="height_scan_node", output="log",
    )


def make_teleop():
    return Node(
        package="legged_control", executable="teleop_node",
        name="teleop_node", output="log",
    )


def make_obs_assembler():
    return Node(
        package="legged_control", executable="obs_assembler",
        name="obs_assembler", output="screen",
    )


# ── shared visualization ──────────────────────────────────────────────────────

def make_robot_state_publisher():
    try:
        dog_share = get_package_share_directory("dog_urdf")
    except Exception:
        return None
    urdf_path = os.path.join(dog_share, "urdf", "dog_urdf.urdf")
    with open(urdf_path) as f:
        desc = f.read()
    desc = desc.replace("__CONTROLLER_YAML__", "")
    desc = desc.replace("package://dog_urdf/", f"file://{dog_share}/")
    return Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": desc}], output="log",
    )


def make_obs_monitor():
    return Node(
        package="legged_control", executable="obs_monitor_node",
        name="obs_monitor_node", output="log",
    )


def make_monitor_node():
    return Node(
        package="legged_control", executable="monitor_node",
        name="monitor_node", output="screen",
    )


def make_vel_viz():
    return Node(
        package="legged_control", executable="vel_viz_node",
        name="vel_viz_node", output="log",
    )


def make_rviz2():
    cfg = os.path.join(_LEGGED_SHARE, "config", "test.rviz")
    return Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", cfg], output="log",
    )
