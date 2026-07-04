"""mpc_preview.launch.py — offline MPC RViz visualization, no hardware needed.

mpc_preview_node provides fake sensors and auto-triggers standup after 2 s.
Commanded joint positions are relayed to robot_state_publisher so the trot
gait is visible in RViz.  Joystick (teleop_node) controls cmd_vel in WALK.

Usage:
  ros2 launch legged_control mpc_preview.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from legged_control.launch_common import make_robot_state_publisher, make_rviz2


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(package="legged_control", executable="mpc_preview_node",
             name="mpc_preview_node", output="screen"),
        Node(package="legged_control", executable="mpc_node",
             name="mpc_node", output="screen"),
        Node(package="legged_control", executable="teleop_node",
             name="teleop_node", output="log"),
    ]

    rsp = make_robot_state_publisher()
    if rsp is not None:
        nodes.append(rsp)
    nodes.append(make_rviz2())

    return LaunchDescription(nodes)
