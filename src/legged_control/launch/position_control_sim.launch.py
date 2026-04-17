"""RViz-based simulation launch for position_control using a URDF model."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_URDF = "/home/grayerd/Desktop/Projects/rc/塞北箭4urdf/urdf/塞北箭4urdf.urdf"


def _load_robot_cfg() -> dict:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _robot_description_from_urdf(path: str) -> str:
    urdf_dir = os.path.dirname(path)
    package_root = os.path.dirname(urdf_dir)
    mesh_root = os.path.join(package_root, "meshes")
    with open(path) as f:
        content = f.read()
    return content.replace("package://塞北箭4urdf/meshes/", f"file://{mesh_root}/")


def _launch_setup(context, *args, **kwargs):
    cfg = _load_robot_cfg()
    share = get_package_share_directory("legged_control")
    joints = cfg["joints"]
    front_joints = [
        joint["name"] for joint in joints if joint["name"].split("_")[0] in ("FR", "FL")
    ]
    rear_joints = [
        joint["name"] for joint in joints if joint["name"].split("_")[0] in ("RR", "RL")
    ]
    loop_hz = float(cfg["control"].get("gait_hz", 50.0))
    robot_description = _robot_description_from_urdf(
        LaunchConfiguration("urdf_path").perform(context)
    )

    return [
        Node(
            package="legged_control",
            executable="fake_motor_bus_node",
            name="motor_bus_front",
            parameters=[{"joint_names": front_joints, "loop_hz": loop_hz}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="fake_motor_bus_node",
            name="motor_bus_rear",
            parameters=[{"joint_names": rear_joints, "loop_hz": loop_hz}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="joint_aggregator",
            name="joint_aggregator",
            output="screen",
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
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="gait_node",
            name="gait_node",
            parameters=[{"config_path": LaunchConfiguration("gait_config_path")}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="urdf_joint_state_bridge",
            name="urdf_joint_state_bridge",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                os.path.join(share, "config", "position_control_sim.rviz"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("legged_control")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=_DEFAULT_URDF,
                description="Absolute path to the quadruped URDF file",
            ),
            DeclareLaunchArgument(
                "gait_config_path",
                default_value=os.path.join(share, "config", "robot_sim.yaml"),
                description="Config file used by gait_node for simulation defaults",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz2",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
