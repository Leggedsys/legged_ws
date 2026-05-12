"""real.launch.py — hardware interface layer.

Starts all nodes that talk directly to physical hardware:
  - motor_bus_node (×2, front/rear)  : read joint positions, accept joint commands
  - odin1_node + imu_filter_madgwick : IMU
  - realsense2_camera_node           : depth camera
  - joy_node                         : gamepad

Published topics (outputs to upper layers):
  /joint_states_aggregated  sensor_msgs/JointState   12-joint positions & velocities
  odin1/imu/filtered        sensor_msgs/Imu          filtered IMU
  /camera/color/image_raw   sensor_msgs/Image        RGB image
  /camera/depth/...         sensor_msgs/Image        depth
  /joy                      sensor_msgs/Joy           gamepad axes/buttons

Subscribed topics (inputs from upper layers):
  /joint_commands           sensor_msgs/JointState   12-joint position targets

Launch args:
  serial_port_front  [/dev/ttyUSB0]   serial port for FR/FL motors
  serial_port_rear   [/dev/ttyUSB1]   serial port for RR/RL motors
  legs               [all]            all | FR | FL | RR | RL | comma-separated

Usage:
  ros2 launch legged_control real.launch.py
  ros2 launch legged_control real.launch.py serial_port_front:=/dev/ttyUSB0
  ros2 launch legged_control real.launch.py legs:=FR
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_VALID_LEGS = {"FR", "FL", "RR", "RL"}
_YAML_SENTINEL = "__from_yaml__"


def _load_config() -> dict:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _leg_group(joint_name: str) -> str:
    return "front" if joint_name.split("_")[0] in ("FR", "FL") else "rear"


def _parse_legs(legs_arg: str) -> set:
    if legs_arg.upper() == "ALL":
        return _VALID_LEGS
    selected = {leg.strip().upper() for leg in legs_arg.split(",")}
    invalid = selected - _VALID_LEGS
    if invalid:
        raise RuntimeError(
            f"Unknown leg(s): {', '.join(sorted(invalid))}. "
            f"Valid: FR, FL, RR, RL, all"
        )
    return selected


def _launch_setup(context, *args, **kwargs):
    cfg = _load_config()
    control = cfg["control"]

    sp_front = LaunchConfiguration("serial_port_front").perform(context)
    sp_rear  = LaunchConfiguration("serial_port_rear").perform(context)
    legs_arg = LaunchConfiguration("legs").perform(context)

    if sp_front == _YAML_SENTINEL:
        sp_front = control["serial_port_front"]
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control["serial_port_rear"]

    motor_hz = float(control["motor_hz"])
    kp = float(control["kp"])
    kd = float(control["kd"])

    active_legs = _parse_legs(legs_arg)
    joints = [
        j for j in cfg["joints"]
        if j["name"].split("_")[0] in active_legs
    ]

    port_map = {"front": sp_front, "rear": sp_rear}
    groups = {
        "front": ("motor_bus_front", port_map["front"]),
        "rear":  ("motor_bus_rear",  port_map["rear"]),
    }

    nodes = []

    # motor_bus_node × 2 (front / rear)
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package="legged_control",
            executable="motor_bus_node",
            name=node_name,
            parameters=[{
                "serial_port":  port,
                "joint_names":  [j["name"] for j in group_joints],
                "kp":           kp,
                "kd":           kd,
                "loop_hz":      motor_hz,
            }],
            output="log",
        ))

    # joint_aggregator — merges 12 individual joint topics into one
    nodes.append(Node(
        package="legged_control",
        executable="joint_aggregator",
        name="joint_aggregator",
        output="screen",
    ))

    # IMU
    nodes += [
        Node(
            package="odin_ros_driver",
            executable="odin1_node",
            name="odin1_node",
            output="log",
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            parameters=[{
                "use_mag":     False,
                "publish_tf":  False,
                "fixed_frame": "base_link",
                "world_frame": "enu",
            }],
            remappings=[
                ("imu/data_raw", "odin1/imu"),
                ("imu/data",     "odin1/imu/filtered"),
            ],
            output="log",
        ),
    ]

    # Depth camera
    nodes.append(Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        output="log",
    ))

    # Gamepad
    nodes.append(Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="log",
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port_front",
            default_value=_YAML_SENTINEL,
            description="Serial port for FR/FL motors (default: from robot.yaml)",
        ),
        DeclareLaunchArgument(
            "serial_port_rear",
            default_value=_YAML_SENTINEL,
            description="Serial port for RR/RL motors (default: from robot.yaml)",
        ),
        DeclareLaunchArgument(
            "legs",
            default_value="all",
            description=(
                "Legs to activate: all | FR | FL | RR | RL | comma-separated"
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
