"""real.launch.py — all real-hardware data-source nodes.

Starts:
  motor_bus_node ×2   kp/kd from config (or overridden)
  joint_aggregator     merge + motor→URDF
  motor_command_bridge URDF cmd → motor frame
  urdf_joint_state_bridge → /joint_states (TF tree)
  host_sdk_sample      Odin1 IMU
  imu_filter_madgwick  IMU filter
  realsense2_camera    depth camera (with topic remap)
  joy_node             gamepad
  static TF            odin1_base_link → base_link

Launch args:
  serial_port_front  [/dev/ttyUSB0]
  serial_port_rear   [/dev/ttyUSB1]
  legs               [all]
  kp_override        [-1]   negative = use robot.yaml control.kp
  kd_override        [-1]   negative = use robot.yaml control.kd
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
    kp_override = float(LaunchConfiguration("kp_override").perform(context))
    kd_override = float(LaunchConfiguration("kd_override").perform(context))

    if sp_front == _YAML_SENTINEL:
        sp_front = control["serial_port_front"]
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control["serial_port_rear"]

    motor_hz = float(control["motor_hz"])
    kp = kp_override if kp_override >= 0 else float(control["kp"])
    kd = kd_override if kd_override >= 0 else float(control["kd"])

    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg["joints"] if j["name"].split("_")[0] in active_legs]

    port_map = {"front": sp_front, "rear": sp_rear}
    groups = {
        "front": ("motor_bus_front", port_map["front"]),
        "rear":  ("motor_bus_rear",  port_map["rear"]),
    }

    nodes = []

    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package="legged_control", executable="motor_bus_node", name=node_name,
            parameters=[{"serial_port": port,
                         "joint_names": [j["name"] for j in group_joints],
                         "kp": kp, "kd": kd, "loop_hz": motor_hz}],
            remappings=[("/joint_commands", "/joint_commands_motor")],
            output="log",
        ))

    nodes += [
        Node(package="legged_control", executable="joint_aggregator",
             name="joint_aggregator", output="screen"),
        Node(package="legged_control", executable="motor_command_bridge",
             name="motor_command_bridge", output="log"),
        Node(package="legged_control", executable="urdf_joint_state_bridge",
             name="urdf_joint_state_bridge", output="log"),
        Node(package="odin_ros_driver", executable="host_sdk_sample",
             name="odin1_node", output="log"),
        Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
             name="imu_filter_madgwick",
             parameters=[{"use_mag": False, "publish_tf": False,
                          "fixed_frame": "base_link", "world_frame": "enu"}],
             remappings=[("imu/data_raw", "odin1/imu"),
                         ("imu/data", "odin1/imu/filtered")],
             output="log"),
        Node(package="realsense2_camera", executable="realsense2_camera_node",
             name="camera",
             remappings=[("/camera/camera/depth/camera_info", "/camera/depth/camera_info"),
                         ("/camera/camera/depth/image_rect_raw", "/camera/depth/image_rect_raw")],
             output="log"),
        Node(package="joy", executable="joy_node", name="joy_node", output="log"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="odin_base_tf",
             arguments=["-0.08558", "0.0349", "-0.142", "0.0", "0.0", "0.0",
                        "odin1_base_link", "base_link"],
             output="log"),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port_front", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("serial_port_rear", default_value=_YAML_SENTINEL),
        DeclareLaunchArgument("legs", default_value="all"),
        DeclareLaunchArgument("kp_override", default_value="-1"),
        DeclareLaunchArgument("kd_override", default_value="-1"),
        OpaqueFunction(function=_launch_setup),
    ])
