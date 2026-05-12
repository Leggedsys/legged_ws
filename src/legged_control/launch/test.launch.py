"""test.launch.py — hardware data-link verification stack.

Starts the full real + real2sim pipeline in passive mode (zero torque)
together with visualization tools for validating sensor data.

Hardware layer (real/):
  motor_bus_front / motor_bus_rear  — joint position readout (kp=kd=0)
  joint_aggregator                  — aggregated joint states
  host_sdk_sample + imu_filter_madgwick  — IMU
  realsense2_camera_node            — depth camera
  joy_node                          — gamepad

Processing layer (real2sim/):
  state_estimator_node      — body velocity + gravity direction
  height_scan_node          — 325-point height scan
  teleop_node               — joystick → cmd_vel
  urdf_joint_state_bridge   — motor frame → URDF frame joint states

Test / visualization:
  robot_state_publisher     — URDF → /tf
  obs_monitor_node          — terminal obs display at 2 Hz
  vel_viz_node              — velocity arrows MarkerArray
  rviz2                     — RobotModel + HeightScan + VelocityArrows

Launch args:
  serial_port_front  [from robot.yaml]
  serial_port_rear   [from robot.yaml]
  legs               [all]
  rviz               [true]   set false to suppress RViz

Usage:
  ros2 launch legged_control test.launch.py
  ros2 launch legged_control test.launch.py rviz:=false
  ros2 launch legged_control test.launch.py legs:=FR,FL
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_YAML_SENTINEL = "__from_yaml__"
_VALID_LEGS = {"FR", "FL", "RR", "RL"}

try:
    _DOG_URDF_SHARE = get_package_share_directory("dog_urdf")
except Exception:
    _DOG_URDF_SHARE = ""


def _robot_description() -> str:
    if not _DOG_URDF_SHARE:
        return ""
    urdf_path = os.path.join(_DOG_URDF_SHARE, "urdf", "dog_urdf.urdf")
    with open(urdf_path) as f:
        content = f.read()
    content = content.replace("__CONTROLLER_YAML__", "")
    content = content.replace("package://dog_urdf/", f"file://{_DOG_URDF_SHARE}/")
    return content


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
    sp_rear = LaunchConfiguration("serial_port_rear").perform(context)
    legs_arg = LaunchConfiguration("legs").perform(context)

    if sp_front == _YAML_SENTINEL:
        sp_front = control["serial_port_front"]
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control["serial_port_rear"]

    motor_hz = float(control["motor_hz"])
    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg["joints"] if j["name"].split("_")[0] in active_legs]

    nodes = []

    # ── real layer ────────────────────────────────────────────────────────────

    groups = {
        "front": ("motor_bus_front", sp_front),
        "rear": ("motor_bus_rear", sp_rear),
    }
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package="legged_control",
            executable="motor_bus_node",
            name=node_name,
            parameters=[{
                "serial_port": port,
                "joint_names": [j["name"] for j in group_joints],
                "kp": 0.0,
                "kd": 0.0,
                "loop_hz": motor_hz,
            }],
            output="log",
        ))

    nodes.append(Node(
        package="legged_control",
        executable="joint_aggregator",
        name="joint_aggregator",
        output="screen",
    ))

    nodes += [
        Node(
            package="odin_ros_driver",
            executable="host_sdk_sample",
            name="odin1_node",
            output="log",
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
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            remappings=[
                ("/camera/camera/depth/camera_info", "/camera/depth/camera_info"),
                ("/camera/camera/depth/image_rect_raw", "/camera/depth/image_rect_raw"),
            ],
            output="log",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="log",
        ),
    ]

    # ── real2sim layer ────────────────────────────────────────────────────────

    nodes += [
        Node(
            package="legged_control",
            executable="state_estimator_node",
            name="state_estimator_node",
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="height_scan_node",
            name="height_scan_node",
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
            executable="urdf_joint_state_bridge",
            name="urdf_joint_state_bridge",
            output="log",
        ),
    ]

    # ── static transforms ────────────────────────────────────────────────────

    # base_link → camera_link: measure physical install position.
    # X=forward, Y=left, Z=up, PITCH=downward positive.
    nodes.append(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_base_tf",
        arguments=["0.15", "0.0", "0.12", "0.0", "0.5236", "0.0",
                   "base_link", "camera_link"],
        output="log",
    ))

    # ── test / visualization layer ────────────────────────────────────────────

    robot_desc = _robot_description()
    if robot_desc:
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
            output="log",
        ))

    nodes += [
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
    ]

    share = get_package_share_directory("legged_control")
    rviz_cfg = os.path.join(share, "config", "test.rviz")
    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
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
            description="Legs to activate: all | FR | FL | RR | RL | comma-separated",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 (true/false)",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
