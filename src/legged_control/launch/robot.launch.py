"""
robot.launch.py — unified launch for all robot operating modes.

Launch args:
  mode          [passive]           passive | stand | standup | position_control
  legs          [all]               all | FR | FL | RR | RL | comma-separated e.g. FR,FL
  serial_port_front   [from robot.yaml]   Override serial port for FR/FL motors
  serial_port_rear    [from robot.yaml]   Override serial port for RR/RL motors

Usage:
  ros2 launch legged_control robot.launch.py
  ros2 launch legged_control robot.launch.py mode:=stand
  ros2 launch legged_control robot.launch.py mode:=standup
  ros2 launch legged_control robot.launch.py legs:=FR
  ros2 launch legged_control robot.launch.py legs:=FR,RL mode:=stand
  ros2 launch legged_control robot.launch.py serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_YAML_SENTINEL = "__from_yaml__"


def _load_config() -> dict:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _leg_group(joint_name: str) -> str:
    """'FR_hip' → 'front',  'RR_hip' → 'rear'"""
    return "front" if joint_name.split("_")[0] in ("FR", "FL") else "rear"


def _bus_nodes(
    joints: list, port_map: dict, motor_hz: float, kp: float, kd: float
) -> list:
    """Start one motor_bus_node per non-empty leg group."""
    groups = {
        "front": ("motor_bus_front", port_map["front"]),
        "rear": ("motor_bus_rear", port_map["rear"]),
    }
    nodes = []
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j["name"]) == group]
        if not group_joints:
            continue
        nodes.append(
            Node(
                package="legged_control",
                executable="motor_bus_node",
                name=node_name,
                parameters=[
                    {
                        "serial_port": port,
                        "joint_names": [j["name"] for j in group_joints],
                        "kp": kp,
                        "kd": kd,
                        "loop_hz": motor_hz,
                    }
                ],
                output="log",
            )
        )
    return nodes


_VALID_LEGS = {"FR", "FL", "RR", "RL"}


def _parse_legs(legs_arg: str) -> set:
    if legs_arg.upper() == "ALL":
        return _VALID_LEGS
    selected = {l.strip().upper() for l in legs_arg.split(",")}
    invalid = selected - _VALID_LEGS
    if invalid:
        raise RuntimeError(
            f"Unknown leg(s): {', '.join(sorted(invalid))}. "
            f"Valid values: FR, FL, RR, RL, all"
        )
    return selected


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()
    legs_arg = LaunchConfiguration("legs").perform(context)
    sp_front = LaunchConfiguration("serial_port_front").perform(context)
    sp_rear = LaunchConfiguration("serial_port_rear").perform(context)

    cfg = _load_config()
    control = cfg["control"]
    if sp_front == _YAML_SENTINEL:
        sp_front = control["serial_port_front"]
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control["serial_port_rear"]
    port_map = {"front": sp_front, "rear": sp_rear}
    motor_hz = float(control["motor_hz"])

    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg["joints"] if j["name"].split("_")[0] in active_legs]

    if mode == "passive":
        motors = _bus_nodes(joints, port_map, motor_hz, kp=0.0, kd=0.0)
        companion = Node(
            package="legged_control",
            executable="passive_monitor_node",
            name="passive_monitor_node",
            output="screen",
        )
        return motors + [companion]

    if mode == "stand":
        kp = float(control["kp"])
        kd = float(control["kd"])
        motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
        return motors + [
            Node(
                package="legged_control",
                executable="stand_node",
                name="stand_node",
                output="screen",
            ),
            Node(
                package="legged_control",
                executable="passive_monitor_node",
                name="passive_monitor_node",
                output="screen",
            ),
        ]

    if mode == "standup":
        kp = float(control["kp"])
        kd = float(control["kd"])
        motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
        return motors + [
            Node(
                package="legged_control",
                executable="joint_aggregator",
                name="joint_aggregator",
                output="screen",
            ),
            Node(
                package="legged_control",
                executable="standup_node",
                name="standup_node",
                output="screen",
            ),
        ]

    if mode == "position_control":
        if active_legs != _VALID_LEGS:
            raise RuntimeError(
                "mode:=position_control requires legs:=all because the gait controller uses a fixed 12-joint contract"
            )
        kp = float(control["kp"])
        kd = float(control["kd"])
        motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
        return motors + [
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
                output="screen",
            ),
        ]

    raise RuntimeError(
        f"Unknown mode '{mode}'. Valid modes: passive, stand, standup, position_control"
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="passive",
                description="Operating mode: passive | stand | standup | position_control",
            ),
            DeclareLaunchArgument(
                "legs",
                default_value="all",
                description="Legs to activate: all | FR | FL | RR | RL | comma-separated e.g. FR,FL",
            ),
            DeclareLaunchArgument(
                "serial_port_front",
                default_value=_YAML_SENTINEL,
                description="Serial port for front legs FR/FL (default: from robot.yaml)",
            ),
            DeclareLaunchArgument(
                "serial_port_rear",
                default_value=_YAML_SENTINEL,
                description="Serial port for rear legs RR/RL (default: from robot.yaml)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
