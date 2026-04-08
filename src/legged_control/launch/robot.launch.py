"""
robot.launch.py — unified launch for all robot operating modes.

Launch args:
  mode          [passive]           passive | stand | policy
  serial_port   [from robot.yaml]   Override serial port for all motors

Usage:
  ros2 launch legged_control robot.launch.py
  ros2 launch legged_control robot.launch.py mode:=stand
  ros2 launch legged_control robot.launch.py mode:=passive serial_port:=/dev/ttyUSB1
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NS_MAP = {
    'FR_hip':   'fr/hip',   'FR_thigh': 'fr/thigh', 'FR_calf': 'fr/calf',
    'FL_hip':   'fl/hip',   'FL_thigh': 'fl/thigh', 'FL_calf': 'fl/calf',
    'RR_hip':   'rr/hip',   'RR_thigh': 'rr/thigh', 'RR_calf': 'rr/calf',
    'RL_hip':   'rl/hip',   'RL_thigh': 'rl/thigh', 'RL_calf': 'rl/calf',
}

_YAML_SENTINEL = '__from_yaml__'


def _load_config() -> dict:
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def _motor_nodes(joints: list, serial_port: str, motor_hz: float,
                 kp: float, kd: float) -> list:
    return [
        Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=_NS_MAP[j['name']],
            name='motor',
            parameters=[{
                'serial_port': serial_port,
                'motor_id':    j['motor_id'],
                'loop_hz':     motor_hz,
                'joint_name':  j['name'],
                'target_q':    float(j['default_q']),
                'target_dq':   0.0,
                'kp':          kp,
                'kd':          kd,
                'tau':         0.0,
            }],
            output='log',
        )
        for j in joints
    ]


def _launch_setup(context, *args, **kwargs):
    mode        = LaunchConfiguration('mode').perform(context).lower()
    serial_port = LaunchConfiguration('serial_port').perform(context)

    cfg     = _load_config()
    control = cfg['control']
    if serial_port == _YAML_SENTINEL:
        serial_port = control['serial_port']
    motor_hz = float(control['motor_hz'])
    joints   = cfg['joints']

    if mode == 'passive':
        motors = _motor_nodes(joints, serial_port, motor_hz, kp=0.0, kd=0.0)
        companion = Node(
            package='legged_control',
            executable='passive_monitor_node',
            name='passive_monitor_node',
            output='screen',
        )
        return motors + [companion]

    if mode == 'stand':
        kp     = float(control['kp'])
        kd     = float(control['kd'])
        motors = _motor_nodes(joints, serial_port, motor_hz, kp=kp, kd=kd)
        companion = Node(
            package='legged_control',
            executable='stand_node',
            name='stand_node',
            output='screen',
        )
        return motors + [companion]

    if mode == 'policy':
        raise RuntimeError(
            "mode:=policy is not yet implemented. "
            "Available modes: passive, stand"
        )

    raise RuntimeError(
        f"Unknown mode '{mode}'. Valid modes: passive, stand, policy"
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='passive',
            description='Operating mode: passive | stand | policy',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value=_YAML_SENTINEL,
            description='Serial port override (default: value from robot.yaml)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
