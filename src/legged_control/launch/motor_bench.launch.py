"""
motor_bench.launch.py — single-leg motor bench test.

Launch args:
  leg          [FR]               Leg to test: FR / FL / RR / RL (case-insensitive)
  joy_device   [/dev/input/js0]   Joystick device path

Usage:
  ros2 launch legged_control motor_bench.launch.py
  ros2 launch legged_control motor_bench.launch.py leg:=RL
  ros2 launch legged_control motor_bench.launch.py leg:=FL joy_device:=/dev/input/js1
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NS_MAP = {
    'FR': ['fr/hip', 'fr/thigh', 'fr/calf'],
    'FL': ['fl/hip', 'fl/thigh', 'fl/calf'],
    'RR': ['rr/hip', 'rr/thigh', 'rr/calf'],
    'RL': ['rl/hip', 'rl/thigh', 'rl/calf'],
}


def _load_robot_config() -> dict:
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def _launch_setup(context, *args, **kwargs):
    leg        = LaunchConfiguration('leg').perform(context).upper()
    joy_device = LaunchConfiguration('joy_device').perform(context)

    cfg         = _load_robot_config()
    control_cfg = cfg['control']
    serial_port = control_cfg['serial_port']
    motor_hz    = control_cfg['motor_hz']
    kp          = control_cfg['kp']
    kd          = control_cfg['kd']

    ns_list = _NS_MAP.get(leg)
    if ns_list is None:
        raise RuntimeError(
            f"motor_bench.launch: unknown leg '{leg}'. "
            f"Valid values: {list(_NS_MAP.keys())}")

    # Pick the 3 joints for this leg (same prefix logic as motor_bench_node)
    leg_joints = [j for j in cfg['joints']
                  if j['name'].upper().startswith(leg)]
    if len(leg_joints) != 3:
        raise RuntimeError(
            f"motor_bench.launch: expected 3 joints for leg '{leg}', "
            f"found {len(leg_joints)}")

    motor_nodes = [
        Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=ns,
            name='motor',
            parameters=[{
                'serial_port': serial_port,
                'motor_id':    j['motor_id'],
                'loop_hz':     motor_hz,
                'joint_name':  j['name'],
                'target_q':    j['default_q'],
                'target_dq':   0.0,
                'kp':          kp,
                'kd':          kd,
                'tau':         0.0,
            }],
            output='screen',
        )
        for j, ns in zip(leg_joints, ns_list)
    ]

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': joy_device}],
        output='screen',
    )

    bench = Node(
        package='legged_control',
        executable='motor_bench_node',
        name='motor_bench_node',
        parameters=[{'leg': leg}],
        output='screen',
    )

    return motor_nodes + [joy, bench]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('leg',        default_value='FR',
                              description='Leg to test: FR/FL/RR/RL'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0',
                              description='Joystick device path'),
        OpaqueFunction(function=_launch_setup),
    ])
