"""
locomotion.launch.py — full first-level RL locomotion stack.

Starts:
  - 12 motor driver nodes (unitree_actuator_sdk)
  - joint_aggregator (legged_control)
  - joy_node         (joy)
  - teleop_node      (legged_control)
  - policy_node      (legged_control)
  - watchdog_node    (legged_control)
  - odin1 lidar      (odin_ros_driver, optional via launch arg)

Launch args:
  with_lidar      [true]              Start Odin1 lidar driver (needs USB connection)
  model_path      ['']                Path to TorchScript .pt policy file; empty = stand-still
  joy_device      [/dev/input/js0]    Joystick device path
  watchdog_timeout [0.5]              Seconds of policy silence before fallback triggers
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_robot_config() -> dict:
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg         = load_robot_config()
    joints_cfg  = cfg['joints']
    control_cfg = cfg['control']

    serial_port = control_cfg['serial_port']
    motor_hz    = control_cfg['motor_hz']
    kp          = control_cfg['kp']
    kd          = control_cfg['kd']

    # ---------- launch arguments ----------
    with_lidar_arg = DeclareLaunchArgument(
        'with_lidar', default_value='true',
        description='Start Odin1 lidar driver')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='TorchScript .pt policy file; empty = stand-still')
    joy_device_arg = DeclareLaunchArgument(
        'joy_device', default_value='/dev/input/js0',
        description='Joystick device path')
    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout', default_value='0.5',
        description='Seconds of policy silence before fallback triggers')

    with_lidar       = LaunchConfiguration('with_lidar')
    model_path       = LaunchConfiguration('model_path')
    joy_device       = LaunchConfiguration('joy_device')
    watchdog_timeout = LaunchConfiguration('watchdog_timeout')

    # ---------- motor nodes (one per joint) ----------
    motor_nodes = []
    ns_map = {
        'FR_hip': 'fr/hip',   'FR_thigh': 'fr/thigh', 'FR_calf': 'fr/calf',
        'FL_hip': 'fl/hip',   'FL_thigh': 'fl/thigh', 'FL_calf': 'fl/calf',
        'RR_hip': 'rr/hip',   'RR_thigh': 'rr/thigh', 'RR_calf': 'rr/calf',
        'RL_hip': 'rl/hip',   'RL_thigh': 'rl/thigh', 'RL_calf': 'rl/calf',
    }
    for j in joints_cfg:
        motor_nodes.append(Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=ns_map[j['name']],
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
        ))

    # ---------- control stack ----------
    joint_aggregator = Node(
        package='legged_control',
        executable='joint_aggregator',
        name='joint_aggregator',
        output='screen',
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': joy_device}],
        output='screen',
    )

    teleop = Node(
        package='legged_control',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
    )

    policy = Node(
        package='legged_control',
        executable='policy_node',
        name='policy_node',
        parameters=[{'model_path': model_path}],
        output='screen',
    )

    watchdog = Node(
        package='legged_control',
        executable='watchdog_node',
        name='watchdog_node',
        parameters=[{'timeout': watchdog_timeout}],
        output='screen',
    )

    # ---------- optional lidar ----------
    odin_launch_dir = os.path.join(
        get_package_share_directory('odin_ros_driver'), 'launch_ROS2')
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odin_launch_dir, 'odin1_ros2.launch.py')),
        condition=IfCondition(with_lidar),
    )

    return LaunchDescription([
        with_lidar_arg,
        model_path_arg,
        joy_device_arg,
        watchdog_timeout_arg,
        *motor_nodes,
        joint_aggregator,
        joy,
        teleop,
        policy,
        watchdog,
        lidar,
    ])
