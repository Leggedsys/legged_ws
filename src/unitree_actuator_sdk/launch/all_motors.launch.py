"""
Launch all 12 Unitree GO-M8010-6 motors for a quadruped robot.

Hardware: all 12 motors on one RS-485 bus at /dev/ttyUSB0, IDs 0-11.

Joint layout:
  FR: hip=0, thigh=1, calf=2
  FL: hip=3, thigh=4, calf=5
  RR: hip=6, thigh=7, calf=8
  RL: hip=9, thigh=10, calf=11

Each motor node:
  - Subscribes to /joint_commands (JointState) filtered by joint_name
  - Publishes /<namespace>/joint_states at loop_hz
  - Falls back to target_q parameter if no /joint_commands received

Control law (executed on motor):
  tau = kp * (target_q - q) + kd * (0 - dq)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

SERIAL_PORT = '/dev/ttyUSB0'
LOOP_HZ = 1000.0
KP = 20.0
KD = 0.5

# (namespace, joint_name, motor_id, default_q_rad)
MOTORS = [
    ('fr/hip',   'FR_hip',    0,  0.0),
    ('fr/thigh', 'FR_thigh',  1,  0.8),
    ('fr/calf',  'FR_calf',   2, -1.5),
    ('fl/hip',   'FL_hip',    3,  0.0),
    ('fl/thigh', 'FL_thigh',  4,  0.8),
    ('fl/calf',  'FL_calf',   5, -1.5),
    ('rr/hip',   'RR_hip',    6,  0.0),
    ('rr/thigh', 'RR_thigh',  7,  0.8),
    ('rr/calf',  'RR_calf',   8, -1.5),
    ('rl/hip',   'RL_hip',    9,  0.0),
    ('rl/thigh', 'RL_thigh', 10,  0.8),
    ('rl/calf',  'RL_calf',  11, -1.5),
]


def generate_launch_description():
    nodes = []
    for ns, joint_name, motor_id, default_q in MOTORS:
        nodes.append(Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=ns,
            name='motor',
            parameters=[{
                'serial_port': SERIAL_PORT,
                'motor_id': motor_id,
                'loop_hz': LOOP_HZ,
                'joint_name': joint_name,
                'target_q': default_q,
                'target_dq': 0.0,
                'kp': KP,
                'kd': KD,
                'tau': 0.0,
            }],
            output='screen',
        ))
    return LaunchDescription(nodes)
