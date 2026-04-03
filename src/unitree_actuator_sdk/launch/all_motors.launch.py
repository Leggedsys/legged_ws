"""
Launch all 12 Unitree GO-M8010-6 motors for a quadruped robot.

Hardware assumption: all 12 motors share one RS-485 bus on /dev/ttyUSB0,
addressed by motor_id 0–11. Adjust serial_port if motors are split across
multiple buses.

Joint layout (typical quadruped):
  FR (Front Right) : hip=0, thigh=1, calf=2
  FL (Front Left)  : hip=3, thigh=4, calf=5
  RR (Rear Right)  : hip=6, thigh=7, calf=8
  RL (Rear Left)   : hip=9, thigh=10, calf=11

Each node runs at 1000 Hz, publishes sensor_msgs/JointState on
  /<leg>/<joint>/joint_states
and receives commands via ROS2 parameter service (SetParameters).

Control law sent to motor each tick:
  tau_output = kp * (target_q - q) + kd * (target_dq - dq) + tau
"""

from launch import LaunchDescription
from launch_ros.actions import Node

SERIAL_PORT = '/dev/ttyUSB0'
LOOP_HZ = 1000.0

# (namespace, motor_id)
MOTORS = [
    ('fr/hip',   0),
    ('fr/thigh', 1),
    ('fr/calf',  2),
    ('fl/hip',   3),
    ('fl/thigh', 4),
    ('fl/calf',  5),
    ('rr/hip',   6),
    ('rr/thigh', 7),
    ('rr/calf',  8),
    ('rl/hip',   9),
    ('rl/thigh', 10),
    ('rl/calf',  11),
]


def generate_launch_description():
    nodes = []
    for ns, motor_id in MOTORS:
        nodes.append(Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=ns,
            name='motor',
            parameters=[{
                'serial_port': SERIAL_PORT,
                'motor_id': motor_id,
                'loop_hz': LOOP_HZ,
                # Safe defaults: zero torque, pure damping, stationary
                'target_q': 0.0,
                'target_dq': 0.0,
                'kp': 0.0,
                'kd': 0.05,
                'tau': 0.0,
            }],
            output='screen',
        ))
    return LaunchDescription(nodes)
