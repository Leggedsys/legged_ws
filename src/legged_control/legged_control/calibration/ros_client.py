"""rclpy node providing thread-safe access to joint states and joystick."""

from __future__ import annotations
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Joy
from nav_msgs.msg import Odometry

# Namespaces matching all_motors.launch.py
_MOTOR_NAMESPACES = [
    'fr/hip', 'fr/thigh', 'fr/calf',
    'fl/hip', 'fl/thigh', 'fl/calf',
    'rr/hip', 'rr/thigh', 'rr/calf',
    'rl/hip', 'rl/thigh', 'rl/calf',
]


class RosClient(Node):
    """Subscribes to all 12 motor joint_states, /joy, /odin1/imu, /odin1/odometry.
    Publishes to /joint_commands.

    All public getters are thread-safe.
    """

    def __init__(self) -> None:
        super().__init__('calibrator')
        self._lock = threading.Lock()

        # Joint state cache: name → (position, velocity)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}

        # Latest joy and imu/odom for Phase 4 display
        self._joy: Joy | None = None
        self._imu: Imu | None = None
        self._odom: Odometry | None = None

        # Subscribe to each motor's individual topic
        self._subs = []
        for ns in _MOTOR_NAMESPACES:
            self._subs.append(self.create_subscription(
                JointState, f'/{ns}/joint_states',
                self._make_joint_cb(), 10))

        self.create_subscription(Joy,      '/joy',              self._on_joy,  10)
        self.create_subscription(Imu,      '/odin1/imu',        self._on_imu,  10)
        self.create_subscription(Odometry, '/odin1/odometry',   self._on_odom, 10)

        self._cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

    def _make_joint_cb(self):
        def cb(msg: JointState) -> None:
            with self._lock:
                for i, name in enumerate(msg.name):
                    if i < len(msg.position):
                        self._joint_pos[name] = msg.position[i]
                    if i < len(msg.velocity):
                        self._joint_vel[name] = msg.velocity[i]
        return cb

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._joy = msg

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self._imu = msg

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg

    def get_joint_positions(self) -> dict[str, float]:
        with self._lock:
            return dict(self._joint_pos)

    def get_joint_velocities(self) -> dict[str, float]:
        with self._lock:
            return dict(self._joint_vel)

    def get_joy(self) -> Joy | None:
        with self._lock:
            return self._joy

    def get_imu(self) -> Imu | None:
        with self._lock:
            return self._imu

    def get_odom(self) -> Odometry | None:
        with self._lock:
            return self._odom

    def send_command(self, targets: dict[str, float]) -> None:
        """Publish target joint positions to /joint_commands."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets.keys())
        msg.position = list(targets.values())
        self._cmd_pub.publish(msg)
