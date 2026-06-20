"""state_estimator_node

Subscribes:
  odin1/imu/filtered        (sensor_msgs/Imu)         — orientation + angular_velocity
  /odin1/odometry           (nav_msgs/Odometry)       — VIO linear velocity
  /joint_states_aggregated  (sensor_msgs/JointState)   — URDF-frame joint states

Publishes:
  /state_estimate (std_msgs/Float32MultiArray, 10 floats)
    data[0:3] = base_lin_vel in body frame (m/s)
    data[3:6] = base_ang_vel in body frame (rad/s)
    data[6:9] = projected_gravity in body frame (unit vector)
    data[9]   = health flag (1.0 = usable, 0.0 = IMU not ready)
                obs_assembler reads only data[:9]; policy_node gates on data[9].

Velocity: prefers VIO odometry (stable, no foot-slip assumption).
Falls back to leg kinematics if odometry is unavailable.
"""

from __future__ import annotations

import os
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from legged_control.kinematics import (
    leg_kinematic_velocity,
    projected_gravity_from_quat,
    quat_rotate_inverse,
)

_LEG_ORDER = ("FL", "FR", "RL", "RR")
_ODOM_TIMEOUT = 0.15  # seconds before VIO considered stale


def _accept_gravity(raw: np.ndarray) -> bool:
    """Accept a projected-gravity sample iff it is a valid unit vector.

    Only the magnitude is checked — the direction is trusted at any tilt so the
    estimate keeps tracking through large pitch/roll (a directional gate would
    freeze the estimate when the robot leans far over).
    """
    n = float(np.linalg.norm(raw))
    return 0.9 < n < 1.1


def _leg_q_urdf(
    leg: str, joint_pos: dict[str, float]
) -> tuple[float, float, float] | None:
    names = [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]
    if any(n not in joint_pos for n in names):
        return None
    return tuple(joint_pos[n] for n in names)


def _leg_dq_urdf(
    leg: str, joint_vel: dict[str, float]
) -> tuple[float, float, float] | None:
    names = [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]
    if any(n not in joint_vel for n in names):
        return None
    return tuple(joint_vel[n] for n in names)


class StateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_estimator_node")

        self._quat = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
        self._ang_vel = (0.0, 0.0, 0.0)
        self._lin_vel = np.zeros(3)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._imu_ready = False
        self._proj_grav = np.array([0.0, 0.0, -1.0], dtype=np.float32)

        # VIO odometry. Despite child_frame_id=odin1_base_link, the Odin twist is
        # empirically in the WORLD (odom) frame (verified on hardware via the
        # vel_viz arrow: it stayed world-fixed as the robot yawed). It is rotated
        # into the body frame in _estimate_velocity to match training.
        self._odom_lin_vel = np.zeros(3)       # world frame m/s (odom)
        self._odom_stamp: float | None = None  # monotonic timestamp

        self._pub = self.create_publisher(Float32MultiArray, "/state_estimate", 10)
        publish_hz = float(self._load_publish_hz())
        self._publish_timer = self.create_timer(1.0 / publish_hz, self._publish)
        self.create_subscription(Imu, "odin1/imu/filtered", self._on_imu, 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Odometry, "/odin1/odometry", self._on_odom, 10)
        self.get_logger().info(
            f"state_estimator_node ready — VIO + kinematics  publish_hz={publish_hz:.0f}"
        )

    def _load_publish_hz(self) -> float:
        try:
            from ament_index_python.packages import get_package_share_directory
            with open(os.path.join(get_package_share_directory("legged_control"),
                                   "config", "robot.yaml")) as f:
                return float(yaml.safe_load(f).get("control", {}).get("estimator_hz", 200.0))
        except Exception:
            return 200.0

    def _on_imu(self, msg: Imu) -> None:
        o = msg.orientation
        self._quat = (o.x, o.y, o.z, o.w)
        av = msg.angular_velocity
        self._ang_vel = (av.x, av.y, av.z)
        self._imu_ready = True

    def _on_joints(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self._odom_lin_vel = np.array([t.linear.x, t.linear.y, t.linear.z])
        self._odom_stamp = time.monotonic()

    def _estimate_velocity(self) -> np.ndarray:
        now = time.monotonic()

        # Primary: VIO odometry. Rotate the world-frame velocity into the body
        # frame (quat_rotate_inverse, matching training's base_lin_vel).
        if self._odom_stamp is not None and (now - self._odom_stamp) < _ODOM_TIMEOUT:
            v_body = quat_rotate_inverse(*self._quat, *self._odom_lin_vel)
            self._lin_vel = 0.6 * v_body + 0.4 * self._lin_vel
            return self._lin_vel

        # Fallback: leg kinematics (Odin disconnected)
        kin_velocities = []
        for leg in _LEG_ORDER:
            q = _leg_q_urdf(leg, self._joint_pos)
            dq = _leg_dq_urdf(leg, self._joint_vel)
            if q is None or dq is None:
                continue
            v_body = leg_kinematic_velocity(leg, q, dq)
            kin_velocities.append(v_body)
        if kin_velocities:
            v_kin = np.mean(kin_velocities, axis=0)
            self._lin_vel = 0.8 * v_kin + 0.2 * self._lin_vel
        return self._lin_vel

    def _publish(self) -> None:
        if not self._imu_ready:
            self.get_logger().info(
                "state_estimator: IMU not ready — publishing g=[0,0,-1], health=0",
                throttle_duration_sec=5.0,
            )
            msg = Float32MultiArray()
            # data[9] = health flag (0.0 = not usable); consumers gate on it.
            msg.data = [0.0] * 6 + [0.0, 0.0, -1.0] + [0.0]
            self._pub.publish(msg)
            return
        lin_vel = self._estimate_velocity()
        ang_vel = np.array(self._ang_vel)
        raw = projected_gravity_from_quat(*self._quat)
        if raw[2] > 0.0:
            raw = -raw
        self._proj_grav = raw

        msg = Float32MultiArray()
        msg.data = [
            float(lin_vel[0]), float(lin_vel[1]), float(lin_vel[2]),
            float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2]),
            float(self._proj_grav[0]), float(self._proj_grav[1]), float(self._proj_grav[2]),
            1.0,  # data[9] = health flag (1.0 = usable)
        ]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
