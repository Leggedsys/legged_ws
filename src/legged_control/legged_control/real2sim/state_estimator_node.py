"""state_estimator_node

Subscribes:
  odin1/imu/filtered  (sensor_msgs/Imu)   — orientation quaternion + angular_velocity
  /joint_states_aggregated (sensor_msgs/JointState) — motor-frame positions + velocities

Publishes:
  /state_estimate (std_msgs/Float32MultiArray, 9 floats)
    data[0:3] = base_lin_vel in yaw frame (m/s)
    data[3:6] = base_ang_vel in body frame (rad/s)
    data[6:9] = projected_gravity in body frame (unit vector)

Velocity estimation: complementary filter blending kinematic velocity
(assuming all four feet in contact) with IMU-integrated velocity.
Alpha = 0.8 (high trust in kinematics; adjust if drift is observed).
"""

from __future__ import annotations

import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from legged_control.kinematics import (
    leg_kinematic_velocity,
    projected_gravity_from_quat,
    yaw_rotation_matrix,
)

_LEG_ORDER = ("FL", "FR", "RL", "RR")


def _load_joint_cfg(config_path: str) -> dict[str, dict]:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return {j["name"]: j for j in cfg["joints"]}


def _motor_to_urdf(q_motor: float, direction: float, zero_offset: float) -> float:
    return direction * q_motor + zero_offset


def _leg_q_urdf(
    leg: str, joint_pos: dict[str, float], joint_cfg: dict[str, dict]
) -> tuple[float, float, float] | None:
    names = [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]
    if any(n not in joint_pos for n in names):
        return None
    return tuple(
        _motor_to_urdf(
            joint_pos[n],
            float(joint_cfg[n]["direction"]),
            float(joint_cfg[n]["zero_offset"]),
        )
        for n in names
    )


def _leg_dq_urdf(
    leg: str, joint_vel: dict[str, float], joint_cfg: dict[str, dict]
) -> tuple[float, float, float] | None:
    names = [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]
    if any(n not in joint_vel for n in names):
        return None
    return tuple(
        float(joint_cfg[n]["direction"]) * joint_vel[n] for n in names
    )


class StateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_estimator_node")

        self.declare_parameter("config_path", "")
        config_path = str(self.get_parameter("config_path").value or "").strip()
        if not config_path:
            share = get_package_share_directory("legged_control")
            config_path = os.path.join(share, "config", "robot.yaml")
        self._joint_cfg = _load_joint_cfg(config_path)

        self.declare_parameter("velocity_alpha", 0.8)

        self._quat = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
        self._ang_vel = (0.0, 0.0, 0.0)
        self._lin_vel = np.zeros(3)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._imu_ready = False

        self._pub = self.create_publisher(Float32MultiArray, "/state_estimate", 10)
        self.create_subscription(Imu, "odin1/imu/filtered", self._on_imu, 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joints, 10
        )
        self.get_logger().info("state_estimator_node ready")

    def _on_imu(self, msg: Imu) -> None:
        o = msg.orientation
        self._quat = (o.x, o.y, o.z, o.w)
        av = msg.angular_velocity
        self._ang_vel = (av.x, av.y, av.z)
        self._imu_ready = True
        self._publish()

    def _on_joints(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _estimate_velocity(self) -> np.ndarray:
        alpha = float(self.get_parameter("velocity_alpha").value)
        R_yaw = yaw_rotation_matrix(*self._quat)
        kin_velocities = []
        for leg in _LEG_ORDER:
            q = _leg_q_urdf(leg, self._joint_pos, self._joint_cfg)
            dq = _leg_dq_urdf(leg, self._joint_vel, self._joint_cfg)
            if q is None or dq is None:
                continue
            v_body = leg_kinematic_velocity(leg, q, dq)
            kin_velocities.append(R_yaw @ v_body)
        if not kin_velocities:
            return self._lin_vel
        v_kin = np.mean(kin_velocities, axis=0)
        self._lin_vel = alpha * v_kin + (1.0 - alpha) * self._lin_vel
        return self._lin_vel

    def _publish(self) -> None:
        if not self._imu_ready:
            return
        lin_vel = self._estimate_velocity()
        ang_vel = np.array(self._ang_vel)
        proj_grav = projected_gravity_from_quat(*self._quat)

        msg = Float32MultiArray()
        msg.data = [
            float(lin_vel[0]), float(lin_vel[1]), float(lin_vel[2]),
            float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2]),
            float(proj_grav[0]), float(proj_grav[1]), float(proj_grav[2]),
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
