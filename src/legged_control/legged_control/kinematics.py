"""Pure single-leg kinematics derived from the quadruped URDF."""

from __future__ import annotations

import math

import numpy as np


HIP_Y_OFFSET = 0.0204632257018415
THIGH_Y_OFFSET = 0.0922
D_LAT = HIP_Y_OFFSET + THIGH_Y_OFFSET
L_HIP_X = 0.0645133382530963
L2 = 0.179998920942403
L3 = 0.181256552442634

LEFT_LEGS = {"FL", "RL"}
RIGHT_LEGS = {"FR", "RR"}
FRONT_LEGS = {"FL", "FR"}
REAR_LEGS = {"RL", "RR"}


def _validate_leg(leg: str) -> str:
    leg = leg.upper()
    if leg not in LEFT_LEGS | RIGHT_LEGS:
        raise ValueError(f"Unknown leg '{leg}'")
    return leg


def _leg_signs(leg: str) -> tuple[float, float, float]:
    """Return (hip axis sign, lateral offset sign, fore-aft x sign).

    hip_sign: maps URDF q1 to physical hip rotation (+1 = forward, -1 = backward).
        FL +1, FR -1 (flipped), RL +1 (flipped), RR -1.
    lat_sign: lateral offset direction (+1 = left, -1 = right), used for D_LAT.
    x_sign: fore-aft sign (+1 = front, -1 = rear), used for L_HIP_X.
    """
    leg = _validate_leg(leg)
    hip_sign = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}[leg]
    lat_sign = 1.0 if leg in LEFT_LEGS else -1.0
    x_sign = 1.0 if leg in FRONT_LEGS else -1.0
    return hip_sign, lat_sign, x_sign


def forward_kinematics(
    leg: str, joints: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Return foot position in the hip frame from URDF-frame leg angles."""
    hip_sign, lat_sign, x_sign = _leg_signs(leg)
    q1, q2, q3 = joints

    hip_angle = hip_sign * q1
    thigh_angle = q2
    calf_angle = q3

    x = (
        x_sign * L_HIP_X
        - L2 * math.sin(thigh_angle)
        - L3 * math.sin(thigh_angle + calf_angle)
    )
    z_plane = -L2 * math.cos(thigh_angle) - L3 * math.cos(thigh_angle + calf_angle)
    y_plane = lat_sign * D_LAT

    y = y_plane * math.cos(hip_angle) - z_plane * math.sin(hip_angle)
    z = y_plane * math.sin(hip_angle) + z_plane * math.cos(hip_angle)
    return x, y, z


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _joint_distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sum(abs(_wrap_angle(x - y)) for x, y in zip(a, b))


def inverse_kinematics(
    leg: str,
    foot_pos: tuple[float, float, float],
    preferred_joints: tuple[float, float, float] | None = None,
) -> tuple[float, float, float] | None:
    """Return URDF-frame (hip, thigh, calf) angles for a foot target in hip frame."""
    hip_sign, lat_sign, x_sign = _leg_signs(leg)
    x, y, z = foot_pos

    yz_norm = math.hypot(y, z)
    if yz_norm < D_LAT:
        return None

    acos_arg = max(-1.0, min(1.0, (lat_sign * D_LAT) / yz_norm))
    hip_angle = math.atan2(z, y) + math.acos(acos_arg)
    q1 = hip_angle / hip_sign

    z_plane = -y * math.sin(hip_angle) + z * math.cos(hip_angle)
    x_prime = x - x_sign * L_HIP_X

    reach_sq = x_prime * x_prime + z_plane * z_plane
    cos_calf = (reach_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if cos_calf < -1.0 or cos_calf > 1.0:
        return None

    cos_calf = max(-1.0, min(1.0, cos_calf))

    candidates = []
    for calf_sign in (-1.0, 1.0):
        q3 = calf_sign * math.acos(cos_calf)
        calf_angle = q3
        u = -x_prime
        v = -z_plane
        thigh_angle = math.atan2(u, v) - math.atan2(
            L3 * math.sin(calf_angle),
            L2 + L3 * math.cos(calf_angle),
        )
        q2 = thigh_angle
        candidates.append((q1, q2, q3))

    if preferred_joints is not None:
        return min(candidates, key=lambda q: _joint_distance(q, preferred_joints))

    return min(candidates, key=lambda q: abs(q[2]))


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _numerical_jacobian(
    leg: str, q_urdf: tuple[float, float, float], eps: float = 1e-4
) -> np.ndarray:
    p0 = np.array(forward_kinematics(leg, q_urdf))
    J = np.zeros((3, 3))
    for i in range(3):
        q_plus = list(q_urdf)
        q_plus[i] += eps
        p_plus = np.array(forward_kinematics(leg, tuple(q_plus)))
        J[:, i] = (p_plus - p0) / eps
    return J


def leg_kinematic_velocity(
    leg: str,
    q_urdf: tuple[float, float, float],
    dq_urdf: tuple[float, float, float],
) -> np.ndarray:
    J = _numerical_jacobian(leg, q_urdf)
    dq = np.array(dq_urdf)
    return -J @ dq


def projected_gravity_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),        1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])
    g_world = np.array([0.0, 0.0, -1.0])
    return R.T @ g_world


def yaw_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy, -sy, 0.0],
        [sy,  cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
