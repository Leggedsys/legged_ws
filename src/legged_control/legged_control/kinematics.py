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

    hip_sign: maps URDF q1 to physical hip rotation. +1 for left legs, -1 for right
        legs — ensures positive q1 = abduction (外展) for all four legs.
    lat_sign: lateral offset direction (+1 = left, -1 = right), used for D_LAT.
    x_sign: fore-aft sign (+1 = front, -1 = rear), used for L_HIP_X.
    """
    leg = _validate_leg(leg)
    hip_sign = 1.0 if leg in LEFT_LEGS else -1.0
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


# Leg link inertials from dog_urdf (FL values; the other legs mirror through
# lat_sign/hip_sign). Only the components that move potential energy matter:
# lateral COM offsets (they load the hip ROLL joint — the largest term, the
# whole leg's mass hangs D_LAT-ish outboard of the roll axis) and the planar
# distances below each pitch joint. The thigh's 0.98 kg sits 13 mm from the
# hip axis (motors live at the hip), so the pitch-plane terms are small.
_M_HIP = 0.230146330118093
_Y_HIP_COM = 0.00174712967469963          # hip-link COM lateral offset (m)
_M_THIGH = 0.978596689781306
_D_THIGH_COM = 0.0127549994713949         # thigh COM below thigh joint (m)
_Y_THIGH_COM = HIP_Y_OFFSET + 0.0530216135226285
_M_FOOT = 0.0179075406761494
_M_CALF = 0.174854579981966 + _M_FOOT     # foot (fixed joint) lumped in
_D_CALF_COM = (
    0.174854579981966 * 0.0670819670227258
    + _M_FOOT * (0.181256552442634 - 0.00474142838422875)
) / _M_CALF                               # combined COM below knee (m)
_GRAV = 9.81


def leg_gravity_torque(
    leg: str, joints: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Holding torque (Nm, URDF joint frame) against the leg's OWN link
    weights, with the body level: tau = dU/dq of the leg's gravitational
    potential. Add as feedforward on a SWINGING leg so the PD no longer
    generates these torques out of position error (droop). Magnitudes:
    hip roll ~0.92 Nm at q2=q3=0 (lateral offsets), thigh <~0.35 Nm,
    calf <~0.15 Nm. Stance legs must NOT get this — the QP force path
    already balances the whole body's weight there."""
    hip_sign, lat_sign, _ = _leg_signs(leg)
    q1, q2, q3 = joints
    h = hip_sign * q1
    ch, sh = math.cos(h), math.sin(h)
    s2 = math.sin(q2)
    s23 = math.sin(q2 + q3)
    z_th = -_D_THIGH_COM * math.cos(q2)
    z_cf = -L2 * math.cos(q2) - _D_CALF_COM * math.cos(q2 + q3)
    y_h = lat_sign * _Y_HIP_COM
    y_th = lat_sign * _Y_THIGH_COM
    y_cf = lat_sign * D_LAT
    tau1 = _GRAV * hip_sign * (
        _M_HIP * y_h * ch
        + _M_THIGH * (y_th * ch - z_th * sh)
        + _M_CALF * (y_cf * ch - z_cf * sh)
    )
    tau2 = _GRAV * ch * (
        _M_THIGH * _D_THIGH_COM * s2 + _M_CALF * (L2 * s2 + _D_CALF_COM * s23)
    )
    tau3 = _GRAV * ch * _M_CALF * _D_CALF_COM * s23
    return tau1, tau2, tau3


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


def quat_rotate_inverse(
    qx: float, qy: float, qz: float, qw: float,
    vx: float, vy: float, vz: float,
) -> np.ndarray:
    """Rotate a world-frame vector into the body frame.

    The quaternion is the body orientation in the world frame (body->world), so
    this applies R.T @ v_world, matching legged_gym's quat_rotate_inverse used to
    form base_lin_vel / projected_gravity at training time.
    """
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),        1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])
    return R.T @ np.array([vx, vy, vz])


def projected_gravity_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return quat_rotate_inverse(qx, qy, qz, qw, 0.0, 0.0, -1.0)


def yaw_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy, -sy, 0.0],
        [sy,  cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
