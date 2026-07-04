"""swing_trajectory — foot trajectory for swing phase.

Given swing progress s ∈ [0,1]:
  XY: linear interpolation from lift-off to land target
  Z:  sinusoidal arc (0 → step_height at s=0.5 → 0)

All positions are in the hip frame of the respective leg.
"""

from __future__ import annotations
import math
import numpy as np

from legged_control.kinematics import D_LAT, L_HIP_X, _leg_signs


def swing_foot_position(
    s: float,
    p_lift: np.ndarray,
    p_land: np.ndarray,
    step_height: float = 0.06,
) -> np.ndarray:
    """Foot position at swing progress s ∈ [0,1].

    Args:
        s:           swing progress (0=lift-off, 1=touch-down)
        p_lift:      foot position at lift-off (hip frame, xyz)
        p_land:      desired foot position at touch-down (hip frame, xyz)
        step_height: peak clearance above nominal ground (m)

    Returns:
        3-element array (x, y, z) in hip frame
    """
    s = float(np.clip(s, 0.0, 1.0))
    xy = (1.0 - s) * p_lift[:2] + s * p_land[:2]
    # sinusoidal arc: 0 at endpoints, peak at s=0.5
    z_arc = step_height * math.sin(math.pi * s)
    # ground level is the lower of lift/land z
    z_ground = min(float(p_lift[2]), float(p_land[2]))
    z = z_ground + z_arc
    return np.array([xy[0], xy[1], z])


def nominal_foot_position(leg: str, stance_height: float = 0.27) -> np.ndarray:
    """Default foot position in hip frame at nominal stance height.

    stance_height: desired CoM height above ground (m).
    inverse_kinematics/forward_kinematics define the hip frame with the leg's
    mechanical lateral offset (D_LAT) and fore-aft offset (L_HIP_X) baked in —
    (0, 0, -stance_height) is NOT "foot straight down"; it forces the hip
    joint to solve for an artificial ~20 deg rotation to hit y=0. Include the
    per-leg offsets so IK lands near the natural q1≈0 stance angle.
    """
    _, lat_sign, x_sign = _leg_signs(leg)
    return np.array([x_sign * L_HIP_X, lat_sign * D_LAT, -stance_height])


def landing_target(
    leg: str,
    body_vel: np.ndarray,
    gait_period: float,
    swing_ratio: float,
    stance_height: float = 0.27,
) -> np.ndarray:
    """Raibert heuristic: place foot ahead of nominal by velocity × stance_time / 2.

    Args:
        leg:          leg name (unused here, kept for symmetry)
        body_vel:     body velocity in body frame [vx, vy] (m/s)
        gait_period:  full gait cycle period (s)
        swing_ratio:  fraction of cycle in swing
        stance_height: nominal CoM height (m)

    Returns:
        desired foot position in hip frame (xyz)
    """
    _, lat_sign, x_sign = _leg_signs(leg)
    stance_time = gait_period * (1.0 - swing_ratio)
    # offset in body-frame XY, half of stance duration × velocity
    offset_x = float(body_vel[0]) * stance_time * 0.5
    offset_y = float(body_vel[1]) * stance_time * 0.5
    # clamp to avoid overreach
    offset_x = float(np.clip(offset_x, -0.12, 0.12))
    offset_y = float(np.clip(offset_y, -0.06, 0.06))
    # Raibert offset is added on top of the leg's natural stance position, not
    # the hip-frame origin — see nominal_foot_position for why (0,0) is wrong.
    return np.array([
        x_sign * L_HIP_X + offset_x,
        lat_sign * D_LAT + offset_y,
        -stance_height,
    ])
