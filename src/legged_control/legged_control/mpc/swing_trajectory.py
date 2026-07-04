"""swing_trajectory — foot trajectory for swing phase.

Given swing progress s ∈ [0,1]:
  XY: linear interpolation from lift-off to land target
  Z:  sinusoidal arc (0 → step_height at s=0.5 → 0)

All positions are in the hip frame of the respective leg.
"""

from __future__ import annotations
import math
import numpy as np


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
    The foot is directly below the hip at -stance_height in Z.
    Hip lateral and fore-aft offsets are zero in hip frame by definition.
    """
    return np.array([0.0, 0.0, -stance_height])


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
    stance_time = gait_period * (1.0 - swing_ratio)
    # offset in body-frame XY, half of stance duration × velocity
    offset_x = float(body_vel[0]) * stance_time * 0.5
    offset_y = float(body_vel[1]) * stance_time * 0.5
    # clamp to avoid overreach
    offset_x = float(np.clip(offset_x, -0.12, 0.12))
    offset_y = float(np.clip(offset_y, -0.06, 0.06))
    return np.array([offset_x, offset_y, -stance_height])
