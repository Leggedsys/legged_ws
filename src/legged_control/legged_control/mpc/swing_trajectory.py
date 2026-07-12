"""swing_trajectory — foot trajectory for swing phase.

Given swing progress s ∈ [0,1]:
  XY: quintic Hermite from lift-off to land target (endpoint velocity from
      xy_end_slope, endpoint acceleration zero), or plain lerp without slopes
  Z:  minimum-jerk quintic bump (0 → step_height at s=0.5 → 0, zero endpoint
      velocity AND acceleration)

Everything is C2: the stance stroke is a constant-velocity sweep
(acceleration zero), so zero endpoint acceleration here means no
acceleration step — hence no commanded-torque impulse through the PD — at
lift-off or touchdown. The flat_top > 0 trapezoid (hurdle/stair) keeps its
original cubic-smoothstep edges: its peak vertical speed is
hardware-validated (see test_hurdle_vertical_speed_in_validated_band) and
quintic edges are ~25 % faster at peak for the same rise window.

All positions are in the hip frame of the respective leg.
"""

from __future__ import annotations
import math
import numpy as np

from legged_control.kinematics import D_LAT, L_HIP_X, _leg_signs

# Hip mount position relative to body center (from dog_urdf hip joint origins)
_HIP_MOUNT_X = 0.1426   # m — fore-aft
_HIP_MOUNT_Y = 0.0465   # m — lateral


def leg_velocity(body_vel_xy: np.ndarray, yaw_rate: float, leg: str) -> np.ndarray:
    """Per-leg equivalent ground velocity at the leg's nominal contact point.

    v_leg = v_body + ω × r, with r the nominal foot contact position in the
    body frame (hip mount + in-leg nominal offsets). Feeding this to the
    Raibert offset / stance stroke gives each side a different stroke length
    under a yaw command, which is what turns the body in position control.
    """
    _, lat_sign, x_sign = _leg_signs(leg)
    rx = x_sign * (_HIP_MOUNT_X + L_HIP_X)
    ry = lat_sign * (_HIP_MOUNT_Y + D_LAT)
    return np.array([
        float(body_vel_xy[0]) - yaw_rate * ry,
        float(body_vel_xy[1]) + yaw_rate * rx,
    ])


def _s5(u: float) -> float:
    """Quintic (minimum-jerk) smoothstep 6u⁵−15u⁴+10u³: rises 0→1 with zero
    first AND second derivative at both ends."""
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0)


def _z_profile(s: float, flat_top: float) -> float:
    """Vertical clearance profile in [0,1] over swing progress s.

    flat_top = 0: minimum-jerk quintic bump — full height at the single
    instant s=0.5, zero slope and zero curvature at lift-off/touchdown, so
    the vertical acceleration is continuous with the constant-height stance
    (no accel step at contact flips; the raised cosine it replaced stepped
    z̈ by 2π²·H/T² there). flat_top > 0 turns it into a cubic-smoothstep
    trapezoid that HOLDS full height over the centered flat_top fraction of
    the swing — for stepping over an obstacle whose position along the
    stride the operator cannot place precisely (hurdle mode: a thin 150 mm
    board is cleared anywhere inside the flat window instead of only at
    exact mid-swing). The trapezoid keeps cubic edges deliberately — see the
    module docstring (validated peak-speed band).
    """
    if flat_top <= 0.0:
        return _s5(2.0 * s) if s < 0.5 else _s5(2.0 * (1.0 - s))
    r = max((1.0 - float(flat_top)) / 2.0, 1e-6)
    if s < r:
        u = s / r
    elif s > 1.0 - r:
        u = (1.0 - s) / r
    else:
        return 1.0
    return u * u * (3.0 - 2.0 * u)


def swing_foot_position(
    s: float,
    p_lift: np.ndarray,
    p_land: np.ndarray,
    step_height: float = 0.06,
    xy_end_slope: np.ndarray | None = None,
    flat_top: float = 0.0,
) -> np.ndarray:
    """Foot position at swing progress s ∈ [0,1].

    Args:
        s:           swing progress (0=lift-off, 1=touch-down)
        p_lift:      foot position at lift-off (hip frame, xyz)
        p_land:      desired foot position at touch-down (hip frame, xyz)
        step_height: peak clearance above nominal ground (m)
        xy_end_slope: optional d(xy)/ds at both endpoints (hip frame, per unit
                     s). Pass −v_leg·T_swing so the foot's ground-relative
                     velocity is zero at lift-off and touchdown: if the leg is
                     still loaded there (body sag), it pushes the same way the
                     stance legs do instead of scuffing the body backward.
                     None keeps the plain linear interpolation.
        flat_top:    fraction of the swing held at full step_height (see
                     _z_profile); 0 keeps the plain arc.

    Returns:
        3-element array (x, y, z) in hip frame
    """
    s = float(np.clip(s, 0.0, 1.0))
    if xy_end_slope is None:
        xy = (1.0 - s) * p_lift[:2] + s * p_land[:2]
    else:
        # quintic Hermite with equal endpoint slopes m and zero endpoint
        # acceleration: endpoints exact, d(xy)/ds = m and d²(xy)/ds² = 0 at
        # s=0 and s=1 — matches the constant-velocity stance stroke through
        # position, velocity AND acceleration.
        m = np.asarray(xy_end_slope, dtype=float)
        s3, s4, s5 = s**3, s**4, s**5
        h01 = 10 * s3 - 15 * s4 + 6 * s5
        h1x = s - 10 * s3 + 15 * s4 - 6 * s5  # h10 + h11 (equal slopes)
        xy = (1.0 - h01) * p_lift[:2] + h01 * p_land[:2] + h1x * m
    # quintic bump (or smoothstep trapezoid when flat_top > 0): 0 at the
    # endpoints with zero vertical velocity, so touchdown is soft instead of
    # descending at peak speed straight into the ground target.
    z_arc = step_height * _z_profile(s, flat_top)
    # z baseline blends between the endpoint heights (not min of them): when
    # the stance height changes mid-swing (/height_command), the lift and
    # land z differ, and pinning the baseline to one of them makes touchdown
    # snap by the height change. The quintic smoothstep spreads it across the
    # swing with zero endpoint velocity/acceleration — the plain lerp it
    # replaced entered the swing with a dz/ds step whenever the heights
    # differed.
    z_base = float(p_lift[2]) + (float(p_land[2]) - float(p_lift[2])) * _s5(s)
    z = z_base + z_arc
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


def _raibert_offset(
    body_vel: np.ndarray, gait_period: float, swing_ratio: float
) -> tuple[float, float]:
    """Raibert heuristic stride half-offset: velocity × stance_time / 2, clamped."""
    stance_time = gait_period * (1.0 - swing_ratio)
    offset_x = float(np.clip(float(body_vel[0]) * stance_time * 0.5, -0.12, 0.12))
    offset_y = float(np.clip(float(body_vel[1]) * stance_time * 0.5, -0.06, 0.06))
    return offset_x, offset_y


def landing_target(
    leg: str,
    body_vel: np.ndarray,
    gait_period: float,
    swing_ratio: float,
    stance_height: float = 0.27,
) -> np.ndarray:
    """Raibert heuristic: place foot ahead of nominal by velocity × stance_time / 2.

    Args:
        leg:          leg name — selects the nominal-stance hip-frame offsets
        body_vel:     body velocity in body frame [vx, vy] (m/s)
        gait_period:  full gait cycle period (s)
        swing_ratio:  fraction of cycle in swing
        stance_height: nominal CoM height (m)

    Returns:
        desired foot position in hip frame (xyz)
    """
    _, lat_sign, x_sign = _leg_signs(leg)
    offset_x, offset_y = _raibert_offset(body_vel, gait_period, swing_ratio)
    # Raibert offset is added on top of the leg's natural stance position, not
    # the hip-frame origin — see nominal_foot_position for why (0,0) is wrong.
    return np.array([
        x_sign * L_HIP_X + offset_x,
        lat_sign * D_LAT + offset_y,
        -stance_height,
    ])


def stance_foot_position(
    leg: str,
    s: float,
    body_vel: np.ndarray,
    gait_period: float,
    swing_ratio: float,
    stance_height: float = 0.27,
) -> np.ndarray:
    """Stance-phase foot target: stroke backward from +offset to −offset.

    Pure position control has no force path pushing the body forward, so the
    stance feet must sweep backward in the hip frame to propel it. Over one
    stance the body should advance v·T_stance, i.e. the foot travels back
    2 × Raibert offset. Endpoints make phase transitions continuous: s=0
    coincides with landing_target (touchdown), s=1 is the lift-off point the
    next swing starts from.

    Args:
        leg:          leg name — selects the nominal-stance hip-frame offsets
        s:            stance progress ∈ [0,1] (0=touchdown, 1=lift-off)
        body_vel:     body velocity in body frame [vx, vy] (m/s)
        gait_period:  full gait cycle period (s)
        swing_ratio:  fraction of cycle in swing
        stance_height: nominal CoM height (m)

    Returns:
        desired foot position in hip frame (xyz)
    """
    _, lat_sign, x_sign = _leg_signs(leg)
    offset_x, offset_y = _raibert_offset(body_vel, gait_period, swing_ratio)
    k = 1.0 - 2.0 * float(np.clip(s, 0.0, 1.0))
    return np.array([
        x_sign * L_HIP_X + offset_x * k,
        lat_sign * D_LAT + offset_y * k,
        -stance_height,
    ])
