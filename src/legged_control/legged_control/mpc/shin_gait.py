"""shin_gait — joint-space crawl for walking on the SHINS (断桥/gap bridge).

A point foot falls straight through a gap in a plank bridge; the shin is an
18 cm beam. Kneeling — knee under the hip, calf lying along the ground,
foot forward — turns each contact from a point into a line that spans gaps
up to ~12–15 cm, so a broken bridge walks like flat ground.

Everything here is JOINT SPACE, not foot-target IK:
  * With the calf laid down the leg is effectively a single link (hip→knee,
    L2), which has too few DOF for 3-D foot-target control — commanding
    joints directly is the honest parameterization, and it can never miss
    an IK solution mid-gait.
  * Convention (from kinematics.forward_kinematics, identical planar
    formulas for all four legs): knee_x = −L2·sin(q2); the shin lies flat
    when q2 + q3 = −π/2. A positive SHIN_PITCH tilts the shin foot-end-down
    (q3 = −π/2 − q2 + pitch): the foot bears the load and the KNEE rides
    L3·sin(pitch) ≈ 9 mm above the deck — that margin is simultaneously the
    swing knee's ground clearance (a rigid L2 link cannot lift the knee any
    higher than the body rides) and the max a shin sinks while bridging a
    gap before its ends catch the planks.
  * Body height is a geometric OUTCOME, L2·cos(q2) + L3·sin(pitch) ≈ 0.189:
    cos is flat around q2 = 0 (knee under hip), so small strides bob the
    body only millimeters — the reason the stance is centered there even
    though it puts the shin line ahead of the hip.

The gait itself reuses the crawl schedule (one leg at a time, three shin
lines always planted — a support polygon far larger than three points).
"""

from __future__ import annotations

import math

import numpy as np

from legged_control.kinematics import L2, L3

# Shin tilt, foot-end-down (rad). Sets knee clearance = gap-sink allowance
# = L3·sin(pitch) ≈ 9 mm. Runtime-tunable through the mpc_node parameter.
SHIN_PITCH = 0.05
# Extra calf fold at mid-swing: lifts the foot ~8.7 cm above the deck while
# the knee arcs low. Keeps q3 ≥ −2.28, well off the −2.65 fold stop.
SHIN_FOLD = 0.5
# Knee stroke half-amplitude clamp (m). asin(0.07/L2) = 0.399 rad of thigh
# keeps every joint inside its limits (calf swing floor ~−2.11, stop −2.65;
# thigh 0.40 vs the −0.5 limit) at a body bob of ~14 mm per stride. Pace
# history: 0.045/6 mm "很稳" → 0.055/9 mm → this (2026-07-16, "还远远不够");
# the actual stride follows the commanded speed, so a lower shin_speed_cap
# shrinks stroke and bob together — this is only the ceiling.
KNEE_OFFSET_MAX = 0.07
# Lateral stroke half-amplitude clamp (m), realized as a hip-roll sweep:
# rolling the leg plane keeps the shin axis parallel to the deck (line
# contact preserved — the shin rolls about its own long axis, it does not
# skid), so strafing works the same way the knee stroke does. Kept small:
# the roll axis sits D_LAT ≈ 0.113 m inboard of the contact line, so a
# hip angle `a` couples ±D_LAT·a of height error into the stroke ends
# (~13 mm here) — the strafe is an alignment nudge, not a cruise axis.
Y_OFFSET_MAX = 0.022


def _smoothstep(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def shin_body_height(pitch: float = SHIN_PITCH) -> float:
    """Hip height above the deck in nominal shin stance (q2 = 0)."""
    return L2 + L3 * math.sin(pitch)


def shin_pose_joints(pitch: float = SHIN_PITCH) -> tuple[float, float, float]:
    """Nominal standing shin pose: knee under hip, shin laid foot-forward.
    URDF q1 = 0 is physical zero abduction for every leg (hip_sign only
    flips the axis direction), so one tuple serves all four legs."""
    return 0.0, 0.0, -math.pi / 2.0 + pitch


def _q2_of_knee_x(knee_x: float) -> float:
    return math.asin(float(np.clip(-knee_x / L2, -0.999, 0.999)))


def _q1_of_dy(dy: float, hip_sign: float, pitch: float) -> float:
    """Hip joint angle that places the shin line `dy` to the leg's +y (body
    left) of nominal. Foot y ≈ y_plane + h·sin(hip_angle) with the leg-plane
    roll `hip_angle` = hip_sign·q1 (kinematics convention), so the world
    direction of q1 flips between left and right legs — hip_sign undoes it."""
    h = shin_body_height(pitch)
    a = math.asin(float(np.clip(dy / h, -0.999, 0.999)))
    return hip_sign * a


def shin_stance_joints(
    s: float,
    knee_offset: float,
    pitch: float = SHIN_PITCH,
    y_offset: float = 0.0,
    hip_sign: float = 1.0,
) -> tuple[float, float, float]:
    """Stance at progress s ∈ [0,1]: knee sweeps +offset → −offset (same
    convention as stance_foot_position — touchdown ahead, lift-off behind),
    calf slaved so the shin keeps its ground tilt throughout. y_offset is
    the matching lateral half-stroke (hip-roll sweep, +y → −y); the roll
    keeps the shin axis parallel to the deck, so the contact line survives."""
    off = float(np.clip(knee_offset, -KNEE_OFFSET_MAX, KNEE_OFFSET_MAX))
    y_off = float(np.clip(y_offset, -Y_OFFSET_MAX, Y_OFFSET_MAX))
    s = float(np.clip(s, 0.0, 1.0))
    knee_x = off * (1.0 - 2.0 * s)
    q1 = _q1_of_dy(y_off * (1.0 - 2.0 * s), hip_sign, pitch)
    q2 = _q2_of_knee_x(knee_x)
    q3 = -math.pi / 2.0 - q2 + pitch
    return q1, q2, q3


def shin_swing_joints(
    s: float,
    knee_offset: float,
    pitch: float = SHIN_PITCH,
    fold: float = SHIN_FOLD,
    y_offset: float = 0.0,
    hip_sign: float = 1.0,
) -> tuple[float, float, float]:
    """Swing at progress s ∈ [0,1]: thigh carries the knee back-to-front
    (smoothstep, zero end velocity), the calf tracks the shin tilt plus a
    raised-cosine extra fold that lifts the FOOT over the deck mid-swing
    and lays the shin back flat exactly at touchdown; the hip rolls back
    −y_offset → +y_offset on the same smoothstep. Endpoints match the
    stance branch bit-for-bit, so phase flips never step the command."""
    off = float(np.clip(knee_offset, -KNEE_OFFSET_MAX, KNEE_OFFSET_MAX))
    y_off = float(np.clip(y_offset, -Y_OFFSET_MAX, Y_OFFSET_MAX))
    s = float(np.clip(s, 0.0, 1.0))
    b = _smoothstep(s)
    q2_lift = _q2_of_knee_x(-off)
    q2_land = _q2_of_knee_x(+off)
    q2 = q2_lift + (q2_land - q2_lift) * b
    q1 = _q1_of_dy(-y_off + 2.0 * y_off * b, hip_sign, pitch)
    bump = 0.5 * (1.0 - math.cos(2.0 * math.pi * s))
    q3 = -math.pi / 2.0 - q2 + pitch - fold * bump
    return q1, q2, q3
