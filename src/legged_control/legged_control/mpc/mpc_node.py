"""mpc_node — hybrid trot gait controller: position gait + SRBD-MPC tau_ff.

Replaces policy_node for non-RL trot gait. Output is /joint_commands
(URDF frame, same as policy_node), so all downstream hardware nodes are
unchanged.

FSM:
  PASSIVE → (posture_command=true) → STANDUP → WALK → PASSIVE
  B-button (posture_command=false) → LIEDOWN → PASSIVE from any state.

/joint_commands published at mpc.tick_hz (default 100 Hz). Deliberately a
separate key from control.gait_hz (policy_node's RL loop rate) — the two
control stacks have different bandwidth needs and must not move together.

Structure: joint position targets come from gait-scheduled IK (stance
stroke + swing spline + IMU attitude leveling) with one fixed kp/kd scale
for the whole cycle — no stance/swing gain switching. On top of that,
SRBD-MPC solves ground reaction forces and J^T·f torque feedforward is
added, smoothed at every transition (global blend, per-leg stance load
ramp, output low-pass) so the commanded torque can never step. Runtime
toggle: `ros2 param set /mpc_node tau_ff_enabled false`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
import time

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import Twist

from legged_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    leg_kinematic_velocity,
    _leg_signs,
    _numerical_jacobian,
    _smoothstep,
)
from legged_control.mpc.gait_scheduler import GaitScheduler, LEG_NAMES
from legged_control.mpc.srbd_mpc import SRBDMPC, _euler_to_R
from legged_control.mpc.terrain_estimator import TerrainEstimator, dz_on_plane
from legged_control.mpc.swing_trajectory import (
    swing_foot_position,
    stance_foot_position,
    nominal_foot_position,
    landing_target,
    leg_velocity,
    _HIP_MOUNT_X,
    _HIP_MOUNT_Y,
)


def _lateral_spread(stance_h: float, start: float, s_max: float) -> float:
    """Low-posture lateral stance spread (m): 0 at/above `start`, growing
    1:1 as the body drops below it, capped at `s_max`.

    Crouching with the feet at the nominal lateral offset folds the leg
    hard — the knee drops toward the ground (h=0.16: clearance 8 cm and
    the calf sits at −2.22 rad, 0.4 from its −2.65 limit; mid-swing folds
    eat most of what's left → knees knock the floor). Abducting the hips
    and planting the feet wider keeps the same body height with less fold:
    +6 cm spread at h=0.16 buys ~2 cm knee clearance and moves the calf
    to −1.92 rad. Capped at 0.06 m because the hips saturate at ±0.4 rad
    (q1 = 0.32 at h=0.16, s=0.06 — leaves margin for attitude offsets).
    Zero at normal heights: the validated baseline is untouched.
    """
    return float(np.clip(start - stance_h, 0.0, s_max))


def _hip_mount_xy(leg: str) -> tuple[float, float]:
    """Hip-frame → body-frame xy offset for a leg. forward_kinematics and
    the foot targets live in per-leg HIP frames whose origins sit at the
    hip mounts; anything comparing feet ACROSS legs (the terrain plane
    fit and its dz evaluation) needs the shared body frame, or the
    fore-aft lever arm shrinks ~3× (0.207 m → 0.065 m)."""
    _, lat_sign, x_sign = _leg_signs(leg)
    return x_sign * _HIP_MOUNT_X, lat_sign * _HIP_MOUNT_Y


# YAML joint order (robot.yaml / /joint_commands convention)
_YAML_JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

# Default stance angles (URDF frame) — same as policy.yaml joint_default_q_urdf
_DEFAULT_Q = {
    "FR_hip": 0.1,  "FR_thigh": 0.8,  "FR_calf": -1.5,
    "FL_hip": 0.1,  "FL_thigh": 0.8,  "FL_calf": -1.5,
    "RR_hip": 0.1,  "RR_thigh": 1.0,  "RR_calf": -1.5,
    "RL_hip": 0.1,  "RL_thigh": 1.0,  "RL_calf": -1.5,
}

_PHASE_PASSIVE  = "PASSIVE"
_PHASE_STANDUP  = "STANDUP"
_PHASE_WALK     = "WALK"
_PHASE_LIEDOWN  = "LIEDOWN"

_STANDUP_TOL  = 0.15   # rad — standup convergence threshold
_VEL_SETTLED  = 0.05   # rad/s — velocity threshold
_LIEDOWN_TIMEOUT = 3.0
_WALK_VEL_THRESH = 0.04  # m/s or rad/s — below this in all axes → hold stance
_EST_TIMEOUT = 0.5       # s — state_estimate older than this → fall back to balance stance
_VEL_FILTER_TAU = 0.25   # s — first-order lag on cmd_vel for trajectory stride.
                         # Stride grows/shrinks smoothly instead of jumping when
                         # the stick moves; walking only stops once the filtered
                         # stride has decayed to ~0, so start/stop never snaps.
_HEIGHT_SLEW = 0.05      # m/s — max stance-height change rate (LT/RT via /height_command)
_HEIGHT_MIN  = 0.15      # m — leg reach / collision guard on commanded height
_HEIGHT_MAX  = 0.30      # m — L2+L3 = 0.361 m; 0.30 leaves stroke + swing margin
_STEP_VEL_REF = 0.15     # m/s — leg speed at which swing clearance reaches full
                         # step_height. Below it clearance shrinks proportionally,
                         # so the shrinking-stride steps before a stop stay near
                         # the ground instead of lifting the full 6 cm.
_ATT_DZ_MAX = 0.04       # m — clamp on per-foot attitude-leveling z correction
_ATT_TILT_TAU = 0.10     # s — low-pass on the tilt (gravity) signal: keeps
                         # touchdown-impact noise out of the foot targets.
_ATT_DRV_TAU = 0.05      # s — light smoothing on the tilt derivative. Damping
                         # is the numerical derivative of the same filtered
                         # tilt signal used for P — not the raw gyro, whose
                         # axis convention depends on IMU mounting and, if
                         # guessed wrong, turns the damping into fast positive
                         # feedback (observed as violent hopping on tiny tilts).
_ATT_DZ_SLEW = 0.25      # m/s — hard rate limit on the leveling correction:
                         # whatever the tuning does, foot targets cannot jump.
_ATT_I_MAX = 0.03        # m — clamp on the integral trim (windup guard). The
                         # proportional loop only rejects gain/(1+gain) ≈ 1/3 of
                         # a static tilt; the integrator trims the remainder.
_ATT_I_LEAK_TAU = 1.0    # s — integral decays to zero when the estimate is
                         # stale or leveling is disabled, same as the P path.
_TERRAIN_BLEND_TAU = 0.3 # s — enable/disable ramp on the terrain adaptation
                         # outputs (foot dz + attitude reference), so a runtime
                         # toggle can never step the foot targets or the QP ref.
# tau_ff smoothing — three layers so the feedforward torque can never step
# (the phase-transition snaps that plagued the original force controller):
_TAU_BLEND_TAU = 0.3     # s — global ramp on WALK entry / runtime enable-disable
_TAU_RAMP_FRAC = 0.2     # per-leg force ramps over the first/last 20% of stance,
                         # so a foot carries no commanded force at the instants
                         # of touchdown and lift-off
_TAU_LP_TAU = 0.04       # s — low-pass on the final torque vector: absorbs the
                         # MPC force-redistribution step the *other* legs see
                         # when a contact flips. Feedforward lag only; position
                         # targets are untouched and PD covers the transient.
_RATE_LP_TAU = 0.03      # s — light low-pass on the gyro before it enters the
                         # QP state. Short on purpose: this channel exists to
                         # damp 2 Hz body sway, so lag here eats the phase
                         # margin the whole feature is meant to buy.
_Z_ERR_CLIP = 0.05       # m — cap on the height error fed to the QP: a wrong
                         # contact set or an IK-failed leg can fake a large
                         # sag, and z weight 200 would turn it into a launch.
_VZ_CLIP = 0.4           # m/s — same guard on the leg-odometry vertical rate.
_SCHEDULE_SUBSAMPLES = 5 # per-bucket contact subsamples → fractional contact
                         # shares. Quantizes flip times at mpc_dt/5 = 5 ms
                         # instead of a full 25 ms bucket; keeps the per-tick
                         # QP solution change under the torque-step budget.

# Leg ordering used for stance IK loops: FR=0, FL=1, RR=2, RL=3
_MPC_LEG_ORDER = ["FR", "FL", "RR", "RL"]


@dataclass
class JointCommand:
    """Per-tick joint command output from the gait controller."""
    q:   list[float]
    dq:  list[float]
    tau: list[float]
    kp:  list[float]
    kd:  list[float]


def _leg_joints(leg: str) -> list[str]:
    return [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]


def _stance_load_ramp(s: float, frac: float = _TAU_RAMP_FRAC) -> float:
    """Per-leg force scale over stance progress s ∈ [0,1]: 0 at touchdown,
    1 through mid-stance, 0 at lift-off (smoothstep ramps over `frac`)."""
    s = float(np.clip(s, 0.0, 1.0))
    frac = max(1e-3, float(frac))
    return _smoothstep(s / frac) * _smoothstep((1.0 - s) / frac)


def _remove_mount_bias(g: np.ndarray, roll_off: float, pitch_off: float) -> np.ndarray:
    """Remove the IMU mounting bias from a projected-gravity vector.

    Every attitude consumer (position-side leveling trim AND the MPC attitude
    P) drives the tilt it sees to zero — so a mounting bias makes both loops
    actively HOLD the body at the biased attitude, and enabling tau_ff leans
    the body harder into it (more authority toward the same wrong zero).
    Converting to roll/pitch, subtracting the calibrated offsets and
    rebuilding the vector makes "zero tilt" mean geometrically level.
    Calibration: standing with tau_ff on, nudge att_pitch_offset (negative =
    nose-up correction) until height_check shows equal front/rear measured
    heights; same for att_roll_offset left/right."""
    if roll_off == 0.0 and pitch_off == 0.0:
        return g
    roll  = float(np.arctan2(-g[1], -g[2])) - roll_off
    pitch = float(np.arctan2(g[0], float(np.hypot(g[1], g[2])))) - pitch_off
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    return np.array([sp, -sr * cp, -cr * cp]) * float(np.linalg.norm(g))


def _stance_gain_scale(base: float, stance: float, w: float) -> float:
    """Crossfade a gain scale from its swing value (w=0) to its stance value
    (w=1). stance <= 0 disables the split — the gain follows base everywhere."""
    if stance <= 0.0:
        return base
    return base + (stance - base) * float(np.clip(w, 0.0, 1.0))


def _state_from_estimate(est: np.ndarray, pos: np.ndarray) -> np.ndarray:
    """Build 12-dim SRBD state from /state_estimate and an assumed CoM position.

    /state_estimate layout: [0:3] base_lin_vel, [3:6] base_ang_vel,
    [6:9] projected_gravity, [9] health. Roll/pitch from gravity direction,
    yaw unobservable from IMU alone → 0.

    projected_gravity = R^T·[0,0,−1] (legged_gym convention), so for ZYX
    Euler angles g_body = [sin p, −sin r·cos p, −cos r·cos p] and the
    inversions are roll = atan2(−gy, −gz), pitch = atan2(gx, √(gy²+gz²)).
    The previous atan2(gy,−gz)/atan2(−gx,−gz) NEGATED both angles — the MPC
    saw a mirrored tilt and, roll/pitch being its heaviest-weighted states,
    pushed the body harder INTO the lean: positive feedback through the
    force path (hardware: growing tilt while walking, violent non-converging
    sway in balance stance fighting the position-side leveling).
    """
    ang_vel = est[3:6]
    proj_g  = est[6:9]
    roll  = float(np.arctan2(-proj_g[1], -proj_g[2]))
    pitch = float(np.arctan2(proj_g[0], float(np.hypot(proj_g[1], proj_g[2]))))
    lin_vel = est[0:3]
    return np.array([
        roll, pitch, 0.0,
        pos[0], pos[1], pos[2],
        ang_vel[0], ang_vel[1], ang_vel[2],
        lin_vel[0], lin_vel[1], lin_vel[2],
    ], dtype=float)


def _measured_body_z(
    joint_pos: dict[str, float],
    joint_vel: dict[str, float],
    weights: dict[str, float],
    R_body: np.ndarray,
) -> tuple[float, float] | None:
    """Body height + vertical velocity from stance-leg FK (leg odometry, z only).

    A planted foot is pinned to the ground, so the body sits −(R·p_foot)_z
    above it and moves at (R·(−J·q̇))_z. Averaged over legs weighted by their
    stance load ramp: a foot at touchdown/lift-off carries ~zero weight in the
    estimate exactly when its contact is least trustworthy. Returns None when
    no leg carries load (full flight — never happens in trot)."""
    num_h = num_v = den = 0.0
    for leg in _MPC_LEG_ORDER:
        w = float(weights.get(leg, 0.0))
        if w <= 1e-6:
            continue
        q = tuple(joint_pos[j] for j in _leg_joints(leg))
        dq = tuple(joint_vel[j] for j in _leg_joints(leg))
        p_w = R_body @ np.asarray(forward_kinematics(leg, q), dtype=float)
        v_w = R_body @ leg_kinematic_velocity(leg, q, dq)
        num_h += w * -p_w[2]
        num_v += w * v_w[2]
        den += w
    if den <= 1e-6:
        return None
    return num_h / den, num_v / den


def _project_vertical_grf(grf: np.ndarray, foot_pos: np.ndarray) -> np.ndarray:
    """Project the QP GRF onto vertical-only forces, preserving total weight
    support and the roll/pitch leveling moments.

    With a CoM offset the feet sit asymmetrically about the CoM, and the QP
    discovers it can balance the pitch moment with a NET HORIZONTAL push
    (moment arm = stance height) instead of a front/rear fz split — x position
    has zero cost weight and the velocity channels are neutralised, so the
    push looks free. On hardware it shoves the body sideways/forward until
    the PD leg stiffness catches it: body creeps forward as tau_ff blends in,
    walking toward the tip-over boundary (com_x 0.05 → ~25 N forward).

    Keep what the QP is trusted for — Σfz (weight) and Mx/My (leveling) —
    re-solved as the min-change vertical distribution over the stance legs;
    horizontal dynamics belong to the position-control PD. Yaw moment is
    dropped (yaw is unobservable and its reference is zero).
    """
    f = grf.reshape(4, 3).copy()
    stance = f[:, 2] > 1e-9
    if not np.any(stance):
        return grf
    r = foot_pos[stance]
    fz0 = f[stance, 2]
    M = np.cross(foot_pos, f).sum(axis=0)
    # rows: Σfz = weight, Σ r_y·fz = Mx, Σ −r_x·fz = My
    A = np.stack([np.ones(len(fz0)), r[:, 1], -r[:, 0]])
    b = np.array([float(f[:, 2].sum()), float(M[0]), float(M[1])])
    # min ‖fz − fz0‖² s.t. A·fz = b — least-squares via pseudo-inverse so a
    # two-leg stance (rank-deficient) degrades gracefully instead of raising.
    lam = np.linalg.pinv(A @ A.T, rcond=1e-8) @ (b - A @ fz0)
    fz = np.clip(fz0 + A.T @ lam, 0.0, None)
    # Weight support is a hard constraint: on a rank-deficient (two-leg)
    # stance the least-squares splits the residual between weight and
    # moments — rescale so Σfz is exact and let the leveling loops absorb
    # the (second-order) moment remainder.
    if fz.sum() > 1e-9:
        fz *= b[0] / fz.sum()
    out = np.zeros_like(f)
    out[stance, 2] = fz
    return out.reshape(-1)


def _apply_load_ramp(
    grf: np.ndarray,
    leg_scale: dict[str, float],
    mu: float = 0.6,
) -> np.ndarray:
    """Scale each leg's GRF by its stance load ramp and hand the removed share
    to the still-loaded legs.

    The per-leg ramp alone commands less than the QP's total force through
    every touchdown/lift-off window (~60 ms each flip), and the PD absorbs
    the deficit — measured on hardware as a 1-2 cm body dip at every contact
    flip (walking bob/sway that more kp does not fix, only stiffens). Real
    weight transfer is a hand-off, not a fade-out: redistribute the ramped-off
    force to the other stance legs in proportion to their own ramp, keeping
    the commanded total constant. Receiving legs are re-clipped to the
    friction cone.
    """
    f = grf.reshape(4, 3)
    s = np.clip(
        [float(leg_scale.get(leg, 0.0)) for leg in _MPC_LEG_ORDER], 0.0, 1.0
    )
    f_scaled = f * s[:, None]
    deficit = (f - f_scaled).sum(axis=0)
    w = s * (f[:, 2] > 1e-9)
    if w.sum() > 1e-6:
        f_scaled = f_scaled + np.outer(w / w.sum(), deficit)
        for i in range(4):
            fz = f_scaled[i, 2]
            if fz <= 0.0:
                f_scaled[i] = 0.0
                continue
            f_xy_max = mu * fz
            f_scaled[i, 0] = float(np.clip(f_scaled[i, 0], -f_xy_max, f_xy_max))
            f_scaled[i, 1] = float(np.clip(f_scaled[i, 1], -f_xy_max, f_xy_max))
    return f_scaled.reshape(-1)


def _build_stance_tau(
    grf: np.ndarray,
    joint_targets: dict[str, float],
    leg_scale: dict[str, float],
    R_body: np.ndarray,
) -> list[float]:
    """Convert MPC GRF to joint torques via Jacobian transpose, per-leg scaled.

    τ[leg] = −leg_scale · J(q)^T · R^T · f_leg. The MPC GRF is ground-on-body
    in the world frame (z up, fz ∈ [f_min, f_max]); R^T maps it to the body
    frame the Jacobian lives in, and the minus sign is the statics of the
    actuator RESISTING that external force (τ + J^T·f = 0). Without it the
    feedforward pushes the body down with ~mg instead of carrying it —
    observed on hardware as the stance height dropping when tau_ff is
    enabled. leg_scale carries the stance load ramp (0 at touchdown and
    lift-off), so commanded force never steps at a contact transition.
    """
    tau_dict: dict[str, float] = {n: 0.0 for n in _YAML_JOINTS}
    for i, leg in enumerate(_MPC_LEG_ORDER):
        scale = float(leg_scale.get(leg, 0.0))
        if scale <= 0.0:
            continue
        f_leg = R_body.T @ grf[i * 3 : i * 3 + 3]
        joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
        J = _numerical_jacobian(leg, joints_leg)
        tau_leg = -(J.T @ f_leg) * scale
        for jname, t in zip(_leg_joints(leg), tau_leg):
            tau_dict[jname] = float(t)
    return [tau_dict[n] for n in _YAML_JOINTS]


def _leveling_dz(u_x: float, u_y: float, dz_max: float = _ATT_DZ_MAX) -> dict[str, float]:
    """Map body-tilt control effort to per-foot z offsets (hip frame).

    u_x/u_y are the leveling PD outputs for the fore-aft / lateral tilt axes
    (positive = that body axis tilted down, in projected-gravity terms). The
    tilted-down side gets a more negative (longer) foot target, which pushes
    that side of the body back up.
    """
    out: dict[str, float] = {}
    for leg in LEG_NAMES:
        _, lat_sign, x_sign = _leg_signs(leg)
        dz = -(u_x * x_sign + u_y * lat_sign)
        out[leg] = float(np.clip(dz, -dz_max, dz_max))
    return out


class MPCNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_node")

        cfg = self._load_config()
        control = cfg["control"]
        standup_cfg = cfg.get("standup", {})

        mpc_cfg = cfg.get("mpc", {})
        tick_hz = float(mpc_cfg.get("tick_hz", 100.0))
        self._dt = 1.0 / tick_hz

        self.declare_parameter("gait_period",     float(mpc_cfg.get("gait_period",   0.6)))
        self.declare_parameter("swing_ratio",      float(mpc_cfg.get("swing_ratio",   0.4)))
        self.declare_parameter("step_height",      float(mpc_cfg.get("step_height",   0.06)))
        self.declare_parameter("stance_height",    float(mpc_cfg.get("stance_height", 0.27)))
        self.declare_parameter("ramp_duration",    float(standup_cfg.get("ramp_duration", 6.0)))
        self.declare_parameter("lie_down_duration", float(standup_cfg.get("lie_down_duration", 2.0)))
        # Single fixed kp/kd scale applied for the whole gait cycle (no
        # stance/swing gain switching — see dev/mpc-purepos branch notes).
        # Runtime-tunable: `ros2 param set /mpc_node kp_scale 0.5`
        self.declare_parameter("kp_scale", float(mpc_cfg.get("kp_scale", 0.6)))
        self.declare_parameter("kd_scale", float(mpc_cfg.get("kd_scale", 1.0)))
        # "Soft stance / stiff swing" split (<= 0 disables → kp_scale
        # everywhere). Loaded stance legs crossfade toward these as tau_ff
        # takes their weight (weight = load ramp × tau blend), swing legs — the
        # ones with no feedforward at all — keep the full kp_scale/kd_scale.
        # Reverts to stiff automatically whenever tau_ff is off or ramping.
        self.declare_parameter("kp_stance_scale", float(mpc_cfg.get("kp_stance_scale", -1.0)))
        self.declare_parameter("kd_stance_scale", float(mpc_cfg.get("kd_stance_scale", -1.0)))
        # IMU attitude leveling (WALK phase): tilt from projected_gravity,
        # damping from that signal's own derivative → per-foot z offsets.
        # att_kp in m per unit tilt (≈ m/rad for small angles); 0 disables.
        # Loop gain: dz maps back to commanded counter-tilt as 2·att_kp/track
        # (track ≈ 0.32 m) — att_kp 0.16 is unity DC loop gain, keep well below.
        self.declare_parameter("att_kp", float(mpc_cfg.get("att_kp", 0.08)))
        self.declare_parameter("att_kd", float(mpc_cfg.get("att_kd", 0.02)))
        self.declare_parameter("att_ki", float(mpc_cfg.get("att_ki", 0.10)))
        # IMU mounting bias (rad): what "level" should mean. Both leveling
        # loops zero the corrected tilt, so these are, directly, the attitude
        # the body is held at. See _remove_mount_bias for the calibration.
        self.declare_parameter("att_roll_offset",  float(mpc_cfg.get("att_roll_offset", 0.0)))
        self.declare_parameter("att_pitch_offset", float(mpc_cfg.get("att_pitch_offset", 0.0)))
        # SRBD-MPC GRF feedforward on top of the (unchanged) position gait.
        # Runtime A/B: `ros2 param set /mpc_node tau_ff_enabled false` — the
        # global blend ramps it out over ~0.3 s, never a step.
        self.declare_parameter("tau_ff_enabled", bool(mpc_cfg.get("tau_ff_enabled", True)))
        # Physical calibration, runtime-tunable against scripts/height_check.py
        # while standing with tau_ff on:
        #   mass  — mean Δ (meas−cmd height) nulls when mass is right;
        #   com_x/com_y — per-leg Δ spread nulls when the CoM offset is right
        #   (front legs sagging while rear legs ride above command ⇒ real CoM
        #   is forward of the geometric foot center ⇒ raise com_x).
        self.declare_parameter("mass",  float(mpc_cfg.get("mass", 14.55)))
        self.declare_parameter("com_x", float(mpc_cfg.get("com_x", 0.0)))
        self.declare_parameter("com_y", float(mpc_cfg.get("com_y", 0.0)))
        # Body-rate feedback into the QP state (force-level attitude damping,
        # the MIT fast channel). Assumes the standard gyro axis convention
        # (wx=ang_vel[0], wy=ang_vel[1], right-handed x-forward/z-up); if the
        # body starts a growing oscillation the moment tau blends in, an axis
        # is flipped — `ros2 param set /mpc_node rate_fb_enabled false` reverts
        # instantly. rate_fb_weight is the QP cost on the wx/wy error relative
        # to the roll/pitch weight of 200: damping strength knob.
        self.declare_parameter("rate_fb_enabled", bool(mpc_cfg.get("rate_fb_enabled", True)))
        self.declare_parameter("rate_fb_weight",  float(mpc_cfg.get("rate_fb_weight", 1.0)))
        # Height loop closure: measured body z (+ vertical rate) from
        # stance-leg FK fed into the QP. Sag → z error → extra lift, so total
        # force no longer relies on the mass parameter being exact — set mass
        # back to its calibrated value when this is on; an inflated mass just
        # biases the feedforward and parks the loop against the error clip.
        self.declare_parameter("z_fb_enabled", bool(mpc_cfg.get("z_fb_enabled", True)))
        self.declare_parameter("z_fb_weight",  float(mpc_cfg.get("z_fb_weight", 2000.0)))
        # Hand-off dip knobs (mpc_debug data 2026-07-09: sag is a 4 Hz
        # transient peaking ~90 ms after touchdown — inside the load-ramp
        # window — not a stiffness problem):
        # tau_ramp_frac shrinks the under-supported window itself;
        # vz_fb_weight is the QP cost on measured vertical rate — damping
        # that catches the body early in the fall, when the position error
        # is still too small for z_fb_weight to matter.
        self.declare_parameter("tau_ramp_frac", float(mpc_cfg.get("tau_ramp_frac", _TAU_RAMP_FRAC)))
        self.declare_parameter("vz_fb_weight",  float(mpc_cfg.get("vz_fb_weight", 10.0)))
        # Terrain adaptation (slopes ≤ ~15°, omnidirectional). Two layers,
        # independently runtime-toggleable, each ramped over 0.3 s so a
        # toggle never steps anything. Both are ≈ no-ops on flat ground.
        #   terrain_adapt_enabled  — per-foot z offsets put swing landings
        #     and stance targets on the estimated ground plane (position
        #     level; THE fix for slope-transition early/late touchdown).
        #   terrain_att_ref_enabled — MPC attitude reference follows the
        #     slope instead of level, so the QP stops torquing the body
        #     back to horizontal on an incline (force level, weak gains).
        self.declare_parameter(
            "terrain_adapt_enabled", bool(mpc_cfg.get("terrain_adapt_enabled", True))
        )
        self.declare_parameter(
            "terrain_att_ref_enabled", bool(mpc_cfg.get("terrain_att_ref_enabled", True))
        )
        # Low-posture stance spread (see _lateral_spread): below
        # low_spread_start the feet plant wider, up to low_spread_max.
        self.declare_parameter(
            "low_spread_start", float(mpc_cfg.get("low_spread_start", 0.22))
        )
        self.declare_parameter(
            "low_spread_max", float(mpc_cfg.get("low_spread_max", 0.06))
        )

        mass = float(self.get_parameter("mass").value)
        inertia = np.diag([
            float(mpc_cfg.get("Ixx", 0.0196)),
            float(mpc_cfg.get("Iyy", 0.0228)),
            float(mpc_cfg.get("Izz", 0.0169)),
        ])
        horizon = int(mpc_cfg.get("horizon", 6))
        # MPC internal prediction step, decoupled from the tick period: the
        # QP re-solves every tick regardless, but predicts mpc_dt·horizon
        # ahead. At tick dt (10 ms) a horizon of 6 sees only 60 ms — less
        # than one hand-off — so contact flips arrive unannounced and the
        # load transfer is done entirely by the reactive ramp. 0.025×8 =
        # 200 ms covers a full swing: the QP sees the upcoming touchdown /
        # lift-off inside its window and starts migrating force beforehand.
        # NOTE: effective gains scale with the window — z_fb_weight /
        # vz_fb_weight / rate_fb_weight defaults are calibrated per
        # (mpc_dt, horizon) pair; changing one means re-checking the others
        # (sweep in tests/test_mpc.py::test_long_lookahead_gain_equivalence).
        mpc_dt = float(mpc_cfg.get("mpc_dt", self._dt))
        # grf_slew_weight: Δu continuity cost inside the QP (see srbd_mpc).
        # Calibrated offline via check_tau_continuity: keeps the hand-off
        # pre-load spread over the double-support window instead of slammed
        # into the last ~40 ms before the flip.
        slew_w = float(mpc_cfg.get("grf_slew_weight", 0.0))
        self._mpc = SRBDMPC(mass=mass, inertia_body=inertia, dt=mpc_dt,
                            horizon=horizon, slew_weight=slew_w)
        self._grf_prev: np.ndarray | None = None  # last projected GRF (Δu anchor)
        self._tau_blend = 0.0            # global enable ramp state
        self._tau_lp = [0.0] * 12        # low-passed output torque
        self._rate_lp = np.zeros(3)      # low-passed world-frame body rate
        self._z_lp: np.ndarray | None = None  # low-passed (height, vz) leg odometry

        # Per-joint base kp/kd (motor side, same basis as motor_bus_node)
        _ctrl = control
        _global_kp = float(_ctrl.get("kp", 0.5))
        _global_kd = float(_ctrl.get("kd", 0.0125))
        _calf_kp   = float(_ctrl.get("kp_calf", _global_kp))
        _calf_kd   = float(_ctrl.get("kd_calf", _global_kd))
        self._base_kp: dict[str, float] = {}
        self._base_kd: dict[str, float] = {}
        for _j in cfg.get("joints", []):
            _n = _j["name"]
            _is_calf = "calf" in _n.lower()
            self._base_kp[_n] = float(_j["kp"]) if "kp" in _j else (_calf_kp if _is_calf else _global_kp)
            self._base_kd[_n] = float(_j["kd"]) if "kd" in _j else (_calf_kd if _is_calf else _global_kd)

        # Joint limits from robot.yaml — used to clip commanded q before publishing
        joint_list = cfg.get("joints", [])
        self._q_min = {j["name"]: float(j["q_min"]) for j in joint_list}
        self._q_max = {j["name"]: float(j["q_max"]) for j in joint_list}

        self._gait = GaitScheduler(
            period=float(self.get_parameter("gait_period").value),
            swing_ratio=float(self.get_parameter("swing_ratio").value),
        )

        self._q_default = np.array(
            [_DEFAULT_Q[n] for n in _YAML_JOINTS], dtype=float
        )
        self._joint_pos: dict[str, float] = {n: _DEFAULT_Q[n] for n in _YAML_JOINTS}
        self._joint_vel: dict[str, float] = {n: 0.0 for n in _YAML_JOINTS}

        self._state_estimate = np.zeros(10, dtype=float)
        self._est_stamp: float | None = None
        self._cmd_vel = np.zeros(3)           # [vx, vy, yaw_rate] world frame
        self._vel_filt = np.zeros(3)          # low-passed cmd_vel used for stride
        self._att_gx = 0.0                    # filtered tilt, fore-aft
        self._att_gy = 0.0                    # filtered tilt, lateral
        self._att_dgx = 0.0                   # filtered tilt derivative, fore-aft
        self._att_dgy = 0.0                   # filtered tilt derivative, lateral
        self._att_ix = 0.0                    # integral trim, fore-aft
        self._att_iy = 0.0                    # integral trim, lateral
        self._att_dz = {leg: 0.0 for leg in LEG_NAMES}  # slew-limited output
        # Ground-plane estimate from loaded-foot FK (see terrain_estimator).
        # Anchor pattern in the BODY frame: nominal hip-frame stance xy plus
        # the hip-mount offsets (see _hip_mount_xy).
        def _body_xy(leg: str) -> tuple[float, float]:
            mx, my = _hip_mount_xy(leg)
            nom = nominal_foot_position(leg, 0.27)
            return float(nom[0]) + mx, float(nom[1]) + my

        self._terrain = TerrainEstimator(
            foot_xy={leg: _body_xy(leg) for leg in LEG_NAMES},
            stance_height=float(self.get_parameter("stance_height").value),
        )
        self._terrain_dz_blend = 0.0   # enable ramp, foot-offset layer
        self._terrain_att_blend = 0.0  # enable ramp, attitude-reference layer
        # Live stance height: follows /height_command (LT/RT) slew-limited;
        # falls back to the stance_height parameter until a command arrives.
        self._stance_h = float(self.get_parameter("stance_height").value)
        self._height_cmd: float | None = None

        # Lift-off foot positions per leg (hip frame), updated at swing start
        self._lift_pos: dict[str, np.ndarray] = {
            leg: nominal_foot_position(leg, self._stance_h) for leg in LEG_NAMES
        }
        self._prev_contact: dict[str, bool] = {leg: True for leg in LEG_NAMES}
        # Last commanded foot target per leg — swing continues from this at
        # lift-off (not the measured pose, whose tracking error would step the
        # command trajectory).
        self._last_p_foot: dict[str, np.ndarray | None] = {leg: None for leg in LEG_NAMES}
        # Last commanded joint target per joint — finite-diff source for dq
        # feedforward and hold value when IK fails.
        self._prev_cmd_q: dict[str, float | None] = {n: None for n in _YAML_JOINTS}

        self._phase = _PHASE_PASSIVE
        self._phase_start: float | None = None
        self._stand_requested = False
        self._walking = False  # True only when cmd_vel exceeds threshold
        self._standup_start: list[float] | None = None
        self._initial_pos: list[float] | None = None
        self._lie_down_start: list[float] | None = None
        self._last_published: list[float] | None = None
        self._passive_broadcast = False

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self._pub_gains = self.create_publisher(Float32MultiArray, "/joint_gains", 10)
        # MPC internals for scripts/mpc_debug_check.py — layout:
        # [0] h_meas [1] vz_meas [2] z_err(clipped) [3] Σfz
        # [4:8] fz FR,FL,RR,RL [8] tau_blend [9] roll [10] pitch
        # [11] wx [12] wy (world) [13] stance_h ref
        # [14:16] terrain world slope [a, b]
        self._pub_debug = self.create_publisher(Float32MultiArray, "/mpc_debug", 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/posture_command", self._on_posture, 10)
        self.create_subscription(Float32, "/height_command", self._on_height, 10)

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"mpc_node ready — {tick_hz:.0f} Hz  "
            f"kp_scale={self.get_parameter('kp_scale').value} "
            f"kd_scale={self.get_parameter('kd_scale').value}  "
            f"period={self.get_parameter('gait_period').value:.2f}s"
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_joints(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self._joint_pos:
                self._joint_pos[name] = float(pos)
                self._joint_vel[name] = float(vel)

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_estimate = np.array(msg.data[:10], dtype=float)
        self._est_stamp = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = np.array([
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        ])

    def _on_height(self, msg: Float32) -> None:
        self._height_cmd = float(msg.data)

    def _update_stance_height(self) -> None:
        """Slew the live stance height toward the commanded target.

        /height_command (teleop LT/RT) wins over the stance_height parameter
        once the first message arrives. Slew limiting keeps a fresh subscriber
        (teleop publishes its own absolute target on connect) from stepping
        the body height.
        """
        target = self._height_cmd if self._height_cmd is not None \
            else float(self.get_parameter("stance_height").value)
        target = min(max(target, _HEIGHT_MIN), _HEIGHT_MAX)
        max_step = _HEIGHT_SLEW * self._dt
        self._stance_h += float(np.clip(target - self._stance_h, -max_step, max_step))

    def _attitude_dz(self) -> dict[str, float]:
        """IMU attitude leveling: per-foot z offsets from tilt PD, slew-limited.

        Tilt is read as the x/y components of the unit projected gravity in the
        body frame (0 when level). Damping comes from the numerical derivative
        of the same filtered tilt signal — never the raw gyro, whose axis
        convention depends on IMU mounting and would flip the damping into
        positive feedback if guessed wrong. Effort decays to zero when the
        estimate is stale or leveling is disabled, and the final correction is
        rate-limited so no tuning can make the feet jump.
        """
        kp = float(self.get_parameter("att_kp").value)
        kd = float(self.get_parameter("att_kd").value)
        ki = float(self.get_parameter("att_ki").value)
        g = _remove_mount_bias(
            np.asarray(self._state_estimate[6:9], dtype=float),
            float(self.get_parameter("att_roll_offset").value),
            float(self.get_parameter("att_pitch_offset").value),
        )
        g_norm = float(np.linalg.norm(g))
        fresh = (
            self._est_stamp is not None
            and (time.monotonic() - self._est_stamp) <= _EST_TIMEOUT
        )
        valid = fresh and g_norm >= 0.5 and kp > 0.0
        if valid:
            gx, gy = float(g[0]) / g_norm, float(g[1]) / g_norm
        else:
            gx = gy = 0.0
        a_g = min(1.0, self._dt / _ATT_TILT_TAU)
        a_d = min(1.0, self._dt / _ATT_DRV_TAU)
        prev_gx, prev_gy = self._att_gx, self._att_gy
        self._att_gx += a_g * (gx - self._att_gx)
        self._att_gy += a_g * (gy - self._att_gy)
        self._att_dgx += a_d * ((self._att_gx - prev_gx) / self._dt - self._att_dgx)
        self._att_dgy += a_d * ((self._att_gy - prev_gy) / self._dt - self._att_dgy)
        # Integral trim: the P loop settles at gain/(1+gain) rejection (~1/3 of
        # a static tilt at att_kp 0.08); the integrator walks out the residual
        # in ~1-2 s. Clamped against windup; leaks to zero whenever the tilt
        # signal is invalid so a stale estimate cannot hold a stance offset.
        if valid and ki > 0.0:
            self._att_ix = float(np.clip(self._att_ix + ki * self._dt * self._att_gx,
                                         -_ATT_I_MAX, _ATT_I_MAX))
            self._att_iy = float(np.clip(self._att_iy + ki * self._dt * self._att_gy,
                                         -_ATT_I_MAX, _ATT_I_MAX))
        else:
            leak = 1.0 - min(1.0, self._dt / _ATT_I_LEAK_TAU)
            self._att_ix *= leak
            self._att_iy *= leak
        u_x = kp * self._att_gx + kd * self._att_dgx + self._att_ix
        u_y = kp * self._att_gy + kd * self._att_dgy + self._att_iy
        raw = _leveling_dz(u_x, u_y)
        max_step = _ATT_DZ_SLEW * self._dt
        out: dict[str, float] = {}
        for leg, val in raw.items():
            prev = self._att_dz[leg]
            val = prev + float(np.clip(val - prev, -max_step, max_step))
            self._att_dz[leg] = val
            out[leg] = val
        return out

    def _R_body_est(self) -> np.ndarray:
        """Body→world rotation (yaw-free) from the bias-corrected gravity
        estimate — the same attitude the QP sees."""
        est = np.asarray(self._state_estimate, dtype=float).copy()
        est[6:9] = _remove_mount_bias(
            est[6:9],
            float(self.get_parameter("att_roll_offset").value),
            float(self.get_parameter("att_pitch_offset").value),
        )
        rpy = _state_from_estimate(est, np.zeros(3))[:3]
        return _euler_to_R(rpy)

    def _terrain_tick(
        self, leg_scale: dict[str, float], R_body: np.ndarray
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        """Update the ground-plane estimate from measured foot FK and return
        (body-frame terrain plane coefficients (a, b), MPC attitude ref).
        Foot targets evaluate dz = a·x + b·y at their OWN xy — the landing
        spot's terrain height, not the nominal stance point's.

        Anchors come from MEASURED joints — where the feet actually are —
        weighted by the stance load ramp, so a foot is trusted exactly while
        it is planted. Both output layers ride their own 0.3 s enable blend;
        on flat ground the plane fit is level and everything here is ≈ 0.
        """
        foot_body = {}
        for leg in LEG_NAMES:
            if float(leg_scale.get(leg, 0.0)) > 0.5:
                q = tuple(self._joint_pos[j] for j in _leg_joints(leg))
                mx, my = _hip_mount_xy(leg)
                p = np.asarray(forward_kinematics(leg, q), dtype=float)
                foot_body[leg] = p + np.array([mx, my, 0.0])
        self._terrain.update(foot_body, leg_scale, R_body, self._dt)

        a_b = min(1.0, self._dt / _TERRAIN_BLEND_TAU)
        dz_on = 1.0 if bool(self.get_parameter("terrain_adapt_enabled").value) else 0.0
        att_on = 1.0 if bool(self.get_parameter("terrain_att_ref_enabled").value) else 0.0
        self._terrain_dz_blend += a_b * (dz_on - self._terrain_dz_blend)
        self._terrain_att_blend += a_b * (att_on - self._terrain_att_blend)

        a_p, b_p = self._terrain.body_plane(R_body)
        plane = (self._terrain_dz_blend * a_p, self._terrain_dz_blend * b_p)
        r_ref, p_ref = self._terrain.ref_attitude()
        att_ref = (self._terrain_att_blend * r_ref, self._terrain_att_blend * p_ref)
        return plane, att_ref

    def _tau_feedforward(
        self,
        joint_targets: dict[str, float],
        leg_scale: dict[str, float],
        contact_schedule: list[list[bool]],
        stance_h: float,
        vel_ref: np.ndarray,
        yaw_ref: float,
        att_ref: tuple[float, float] = (0.0, 0.0),
    ) -> list[float]:
        """SRBD-MPC GRF → J^T·f feedforward torque, smooth by construction.

        Three smoothing layers guarantee no torque step at any transition:
        global enable blend (WALK entry / runtime toggle), per-leg stance load
        ramp in leg_scale, and a low-pass on the output vector (absorbs the
        force redistribution other legs see when a contact flips). Falls back
        to a decaying hold on solver failure and ramps out on stale estimates.
        """
        fresh = (
            self._est_stamp is not None
            and (time.monotonic() - self._est_stamp) <= _EST_TIMEOUT
        )
        enabled = bool(self.get_parameter("tau_ff_enabled").value) and fresh
        a_b = min(1.0, self._dt / _TAU_BLEND_TAU)
        self._tau_blend += a_b * ((1.0 if enabled else 0.0) - self._tau_blend)

        tau_raw = [0.0] * 12
        if self._tau_blend > 1e-3:
            est = np.asarray(self._state_estimate, dtype=float).copy()
            est[6:9] = _remove_mount_bias(
                est[6:9],
                float(self.get_parameter("att_roll_offset").value),
                float(self.get_parameter("att_pitch_offset").value),
            )
            srbd_state = _state_from_estimate(
                est, np.array([0.0, 0.0, stance_h])
            )
            # Attitude reference: terrain-parallel on a slope (att_ref from
            # the ground-plane fit, ≈ 0 on flat ground). Referencing level
            # on an incline makes the QP torque the body back to horizontal
            # forever, shifting weight onto the downhill legs.
            state_ref = np.array([
                float(att_ref[0]), float(att_ref[1]), 0.0,
                0.0, 0.0, stance_h,
                0.0, 0.0, yaw_ref,
                float(vel_ref[0]), float(vel_ref[1]), 0.0,
            ])
            R_body = _euler_to_R(srbd_state[:3])
            # lin_vel stays neutralised: no leg-odometry estimate yet and the
            # VIO axes are unverified — a flipped velocity feeding the QP is
            # anti-damping. wz too: yaw moments need horizontal forces, which
            # _project_vertical_grf discards, so measured wz could only
            # perturb the solve for zero output.
            srbd_state[8:12] = state_ref[8:12]
            # wx/wy: measured body rates → force-level attitude damping (the
            # MIT fast channel; unlike _attitude_dz there is no tilt-filter
            # lag in this path). The gyro is body-frame but the dynamics use
            # world-frame ω (Θ̇ ≈ Rz⁻¹ω) → rotate by R_body (yaw is 0 here).
            # Axis convention assumed standard; if a growing sway starts the
            # moment tau blends in, flip rate_fb_enabled off (see param note).
            a_r = min(1.0, self._dt / _RATE_LP_TAU)
            self._rate_lp += a_r * (R_body @ est[3:6] - self._rate_lp)
            if bool(self.get_parameter("rate_fb_enabled").value):
                srbd_state[6:8] = self._rate_lp[:2]
            else:
                srbd_state[6:8] = state_ref[6:8]
            self._mpc._Q[6, 6] = self._mpc._Q[7, 7] = float(
                self.get_parameter("rate_fb_weight").value
            )
            # z feedback: measured height + vertical rate from stance-leg FK
            # closes the height loop — sag becomes a z error becomes extra
            # lift, instead of relying on the mass feedforward being exact.
            # Load-ramp-weighted so a barely-touching foot barely counts;
            # same 30 ms LP as the gyro to smooth contact-set changes.
            zm = _measured_body_z(
                self._joint_pos, self._joint_vel, leg_scale, R_body
            )
            if zm is not None:
                if self._z_lp is None:
                    self._z_lp = np.array(zm)
                else:
                    self._z_lp += a_r * (np.array(zm) - self._z_lp)
            if zm is not None and bool(self.get_parameter("z_fb_enabled").value):
                srbd_state[5] = stance_h + float(np.clip(
                    self._z_lp[0] - stance_h, -_Z_ERR_CLIP, _Z_ERR_CLIP
                ))
                srbd_state[11] = float(np.clip(self._z_lp[1], -_VZ_CLIP, _VZ_CLIP))
            self._mpc._Q[5, 5] = float(self.get_parameter("z_fb_weight").value)
            self._mpc._Q[11, 11] = float(self.get_parameter("vz_fb_weight").value)
            # QP moments balance about the CoM, so foot vectors are taken
            # relative to it — not the body-frame origin. A real CoM forward
            # of the origin loads the front pair more; without this offset the
            # QP splits weight evenly and each stance leg holds a standing PD
            # error that releases as a twitch in the lift-off ramp window.
            self._mpc._mass = float(self.get_parameter("mass").value)
            com = np.array([
                float(self.get_parameter("com_x").value),
                float(self.get_parameter("com_y").value),
                0.0,
            ])
            foot_pos_world = np.zeros((4, 3))
            for i, leg in enumerate(_MPC_LEG_ORDER):
                joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
                foot_pos_world[i] = R_body @ (
                    np.array(forward_kinematics(leg, joints_leg)) - com
                )
            try:
                grf = self._mpc.solve(
                    srbd_state, state_ref, foot_pos_world, contact_schedule,
                    u_prev=self._grf_prev,
                )
                # Vertical-only projection (weight + leveling moments kept,
                # net horizontal push removed), then load ramp + hand-off at
                # force level: total commanded force stays equal to the
                # projected solution through every contact flip.
                grf = _project_vertical_grf(grf, foot_pos_world)
                self._grf_prev = grf.copy()
                grf = _apply_load_ramp(grf, leg_scale, mu=self._mpc._mu)
                fz = grf.reshape(4, 3)[:, 2]
                dbg = Float32MultiArray()
                dbg.data = [
                    float(self._z_lp[0]) if self._z_lp is not None else 0.0,
                    float(self._z_lp[1]) if self._z_lp is not None else 0.0,
                    float(srbd_state[5] - stance_h),
                    float(fz.sum()),
                    float(fz[0]), float(fz[1]), float(fz[2]), float(fz[3]),
                    float(self._tau_blend),
                    float(srbd_state[0]), float(srbd_state[1]),
                    float(srbd_state[6]), float(srbd_state[7]),
                    float(stance_h),
                    # [14:16] world-frame terrain slope [a, b] (LP'd fit,
                    # pre-blend) — sanity: ≈0 on flat, ≈±0.27 on a 15° ramp.
                    float(self._terrain.world_slope[0]),
                    float(self._terrain.world_slope[1]),
                ]
                self._pub_debug.publish(dbg)
                tau_raw = _build_stance_tau(
                    grf, joint_targets, {leg: 1.0 for leg in LEG_NAMES}, R_body
                )
                tau_raw = [t * self._tau_blend for t in tau_raw]
            except Exception as exc:
                self.get_logger().warn(
                    f"[mpc/tau] solver failed: {exc}", throttle_duration_sec=2.0
                )
                tau_raw = [t * 0.8 for t in self._tau_lp]
                self._grf_prev = None  # don't pull the next solve toward stale force

        a_lp = min(1.0, self._dt / _TAU_LP_TAU)
        self._tau_lp = [p + a_lp * (t - p) for p, t in zip(self._tau_lp, tau_raw)]
        return list(self._tau_lp)

    def _tau_rampout(self) -> list[float]:
        """Decay the held feedforward smoothly to zero — same time constant
        as the enable blend, so leaving WALK never steps the torque."""
        a_b = min(1.0, self._dt / _TAU_BLEND_TAU)
        self._tau_blend += a_b * (0.0 - self._tau_blend)
        self._tau_lp = [t * (1.0 - a_b) for t in self._tau_lp]
        return list(self._tau_lp)

    def _on_posture(self, msg: Bool) -> None:
        if bool(msg.data):
            if self._phase == _PHASE_PASSIVE:
                self._stand_requested = True
        else:
            if self._phase in (_PHASE_STANDUP, _PHASE_WALK):
                self._phase = _PHASE_LIEDOWN
                self._phase_start = time.monotonic()
                # Anchor the ramp to the last COMMANDED pose, not the measured
                # one: while gains are active, snapping the command onto the
                # measured pose injects the tracking error as a step — the PD
                # torque releases all at once and the body drops before the
                # ramp catches it. Measured is only right when no command has
                # been issued yet (kp was zero, so there is no error to step).
                self._lie_down_start = (
                    list(self._last_published)
                    if self._last_published is not None
                    else list(self._current_pos())
                )

    def _current_pos(self) -> list[float]:
        return [self._joint_pos[n] for n in _YAML_JOINTS]

    def _is_near(self, targets: list[float], tol: float) -> bool:
        pos = self._current_pos()
        return all(abs(p - t) <= tol for p, t in zip(pos, targets))

    def _is_settled(self) -> bool:
        return all(abs(self._joint_vel[n]) <= _VEL_SETTLED for n in _YAML_JOINTS)

    def _publish(self, cmd: JointCommand) -> None:
        clipped_q = [
            float(np.clip(q, self._q_min.get(n, -3.14), self._q_max.get(n, 3.14)))
            for n, q in zip(_YAML_JOINTS, cmd.q)
        ]
        self._last_published = clipped_q

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = list(_YAML_JOINTS)
        js.position = clipped_q
        js.velocity = list(cmd.dq)
        js.effort   = list(cmd.tau)
        self._pub.publish(js)

        gains = Float32MultiArray()
        gains.data = [val for pair in zip(cmd.kp, cmd.kd) for val in pair]
        self._pub_gains.publish(gains)

    def _fixed_gains(self) -> tuple[list[float], list[float]]:
        kp_scale = float(self.get_parameter("kp_scale").value)
        kd_scale = float(self.get_parameter("kd_scale").value)
        kp = [self._base_kp[n] * kp_scale for n in _YAML_JOINTS]
        kd = [self._base_kd[n] * kd_scale for n in _YAML_JOINTS]
        return kp, kd

    def _walk_gains(self, leg_scale: dict[str, float]) -> tuple[list[float], list[float]]:
        """Per-joint gains for WALK: loaded stance legs crossfade toward
        kp_stance_scale/kd_stance_scale as tau_ff takes their weight, swing
        legs keep the full kp_scale/kd_scale (no feedforward exists in the
        air — PD alone tracks the swing arc). Crossfade weight is the same
        load ramp that scales the leg's force, times the global tau blend,
        so gains soften exactly where — and only while — the feedforward is
        actually carrying the load. Call after _tau_feedforward so the blend
        state is current."""
        kp_base = float(self.get_parameter("kp_scale").value)
        kd_base = float(self.get_parameter("kd_scale").value)
        kp_st = float(self.get_parameter("kp_stance_scale").value)
        kd_st = float(self.get_parameter("kd_stance_scale").value)
        kp, kd = [], []
        for n in _YAML_JOINTS:
            w = float(leg_scale.get(n.split("_")[0], 0.0)) * self._tau_blend
            kp.append(self._base_kp[n] * _stance_gain_scale(kp_base, kp_st, w))
            kd.append(self._base_kd[n] * _stance_gain_scale(kd_base, kd_st, w))
        return kp, kd

    def _compute_stance_q(self, stance_h: float) -> list[float]:
        """IK-derived joint targets for nominal stance at stance_h. Used by both
        standup ramp and balance_stance so they share the same goal with no snap."""
        targets: dict[str, float] = {}
        for leg in _MPC_LEG_ORDER:
            p_foot = nominal_foot_position(leg, stance_h)
            preferred = tuple(self._joint_pos.get(j, 0.0) for j in _leg_joints(leg))
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                q_leg = tuple(self._q_default[i] for i, n in enumerate(_YAML_JOINTS) if n in _leg_joints(leg))
            for jname, qval in zip(_leg_joints(leg), q_leg):
                targets[jname] = float(qval)
        return [targets[n] for n in _YAML_JOINTS]

    def _standup_targets(self, elapsed: float) -> tuple[JointCommand, bool]:
        ramp = max(float(self.get_parameter("ramp_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / ramp)
        start  = self._standup_start or self._q_default.tolist()
        goal   = self._compute_stance_q(self._stance_h)
        q = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        kp, kd = self._fixed_gains()
        return JointCommand(q=q, dq=[0.0]*12, tau=[0.0]*12, kp=kp, kd=kd), elapsed >= ramp

    def _liedown_targets(self, elapsed: float) -> tuple[JointCommand, bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or self._q_default.tolist()
        goal  = self._initial_pos or [0.0] * 12
        q = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        # Ramp the held tau_ff out over _TAU_BLEND_TAU instead of cutting
        # ~full body weight of feedforward in one tick (the body would
        # free-fall onto the position ramp). Gains come from _walk_gains so
        # soft stance kp hardens in sync as the blend decays — same invariant
        # as everywhere else: soft PD only where tau_ff still carries load.
        tau = self._tau_rampout()
        kp, kd = self._walk_gains({leg: 1.0 for leg in LEG_NAMES})
        return JointCommand(q=q, dq=[0.0]*12, tau=tau, kp=kp, kd=kd), elapsed >= dur

    def _balance_stance(self, stance_h: float) -> JointCommand:
        """Four-foot stance: nominal IK pose + attitude leveling + MPC tau_ff.
        Terrain adaptation stays live here too — standing on a slope keeps
        the feet on the incline plane instead of forcing them coplanar with
        the (level-referenced) body."""
        leg_scale = {leg: 1.0 for leg in LEG_NAMES}
        R_body = self._R_body_est()
        terrain_plane, att_ref = self._terrain_tick(leg_scale, R_body)
        spread = _lateral_spread(
            stance_h,
            float(self.get_parameter("low_spread_start").value),
            float(self.get_parameter("low_spread_max").value),
        )
        dz = self._attitude_dz()
        targets: dict[str, float] = {}
        for leg in _MPC_LEG_ORDER:
            p_foot = nominal_foot_position(leg, stance_h)
            y_sp = _leg_signs(leg)[1] * spread
            mx, my = _hip_mount_xy(leg)
            dz_t = dz_on_plane(
                terrain_plane[0], terrain_plane[1],
                p_foot[0] + mx, p_foot[1] + y_sp + my,
            )
            p_foot = np.array([
                p_foot[0], p_foot[1] + y_sp, p_foot[2] + dz[leg] + dz_t
            ])
            preferred = tuple(self._joint_pos.get(j, 0.0) for j in _leg_joints(leg))
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                q_leg = tuple(_DEFAULT_Q[j] for j in _leg_joints(leg))
            for jname, qval in zip(_leg_joints(leg), q_leg):
                targets[jname] = float(qval)
        tau = self._tau_feedforward(
            targets,
            leg_scale=leg_scale,
            contact_schedule=[[True] * 4] * self._mpc._N,
            stance_h=stance_h,
            vel_ref=np.zeros(2),
            yaw_ref=0.0,
            att_ref=att_ref,
        )
        kp, kd = self._walk_gains(leg_scale)
        return JointCommand(
            q=[targets[n] for n in _YAML_JOINTS],
            dq=[0.0] * 12, tau=tau, kp=kp, kd=kd,
        )

    def _compute_mpc_joints(self, now: float) -> JointCommand:
        """Run one gait-scheduler step and return a full JointCommand."""
        stance_h = self._stance_h
        step_h   = float(self.get_parameter("step_height").value)
        # GaitScheduler.period/swing_ratio are plain attributes set once at
        # construction — `ros2 param set` alone never reached them. Sync both
        # every tick so runtime changes actually take effect; swing_ratio
        # especially, since the node reads its param per tick for the foot
        # trajectory and an unsynced scheduler would flip contacts on a
        # different clock than the trajectory it gates.
        self._gait.set_period(float(self.get_parameter("gait_period").value))
        self._gait.set_swing_ratio(float(self.get_parameter("swing_ratio").value))

        if self._est_stamp is None or (now - self._est_stamp) > _EST_TIMEOUT:
            self.get_logger().warn(
                "[mpc] state_estimate stale — holding stance", throttle_duration_sec=1.0
            )
            return self._balance_stance(stance_h)

        # Low-pass cmd_vel for the trajectory: stride grows/shrinks smoothly
        # instead of jumping when the stick steps. Walking keeps running until
        # the filtered stride has decayed to ~0, so stop never snaps either.
        alpha = min(1.0, self._dt / _VEL_FILTER_TAU)
        self._vel_filt += alpha * (self._cmd_vel - self._vel_filt)

        moving = (
            float(np.max(np.abs(self._cmd_vel))) >= _WALK_VEL_THRESH
            or float(np.max(np.abs(self._vel_filt))) >= _WALK_VEL_THRESH
        )

        if not self._walking:
            if not moving:
                return self._balance_stance(stance_h)
            self._gait.reset()
            self._prev_contact = {leg: True for leg in LEG_NAMES}
            self._lift_pos = {
                leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
            }
            self._last_p_foot = {leg: None for leg in LEG_NAMES}
            self._prev_cmd_q = {n: None for n in _YAML_JOINTS}
            self._walking = True

        gait_state = self._gait.query(now)

        if not moving and all(gait_state[leg]["contact"] for leg in LEG_NAMES):
            # Stop only inside an all-contact window: cutting a mid-air swing
            # would snap that leg straight to the stance pose. With
            # swing_ratio < 0.5 the trot has two such windows per period, so
            # this delays the stop by at most a fraction of a cycle.
            self._gait.reset()
            self._prev_contact = {leg: True for leg in LEG_NAMES}
            self._lift_pos = {
                leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
            }
            self._last_p_foot = {leg: None for leg in LEG_NAMES}
            self._prev_cmd_q = {n: None for n in _YAML_JOINTS}
            self._vel_filt[:] = 0.0
            self._walking = False
            return self._balance_stance(stance_h)
        # Read back from the scheduler (post-clamp), not the raw parameter:
        # trajectory and contact schedule must share one swing_ratio.
        swing_ratio = self._gait.swing_ratio
        # Pure position control has no dynamics path from cmd_vel to actual
        # body motion (no tau_ff pushing the body), so the measured
        # state_estimate velocity stays ~0 even while walking is commanded.
        # The (filtered) command velocity drives the trajectory directly.
        # Yaw enters as a per-leg cross term (v + ω × r): the two body sides
        # get opposite fore-aft strokes, which is what turns the body.
        vel_xy = self._vel_filt[0:2]
        yaw_rate = float(self._vel_filt[2])

        for leg in LEG_NAMES:
            in_contact = gait_state[leg]["contact"]
            if self._prev_contact[leg] and not in_contact:
                # Lift-off: swing continues from the last commanded target so
                # the command trajectory stays continuous. Fall back to FK of
                # the measured pose only when no command has been issued yet.
                last = self._last_p_foot.get(leg)
                if last is not None:
                    self._lift_pos[leg] = np.array(last)
                else:
                    joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))
                    self._lift_pos[leg] = np.array(forward_kinematics(leg, joints_leg))
            self._prev_contact[leg] = in_contact

        # Per-leg load ramp: zero commanded force at touchdown and lift-off.
        # Computed before the target loop because it doubles as the anchor
        # trust weighting for the terrain estimate the targets depend on.
        ramp_frac = float(self.get_parameter("tau_ramp_frac").value)
        leg_scale = {
            leg: (_stance_load_ramp(self._gait.stance_phase(leg, now), ramp_frac)
                  if gait_state[leg]["contact"] else 0.0)
            for leg in LEG_NAMES
        }
        R_body = self._R_body_est()
        terrain_plane, att_ref = self._terrain_tick(leg_scale, R_body)
        spread = _lateral_spread(
            stance_h,
            float(self.get_parameter("low_spread_start").value),
            float(self.get_parameter("low_spread_max").value),
        )

        joint_targets: dict[str, float] = {}
        dq_targets:    dict[str, float] = {}
        att_dz = self._attitude_dz()

        for leg in LEG_NAMES:
            in_contact = gait_state[leg]["contact"]
            v_leg = leg_velocity(vel_xy, yaw_rate, leg)

            if in_contact:
                # Stance stroke: touchdown at +offset (== landing_target, where
                # the previous swing ended), sweep back to −offset at lift-off
                # (where the next swing starts). The backward sweep is what
                # propels the body in position control.
                s_st = self._gait.stance_phase(leg, now)
                p_foot = stance_foot_position(
                    leg, s_st, v_leg, self._gait.period, swing_ratio, stance_h
                )
            else:
                s = self._gait.swing_phase(leg, now)
                p_land = landing_target(
                    leg, v_leg, self._gait.period, swing_ratio, stance_h
                )
                # Clearance scales with leg speed so near-zero-stride steps
                # (stride ramping in/out) stay near the ground.
                step_scale = min(1.0, float(np.hypot(*v_leg)) / _STEP_VEL_REF)
                # Endpoint slope −v·T_swing: zero ground-relative foot velocity
                # at lift-off/touchdown. If body sag keeps the "swinging" foot
                # loaded near those moments, it then pushes the body the same
                # way the stance legs do instead of dragging it backward.
                t_swing = max(swing_ratio * self._gait.period, 1e-6)
                p_foot = swing_foot_position(
                    s, self._lift_pos[leg], p_land, step_h * step_scale,
                    xy_end_slope=-v_leg * t_swing,
                )
            # Lift-pos capture stays in the un-leveled, un-terrained frame;
            # leveling and terrain offsets are added after, to stance and
            # swing alike, so phase transitions stay continuous and neither
            # dz ever double-counts. Terrain dz is evaluated at the target's
            # own xy: mid-swing the offset slides along the plane and the
            # touchdown lands on the terrain height of the actual landing
            # spot — this is what removes the slope-transition early/late
            # touchdown.
            self._last_p_foot[leg] = np.array(p_foot)
            # Low-posture spread first (lateral), then terrain dz evaluated
            # at the xy the foot will actually occupy.
            y_sp = _leg_signs(leg)[1] * spread
            mx, my = _hip_mount_xy(leg)
            dz_t = dz_on_plane(
                terrain_plane[0], terrain_plane[1],
                p_foot[0] + mx, p_foot[1] + y_sp + my,
            )
            p_foot = np.array([
                p_foot[0], p_foot[1] + y_sp, p_foot[2] + att_dz[leg] + dz_t
            ])

            preferred = tuple(self._joint_pos[j] for j in _leg_joints(leg))
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                # Hold the previous commanded angles rather than snapping to
                # the default pose — an IK miss must not step the command.
                # Repeated misses freeze-then-jump the foot; on slopes the
                # usual cause is terrain dz stretching the downhill leg past
                # its workspace edge.
                self.get_logger().warn(
                    f"IK miss {leg} target={np.round(p_foot, 3).tolist()}",
                    throttle_duration_sec=1.0,
                )
                q_leg = tuple(
                    self._prev_cmd_q[j] if self._prev_cmd_q[j] is not None else _DEFAULT_Q[j]
                    for j in _leg_joints(leg)
                )

            for jname, qval in zip(_leg_joints(leg), q_leg):
                qval = float(qval)
                joint_targets[jname] = qval
                prev = self._prev_cmd_q[jname]
                # Stance strokes too now, so dq feedforward applies in both
                # phases — finite-diff of consecutive commanded targets.
                dq_targets[jname] = float(
                    np.clip((qval - prev) / self._dt, -12.0, 12.0)
                ) if prev is not None else 0.0
                self._prev_cmd_q[jname] = qval

        # Future contacts sampled at the MPC's own step, not the tick period
        # — this is what gives the QP its lookahead. Each entry is the
        # fraction of that prediction step spent in contact (subsampled),
        # not a boolean: with mpc_dt > tick dt a flip time would otherwise
        # jump a whole bucket between consecutive ticks and step the QP
        # solution (~3 Nm/tick on the calves, verified offline). Foot
        # positions are held at their current commanded targets across the
        # window; a foot that touches down mid-horizon is a few cm off its
        # true landing spot, a second-order moment-arm error we accept.
        mdt = self._mpc._dt
        contact_schedule = []
        for k in range(self._mpc._N):
            frac = [0.0] * 4
            for j in range(_SCHEDULE_SUBSAMPLES):
                gs_k = self._gait.query(
                    now + (k + (j + 0.5) / _SCHEDULE_SUBSAMPLES) * mdt
                )
                for i, leg in enumerate(_MPC_LEG_ORDER):
                    if gs_k[leg]["contact"]:
                        frac[i] += 1.0 / _SCHEDULE_SUBSAMPLES
            contact_schedule.append(frac)
        tau = self._tau_feedforward(
            joint_targets, leg_scale, contact_schedule,
            stance_h, vel_xy, yaw_rate, att_ref=att_ref,
        )

        kp, kd = self._walk_gains(leg_scale)

        return JointCommand(
            q=[joint_targets[n] for n in _YAML_JOINTS],
            dq=[dq_targets.get(n, 0.0) for n in _YAML_JOINTS],
            tau=tau,
            kp=kp,
            kd=kd,
        )

    def _tick(self) -> None:
        now = time.monotonic()
        self._update_stance_height()

        if self._phase == _PHASE_PASSIVE:
            if self._stand_requested:
                self._phase = _PHASE_STANDUP
                self._phase_start = now
                snap = list(self._current_pos())
                self._standup_start = snap
                if self._initial_pos is None:
                    self._initial_pos = snap
                self._last_published = None
                self._stand_requested = False
                self._gait.reset()
                self.get_logger().info("[mpc] posture=true → STANDUP")
            return

        if self._phase_start is None:
            self._phase_start = now
        elapsed = now - self._phase_start

        if self._phase == _PHASE_STANDUP:
            cmd, done = self._standup_targets(elapsed)
            self._publish(cmd)
            est_fresh = self._est_stamp is not None and (now - self._est_stamp) <= _EST_TIMEOUT
            if done and self._is_near(self._compute_stance_q(self._stance_h), _STANDUP_TOL) and self._is_settled():
                if not est_fresh:
                    self.get_logger().warn(
                        "[mpc] standup done but state_estimate stale — holding, waiting for estimate",
                        throttle_duration_sec=2.0,
                    )
                else:
                    self._phase = _PHASE_WALK
                    self._phase_start = now
                    self._gait.reset()
                    self._att_gx = self._att_gy = self._att_dgx = self._att_dgy = 0.0
                    self._att_ix = self._att_iy = 0.0
                    self._att_dz = {leg: 0.0 for leg in LEG_NAMES}
                    self._tau_blend = 0.0
                    self._tau_lp = [0.0] * 12
                    self._rate_lp = np.zeros(3)
                    self._z_lp = None
                    self._grf_prev = None
                    self._terrain.reset(self._stance_h)
                    self.get_logger().info("[mpc] standup done → WALK")
            elif done and elapsed > float(self.get_parameter("ramp_duration").value) + 5.0:
                self._phase = _PHASE_WALK
                self._phase_start = now
                self._gait.reset()
                self._att_gx = self._att_gy = self._att_dgx = self._att_dgy = 0.0
                self._att_ix = self._att_iy = 0.0
                self._att_dz = {leg: 0.0 for leg in LEG_NAMES}
                self._tau_blend = 0.0
                self._tau_lp = [0.0] * 12
                self._rate_lp = np.zeros(3)
                self._z_lp = None
                self._grf_prev = None
                self._terrain.reset(self._stance_h)
                self.get_logger().warn("[mpc] standup timeout → WALK")
            return

        if self._phase == _PHASE_WALK:
            cmd = self._compute_mpc_joints(now)
            self._publish(cmd)
            return

        if self._phase == _PHASE_LIEDOWN:
            cmd, done = self._liedown_targets(elapsed)
            self._publish(cmd)
            lie_dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
            near = self._is_near(self._initial_pos or [0.0] * 12, 0.05) and self._is_settled()
            timed_out = done and elapsed > lie_dur + _LIEDOWN_TIMEOUT
            if (done and near) or timed_out:
                self._phase = _PHASE_PASSIVE
                self._phase_start = None
                if timed_out and not near:
                    self.get_logger().warn("[mpc] liedown timeout → PASSIVE")
                else:
                    self.get_logger().info("[mpc] liedown done → PASSIVE")
            return


def main() -> None:
    rclpy.init()
    node = MPCNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
