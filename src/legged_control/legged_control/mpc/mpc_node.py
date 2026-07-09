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
    _leg_signs,
    _numerical_jacobian,
    _smoothstep,
)
from legged_control.mpc.gait_scheduler import GaitScheduler, LEG_NAMES
from legged_control.mpc.srbd_mpc import SRBDMPC, _euler_to_R
from legged_control.mpc.swing_trajectory import (
    swing_foot_position,
    stance_foot_position,
    nominal_foot_position,
    landing_target,
    leg_velocity,
)


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


def _stance_load_ramp(s: float) -> float:
    """Per-leg force scale over stance progress s ∈ [0,1]: 0 at touchdown,
    1 through mid-stance, 0 at lift-off (smoothstep ramps of _TAU_RAMP_FRAC)."""
    s = float(np.clip(s, 0.0, 1.0))
    return _smoothstep(s / _TAU_RAMP_FRAC) * _smoothstep((1.0 - s) / _TAU_RAMP_FRAC)


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
        # IMU attitude leveling (WALK phase): tilt from projected_gravity,
        # damping from that signal's own derivative → per-foot z offsets.
        # att_kp in m per unit tilt (≈ m/rad for small angles); 0 disables.
        # Loop gain: dz maps back to commanded counter-tilt as 2·att_kp/track
        # (track ≈ 0.32 m) — att_kp 0.16 is unity DC loop gain, keep well below.
        self.declare_parameter("att_kp", float(mpc_cfg.get("att_kp", 0.08)))
        self.declare_parameter("att_kd", float(mpc_cfg.get("att_kd", 0.02)))
        self.declare_parameter("att_ki", float(mpc_cfg.get("att_ki", 0.10)))
        # SRBD-MPC GRF feedforward on top of the (unchanged) position gait.
        # Runtime A/B: `ros2 param set /mpc_node tau_ff_enabled false` — the
        # global blend ramps it out over ~0.3 s, never a step.
        self.declare_parameter("tau_ff_enabled", bool(mpc_cfg.get("tau_ff_enabled", True)))

        mass = float(mpc_cfg.get("mass", 14.55))
        inertia = np.diag([
            float(mpc_cfg.get("Ixx", 0.0196)),
            float(mpc_cfg.get("Iyy", 0.0228)),
            float(mpc_cfg.get("Izz", 0.0169)),
        ])
        horizon = int(mpc_cfg.get("horizon", 6))
        self._mpc = SRBDMPC(mass=mass, inertia_body=inertia, dt=self._dt, horizon=horizon)
        self._tau_blend = 0.0            # global enable ramp state
        self._tau_lp = [0.0] * 12        # low-passed output torque

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
        g = self._state_estimate[6:9]
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

    def _tau_feedforward(
        self,
        joint_targets: dict[str, float],
        leg_scale: dict[str, float],
        contact_schedule: list[list[bool]],
        stance_h: float,
        vel_ref: np.ndarray,
        yaw_ref: float,
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
            srbd_state = _state_from_estimate(
                self._state_estimate, np.array([0.0, 0.0, stance_h])
            )
            state_ref = np.array([
                0.0, 0.0, 0.0,
                0.0, 0.0, stance_h,
                0.0, 0.0, yaw_ref,
                float(vel_ref[0]), float(vel_ref[1]), 0.0,
            ])
            # Neutralise every velocity channel (ang_vel + lin_vel): raw IMU
            # gyro axes are mounting-dependent and were already caught flipping
            # a damping loop into fast positive feedback (see _attitude_dz);
            # VIO lin_vel axes are equally unverified. A flipped rate feeding
            # the QP is ANTI-damping — observed as a limit cycle that starts
            # the moment standup completes. Zero error → zero force response:
            # tau_ff does only verified work (weight support + gravity-vector
            # attitude P). Physical damping comes from the motor PD.
            srbd_state[6:12] = state_ref[6:12]
            R_body = _euler_to_R(srbd_state[:3])
            foot_pos_world = np.zeros((4, 3))
            for i, leg in enumerate(_MPC_LEG_ORDER):
                joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
                foot_pos_world[i] = R_body @ np.array(forward_kinematics(leg, joints_leg))
            try:
                grf = self._mpc.solve(
                    srbd_state, state_ref, foot_pos_world, contact_schedule
                )
                # Load ramp + hand-off at force level: total commanded force
                # stays equal to the QP solution through every contact flip
                # (a leg ramping out passes its share to the loaded legs).
                grf = _apply_load_ramp(grf, leg_scale, mu=self._mpc._mu)
                tau_raw = _build_stance_tau(
                    grf, joint_targets, {leg: 1.0 for leg in LEG_NAMES}, R_body
                )
                tau_raw = [t * self._tau_blend for t in tau_raw]
            except Exception as exc:
                self.get_logger().warn(
                    f"[mpc/tau] solver failed: {exc}", throttle_duration_sec=2.0
                )
                tau_raw = [t * 0.8 for t in self._tau_lp]

        a_lp = min(1.0, self._dt / _TAU_LP_TAU)
        self._tau_lp = [p + a_lp * (t - p) for p, t in zip(self._tau_lp, tau_raw)]
        return list(self._tau_lp)

    def _on_posture(self, msg: Bool) -> None:
        if bool(msg.data):
            if self._phase == _PHASE_PASSIVE:
                self._stand_requested = True
        else:
            if self._phase in (_PHASE_STANDUP, _PHASE_WALK):
                self._phase = _PHASE_LIEDOWN
                self._phase_start = time.monotonic()
                self._lie_down_start = list(self._current_pos())

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
        kp, kd = self._fixed_gains()
        return JointCommand(q=q, dq=[0.0]*12, tau=[0.0]*12, kp=kp, kd=kd), elapsed >= dur

    def _balance_stance(self, stance_h: float) -> JointCommand:
        """Four-foot stance: nominal IK pose + attitude leveling + MPC tau_ff."""
        dz = self._attitude_dz()
        targets: dict[str, float] = {}
        for leg in _MPC_LEG_ORDER:
            p_foot = nominal_foot_position(leg, stance_h)
            p_foot = np.array([p_foot[0], p_foot[1], p_foot[2] + dz[leg]])
            preferred = tuple(self._joint_pos.get(j, 0.0) for j in _leg_joints(leg))
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                q_leg = tuple(_DEFAULT_Q[j] for j in _leg_joints(leg))
            for jname, qval in zip(_leg_joints(leg), q_leg):
                targets[jname] = float(qval)
        tau = self._tau_feedforward(
            targets,
            leg_scale={leg: 1.0 for leg in LEG_NAMES},
            contact_schedule=[[True] * 4] * self._mpc._N,
            stance_h=stance_h,
            vel_ref=np.zeros(2),
            yaw_ref=0.0,
        )
        kp, kd = self._fixed_gains()
        return JointCommand(
            q=[targets[n] for n in _YAML_JOINTS],
            dq=[0.0] * 12, tau=tau, kp=kp, kd=kd,
        )

    def _compute_mpc_joints(self, now: float) -> JointCommand:
        """Run one gait-scheduler step and return a full JointCommand."""
        stance_h = self._stance_h
        step_h   = float(self.get_parameter("step_height").value)
        # GaitScheduler.period is a plain attribute set once at construction —
        # `ros2 param set gait_period` alone never reached it. Sync every tick
        # so runtime changes actually take effect.
        self._gait.set_period(float(self.get_parameter("gait_period").value))

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
        swing_ratio = float(self.get_parameter("swing_ratio").value)
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
            # Lift-pos capture stays in the un-leveled frame; the leveling
            # offset is added after, to stance and swing alike, so phase
            # transitions stay continuous and dz never double-counts.
            self._last_p_foot[leg] = np.array(p_foot)
            p_foot = np.array([p_foot[0], p_foot[1], p_foot[2] + att_dz[leg]])

            preferred = tuple(self._joint_pos[j] for j in _leg_joints(leg))
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                # Hold the previous commanded angles rather than snapping to
                # the default pose — an IK miss must not step the command.
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

        # Per-leg load ramp: zero commanded force at touchdown and lift-off.
        leg_scale = {
            leg: (_stance_load_ramp(self._gait.stance_phase(leg, now))
                  if gait_state[leg]["contact"] else 0.0)
            for leg in LEG_NAMES
        }
        contact_schedule = [
            [self._gait.query(now + k * self._dt)[leg]["contact"]
             for leg in _MPC_LEG_ORDER]
            for k in range(self._mpc._N)
        ]
        tau = self._tau_feedforward(
            joint_targets, leg_scale, contact_schedule,
            stance_h, vel_xy, yaw_rate,
        )

        kp, kd = self._fixed_gains()

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
