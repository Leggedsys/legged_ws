"""mpc_node — Single Rigid Body MPC locomotion controller.

Replaces policy_node for non-RL trot gait. Output is /joint_commands
(URDF frame, same as policy_node), so all downstream hardware nodes are
unchanged.

FSM:
  PASSIVE → (posture_command=true) → STANDUP → WALK → PASSIVE
  B-button (posture_command=false) → LIEDOWN → PASSIVE from any state.

/joint_commands published at gait_hz (default 50 Hz).
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
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Twist

from legged_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    _numerical_jacobian,
    _smoothstep,
)
from legged_control.mpc.gait_scheduler import GaitScheduler, LEG_NAMES
from legged_control.mpc.swing_trajectory import (
    swing_foot_position,
    nominal_foot_position,
    landing_target,
)
from legged_control.mpc.srbd_mpc import SRBDMPC, _euler_to_R


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

# MPC leg ordering: FR=0, FL=1, RR=2, RL=3
_MPC_LEG_ORDER = ["FR", "FL", "RR", "RL"]


@dataclass
class JointCommand:
    """Per-tick joint command output from the MPC controller."""
    q:   list[float]
    dq:  list[float]
    tau: list[float]
    kp:  list[float]
    kd:  list[float]


def _leg_joints(leg: str) -> list[str]:
    return [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]


def _state_from_estimate(est: np.ndarray, pos: np.ndarray | None = None) -> np.ndarray:
    """Build 12-dim SRBD state from /state_estimate and optional CoM position.

    /state_estimate layout (from state_estimator_node):
      [0:3]  base_lin_vel (body frame)
      [3:6]  base_ang_vel (body frame)
      [6:9]  projected_gravity
      [9]    health flag

    We approximate roll/pitch from projected_gravity, yaw from 0 (not observable
    from IMU alone without mag). CoM position tracked by integration if no VIO.
    """
    ang_vel = est[3:6]   # body frame ω
    proj_g  = est[6:9]   # body frame projected gravity

    # roll/pitch from gravity direction (small angle approximation)
    roll  = float(np.arctan2(proj_g[1], -proj_g[2]))
    pitch = float(np.arctan2(-proj_g[0], -proj_g[2]))
    yaw   = 0.0           # IMU-only: no yaw from gravity

    lin_vel = est[0:3]   # body frame

    if pos is None:
        pos = np.zeros(3)

    return np.array([
        roll, pitch, yaw,
        pos[0], pos[1], pos[2],
        ang_vel[0], ang_vel[1], ang_vel[2],
        lin_vel[0], lin_vel[1], lin_vel[2],
    ], dtype=float)


def _build_stance_tau(
    grf: np.ndarray,
    joint_targets: dict[str, float],
    contact_now: list[bool] | None = None,
) -> list[float]:
    """Convert MPC GRF solution to joint torques via Jacobian transpose.

    τ_ff[leg] = J(q)^T · f_contact   (stance legs only; swing legs get 0)

    Args:
        grf:          12-element GRF vector [f0x,f0y,f0z, ..., f3x,f3y,f3z]
        joint_targets: per-joint URDF-frame angles used for Jacobian evaluation
        contact_now:  4-bool list [FR,FL,RR,RL]; None = all in contact

    Returns:
        12-element list of joint torques in YAML_JOINTS order
    """
    if contact_now is None:
        contact_now = [True, True, True, True]
    tau_dict: dict[str, float] = {n: 0.0 for n in _YAML_JOINTS}
    for i, leg in enumerate(_MPC_LEG_ORDER):
        if not contact_now[i]:
            continue
        f_leg = grf[i * 3 : i * 3 + 3]
        joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
        J = _numerical_jacobian(leg, joints_leg)
        tau_leg = J.T @ f_leg
        for jname, t in zip(_leg_joints(leg), tau_leg):
            tau_dict[jname] = float(t)
    return [tau_dict[n] for n in _YAML_JOINTS]


class MPCNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_node")

        cfg = self._load_config()
        control = cfg["control"]
        standup_cfg = cfg.get("standup", {})

        self.declare_parameter("gait_period",     0.6)
        self.declare_parameter("swing_ratio",      0.4)
        self.declare_parameter("step_height",      0.06)
        self.declare_parameter("stance_height",    0.27)
        self.declare_parameter("ramp_duration",    float(standup_cfg.get("ramp_duration", 6.0)))
        self.declare_parameter("lie_down_duration", float(standup_cfg.get("lie_down_duration", 2.0)))

        gait_hz = float(control.get("gait_hz", 50.0))
        self._dt = 1.0 / gait_hz

        # Robot physical parameters for MPC
        mpc_cfg = cfg.get("mpc", {})
        mass = float(mpc_cfg.get("mass", 12.0))
        Ixx  = float(mpc_cfg.get("Ixx", 0.017))
        Iyy  = float(mpc_cfg.get("Iyy", 0.056))
        Izz  = float(mpc_cfg.get("Izz", 0.064))
        inertia = np.diag([Ixx, Iyy, Izz])

        # K_joint deprecated (torque feedforward doesn't need this approximation)
        _ = mpc_cfg.get("K_joint", 20.0)

        # kp/kd scale factors for stance/swing phase switching
        self._kp_stance_scale = float(mpc_cfg.get("kp_stance_scale", 1.0))
        self._kd_stance_scale = float(mpc_cfg.get("kd_stance_scale", 1.0))
        self._kp_swing_scale  = float(mpc_cfg.get("kp_swing_scale",  1.0))
        self._kd_swing_scale  = float(mpc_cfg.get("kd_swing_scale",  1.0))

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

        # Joint limits from robot.yaml — used to clip MPC output before publishing
        joint_list = cfg.get("joints", [])
        self._q_min = {j["name"]: float(j["q_min"]) for j in joint_list}
        self._q_max = {j["name"]: float(j["q_max"]) for j in joint_list}

        horizon = int(mpc_cfg.get("horizon", 10))
        self._mpc = SRBDMPC(
            mass=mass,
            inertia_body=inertia,
            dt=self._dt,
            horizon=horizon,
        )

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
        self._com_pos = np.zeros(3)           # integrated CoM position
        self._com_pos[2] = float(self.get_parameter("stance_height").value)

        # Lift-off foot positions per leg (hip frame), updated at swing start
        self._lift_pos: dict[str, np.ndarray] = {
            leg: nominal_foot_position(leg, self.get_parameter("stance_height").value)
            for leg in LEG_NAMES
        }
        self._prev_contact: dict[str, bool] = {leg: True for leg in LEG_NAMES}
        self._prev_swing_q: dict[str, float | None] = {n: None for n in _YAML_JOINTS}

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

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"mpc_node ready — {gait_hz:.0f} Hz  "
            f"mass={mass} kg  horizon={horizon}  "
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

    def _on_posture(self, msg: Bool) -> None:
        if bool(msg.data):
            if self._phase == _PHASE_PASSIVE:
                self._stand_requested = True
        else:
            if self._phase in (_PHASE_STANDUP, _PHASE_WALK):
                self._phase = _PHASE_LIEDOWN
                self._phase_start = time.monotonic()
                self._lie_down_start = list(self._last_published or self._q_default.tolist())

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
        stance_h = float(self.get_parameter("stance_height").value)
        goal   = self._compute_stance_q(stance_h)
        q = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        kp = [self._base_kp[n] * self._kp_swing_scale for n in _YAML_JOINTS]
        kd = [self._base_kd[n] * self._kd_swing_scale for n in _YAML_JOINTS]
        return JointCommand(q=q, dq=[0.0]*12, tau=[0.0]*12, kp=kp, kd=kd), elapsed >= ramp

    def _liedown_targets(self, elapsed: float) -> tuple[JointCommand, bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or self._q_default.tolist()
        goal  = self._initial_pos or [0.0] * 12
        q = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        kp = [self._base_kp[n] * self._kp_swing_scale for n in _YAML_JOINTS]
        kd = [self._base_kd[n] * self._kd_swing_scale for n in _YAML_JOINTS]
        return JointCommand(q=q, dq=[0.0]*12, tau=[0.0]*12, kp=kp, kd=kd), elapsed >= dur

    def _balance_stance(self, stance_h: float) -> JointCommand:
        """Four-foot MPC balance: all legs in contact, zero velocity reference."""
        stance_q = self._compute_stance_q(stance_h)
        joint_targets = dict(zip(_YAML_JOINTS, stance_q))

        srbd_state = _state_from_estimate(self._state_estimate, self._com_pos)
        state_ref = np.array([
            0.0, 0.0, 0.0,
            0.0, 0.0, stance_h,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ])
        contact_schedule = [[True, True, True, True]] * self._mpc._N

        R_body = _euler_to_R(srbd_state[:3])
        foot_pos_world = np.zeros((4, 3))
        for i, leg in enumerate(_MPC_LEG_ORDER):
            joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
            foot_pos_world[i] = R_body @ np.array(forward_kinematics(leg, joints_leg))

        tau_list = [0.0] * 12
        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            tau_list = _build_stance_tau(grf, joint_targets)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc/balance] solver failed: {exc}", throttle_duration_sec=2.0
            )

        kp = [self._base_kp[n] * self._kp_stance_scale for n in _YAML_JOINTS]
        kd = [self._base_kd[n] * self._kd_stance_scale for n in _YAML_JOINTS]
        return JointCommand(
            q=[joint_targets[n] for n in _YAML_JOINTS],
            dq=[0.0] * 12,
            tau=tau_list,
            kp=kp,
            kd=kd,
        )

    def _compute_mpc_joints(self, now: float) -> JointCommand:
        """Run one MPC step and return a full JointCommand."""
        stance_h = float(self.get_parameter("stance_height").value)
        step_h   = float(self.get_parameter("step_height").value)

        if self._est_stamp is None or (now - self._est_stamp) > _EST_TIMEOUT:
            self.get_logger().warn(
                "[mpc] state_estimate stale — holding stance", throttle_duration_sec=1.0
            )
            return self._balance_stance(stance_h)

        moving = float(np.max(np.abs(self._cmd_vel))) >= _WALK_VEL_THRESH
        if not moving:
            if self._walking:
                self._gait.reset()
                self._prev_contact = {leg: True for leg in LEG_NAMES}
                self._lift_pos = {
                    leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
                }
                self._prev_swing_q = {n: None for n in _YAML_JOINTS}
                self._walking = False
            return self._balance_stance(stance_h)

        if not self._walking:
            self._gait.reset()
            self._lift_pos = {
                leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
            }
            self._prev_swing_q = {n: None for n in _YAML_JOINTS}
            self._walking = True

        gait_state = self._gait.query(now)

        for leg in LEG_NAMES:
            in_contact = gait_state[leg]["contact"]
            if self._prev_contact[leg] and not in_contact:
                joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))
                self._lift_pos[leg] = np.array(forward_kinematics(leg, joints_leg))
            self._prev_contact[leg] = in_contact

        joint_targets: dict[str, float] = {}
        dq_targets:    dict[str, float] = {}

        for leg in LEG_NAMES:
            in_contact = gait_state[leg]["contact"]
            joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))

            if in_contact:
                p_foot = nominal_foot_position(leg, stance_h)
                for jname in _leg_joints(leg):
                    dq_targets[jname] = 0.0
            else:
                s = self._gait.swing_phase(leg, now)
                body_vel_xy = self._state_estimate[0:2]
                p_land = landing_target(
                    leg, body_vel_xy,
                    self._gait.period,
                    float(self.get_parameter("swing_ratio").value),
                    stance_h,
                )
                p_foot = swing_foot_position(s, self._lift_pos[leg], p_land, step_h)

            preferred = joints_leg
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                q_leg = (
                    _DEFAULT_Q[f"{leg}_hip"],
                    _DEFAULT_Q[f"{leg}_thigh"],
                    _DEFAULT_Q[f"{leg}_calf"],
                )

            for jname, qval in zip(_leg_joints(leg), q_leg):
                joint_targets[jname] = float(qval)

            if not in_contact:
                for jname, qval in zip(_leg_joints(leg), q_leg):
                    prev = self._prev_swing_q.get(jname)
                    dq_targets[jname] = float(
                        np.clip((qval - prev) / self._dt, -12.0, 12.0)
                    ) if prev is not None else 0.0
                    self._prev_swing_q[jname] = float(qval)
            else:
                for jname in _leg_joints(leg):
                    self._prev_swing_q[jname] = None

        srbd_state = _state_from_estimate(self._state_estimate, self._com_pos)
        state_ref = np.array([
            0.0, 0.0, 0.0,
            0.0, 0.0, stance_h,
            0.0, 0.0, self._cmd_vel[2],
            self._cmd_vel[0], self._cmd_vel[1], 0.0,
        ])

        contact_now = [gait_state[leg]["contact"] for leg in _MPC_LEG_ORDER]
        contact_schedule = []
        for k in range(self._mpc._N):
            t_future = now + k * self._dt
            future_state = self._gait.query(t_future)
            contact_schedule.append(
                [future_state[leg]["contact"] for leg in _MPC_LEG_ORDER]
            )

        R_body = _euler_to_R(srbd_state[:3])
        foot_pos_world = np.zeros((4, 3))
        for i, leg in enumerate(_MPC_LEG_ORDER):
            joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
            foot_pos_world[i] = R_body @ np.array(forward_kinematics(leg, joints_leg))

        tau_list = [0.0] * 12
        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            tau_list = _build_stance_tau(grf, joint_targets, contact_now=contact_now)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc] solver failed: {exc}", throttle_duration_sec=2.0
            )

        v_world = R_body @ srbd_state[9:12]
        self._com_pos[:2] += v_world[:2] * self._dt

        kp_list = []
        kd_list = []
        for leg in _MPC_LEG_ORDER:
            in_contact = gait_state[leg]["contact"]
            scale_kp = self._kp_stance_scale if in_contact else self._kp_swing_scale
            scale_kd = self._kd_stance_scale if in_contact else self._kd_swing_scale
            for jname in _leg_joints(leg):
                kp_list.append(self._base_kp[jname] * scale_kp)
                kd_list.append(self._base_kd[jname] * scale_kd)

        return JointCommand(
            q=[joint_targets[n] for n in _YAML_JOINTS],
            dq=[dq_targets.get(n, 0.0) for n in _YAML_JOINTS],
            tau=tau_list,
            kp=kp_list,
            kd=kd_list,
        )

    def _tick(self) -> None:
        now = time.monotonic()

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
            stance_h = float(self.get_parameter("stance_height").value)
            if done and self._is_near(self._compute_stance_q(stance_h), _STANDUP_TOL) and self._is_settled():
                if not est_fresh:
                    self.get_logger().warn(
                        "[mpc] standup done but state_estimate stale — holding, waiting for estimate",
                        throttle_duration_sec=2.0,
                    )
                else:
                    self._phase = _PHASE_WALK
                    self._phase_start = now
                    self._gait.reset()
                    self.get_logger().info("[mpc] standup done → WALK")
            elif done and elapsed > float(self.get_parameter("ramp_duration").value) + 5.0:
                self._phase = _PHASE_WALK
                self._phase_start = now
                self._gait.reset()
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
