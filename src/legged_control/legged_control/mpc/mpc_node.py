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
from legged_control.mpc.srbd_mpc import SRBDMPC


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

# MPC leg ordering: FR=0, FL=1, RR=2, RL=3
_MPC_LEG_ORDER = ["FR", "FL", "RR", "RL"]


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

    def _publish(self, positions: list[float]) -> None:
        self._last_published = list(positions)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_YAML_JOINTS)
        msg.position = positions
        self._pub.publish(msg)

    def _standup_targets(self, elapsed: float) -> tuple[list[float], bool]:
        ramp = max(float(self.get_parameter("ramp_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / ramp)
        start = self._standup_start or self._q_default.tolist()
        targets = [(1.0 - alpha) * s + alpha * g
                   for s, g in zip(start, self._q_default.tolist())]
        return targets, elapsed >= ramp

    def _liedown_targets(self, elapsed: float) -> tuple[list[float], bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or self._q_default.tolist()
        goal  = self._initial_pos or [0.0] * 12
        targets = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        return targets, elapsed >= dur

    def _compute_mpc_joints(self, now: float) -> list[float]:
        """Run one MPC step and return 12 joint position targets."""
        stance_h = float(self.get_parameter("stance_height").value)
        step_h   = float(self.get_parameter("step_height").value)

        # Velocity threshold: hold stance when stopped
        moving = float(np.max(np.abs(self._cmd_vel))) >= _WALK_VEL_THRESH
        if not moving:
            if self._walking:
                # Transition: walk → stand; reset gait for clean restart next time
                self._gait.reset()
                self._prev_contact = {leg: True for leg in LEG_NAMES}
                self._lift_pos = {
                    leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
                }
                self._walking = False
            return list(self._q_default)

        if not self._walking:
            # Transition: stand → walk; reset gait phase and lift positions
            self._gait.reset()
            self._lift_pos = {
                leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
            }
            self._walking = True

        gait_state = self._gait.query(now)

        # Update lift-off positions on stance→swing transition
        for leg in LEG_NAMES:
            in_contact = gait_state[leg]["contact"]
            if self._prev_contact[leg] and not in_contact:
                # Just entered swing: record current foot position as lift-off
                joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))
                self._lift_pos[leg] = np.array(forward_kinematics(leg, joints_leg))
            self._prev_contact[leg] = in_contact

        # --- Foot positions in hip frame ---
        # Foot positions in world frame for MPC (relative to CoM)
        # For stance legs: keep at current FK position; for swing: trajectory
        joint_targets: dict[str, float] = {}

        for leg in LEG_NAMES:
            in_contact = gait_state[leg]["contact"]
            phase = gait_state[leg]["phase"]
            joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))

            if in_contact:
                # Stance: hold current joint angles (MPC will compute corrections via GRF,
                # but since we output positions, hold default stance foot position)
                p_foot = np.array([0.0, 0.0, -stance_h])
            else:
                # Swing: interpolate foot trajectory
                s = self._gait.swing_phase(leg, now)
                body_vel_xy = self._cmd_vel[:2]
                p_land = landing_target(
                    leg, body_vel_xy,
                    self._gait.period,
                    float(self.get_parameter("swing_ratio").value),
                    stance_h,
                )
                p_foot = swing_foot_position(s, self._lift_pos[leg], p_land, step_h)

            # IK: foot position in hip frame → joint angles
            preferred = joints_leg
            q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
            if q_leg is None:
                # IK failure: fall back to default
                q_leg = (
                    _DEFAULT_Q[f"{leg}_hip"],
                    _DEFAULT_Q[f"{leg}_thigh"],
                    _DEFAULT_Q[f"{leg}_calf"],
                )
            for jname, qval in zip(_leg_joints(leg), q_leg):
                joint_targets[jname] = float(qval)

        # --- MPC body pose correction for stance legs ---
        # Build current SRBD state
        srbd_state = _state_from_estimate(self._state_estimate, self._com_pos)
        # Desired state: level body at stance height, commanded velocity
        state_ref = np.array([
            0.0, 0.0, 0.0,           # desired rpy = level
            0.0, 0.0, stance_h,       # desired pos (XY relative, Z = height)
            0.0, 0.0, self._cmd_vel[2],  # desired ang_vel (yaw rate)
            self._cmd_vel[0], self._cmd_vel[1], 0.0,  # desired lin_vel
        ])

        contact_now = [gait_state[leg]["contact"] for leg in _MPC_LEG_ORDER]
        # Look-ahead contact schedule: query gait phase at each future step
        contact_schedule = []
        for k in range(self._mpc._N):
            t_future = now + k * self._dt
            future_state = self._gait.query(t_future)
            contact_schedule.append(
                [future_state[leg]["contact"] for leg in _MPC_LEG_ORDER]
            )

        # Foot positions relative to CoM (world frame, approximate as hip frame offsets)
        foot_pos_world = np.zeros((4, 3))
        for i, leg in enumerate(_MPC_LEG_ORDER):
            joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
            p_hip = np.array(forward_kinematics(leg, joints_leg))
            foot_pos_world[i] = p_hip  # approximation: hip ≈ CoM offset

        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            # Convert GRF → joint position correction via Jacobian transpose:
            #   τ = J^T * f_contact  (contact force in hip frame)
            #   Δq = τ / K_joint     (K_joint = effective joint stiffness, Nm/rad)
            # K_joint ≈ kp × gr² ≈ 20 Nm/rad for hip/thigh, calf same by design.
            K_joint = 20.0
            for i, leg in enumerate(_MPC_LEG_ORDER):
                if not contact_now[i]:
                    continue
                f_leg = grf[i * 3 : i * 3 + 3]          # contact force for this leg
                joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
                J = _numerical_jacobian(leg, joints_leg)  # 3×3, hip frame
                tau = J.T @ f_leg                          # joint torques (3,)
                dq = tau / K_joint                         # position correction (rad)
                dq = np.clip(dq, -0.05, 0.05)             # safety clamp
                for jname, delta in zip(_leg_joints(leg), dq):
                    joint_targets[jname] = float(joint_targets[jname] + delta)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc] solver failed: {exc}", throttle_duration_sec=2.0
            )

        # Integrate CoM velocity
        self._com_pos[:2] += srbd_state[9:11] * self._dt

        return [joint_targets[n] for n in _YAML_JOINTS]

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
            targets, done = self._standup_targets(elapsed)
            self._publish(targets)
            if done and self._is_near(self._q_default.tolist(), _STANDUP_TOL) and self._is_settled():
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
            targets = self._compute_mpc_joints(now)
            self._publish(targets)
            return

        if self._phase == _PHASE_LIEDOWN:
            targets, done = self._liedown_targets(elapsed)
            self._publish(targets)
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
