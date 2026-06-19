"""policy_node_sim — RL policy for Gazebo simulation.

Works entirely in URDF frame. No motor frame conversion, no robot.yaml.
All joint parameters come from legged_deploy/policies/README.md.

Observation (373 dims):
  [0:3]   base_lin_vel      from /state_estimate
  [3:6]   base_ang_vel      from /state_estimate
  [6:9]   projected_gravity from /state_estimate
  [9:12]  velocity_commands [vx, vy, omega_z]
  [12:24] joint_pos_rel     q_urdf - q_default  (sim order -> policy order)
  [24:36] joint_vel         dq_urdf             (sim order -> policy order)
  [36:48] last_action       previous action (policy order)
  [48:373] height_scan      325 floats, m

Action (12 dims, policy order):
  FR_hip / RL_hip negated first (Isaac Lab USD axis flip vs Gazebo URDF)
  q_target = q_default + action * scale  (URDF frame, clipped to soft limits)
"""

from __future__ import annotations

import numpy as np

# Sim joint order: matches Gazebo controller + gazebo_control_bridge output
_SIM_NAMES = [
    "FL_hip",   "FL_thigh", "FL_calf",
    "FR_hip",   "FR_thigh", "FR_calf",
    "RL_hip",   "RL_thigh", "RL_calf",
    "RR_hip",   "RR_thigh", "RR_calf",
]

# Policy joint order (legged_deploy README)
_POLICY_NAMES = [
    "FL_hip",  "FR_hip",   "FL_thigh", "FR_thigh",
    "FL_calf", "FR_calf",  "RL_hip",   "RR_hip",
    "RL_thigh", "RR_thigh", "RL_calf",  "RR_calf",
]

_SIM_TO_POLICY = [_SIM_NAMES.index(n) for n in _POLICY_NAMES]
_POLICY_TO_SIM = [_POLICY_NAMES.index(n) for n in _SIM_NAMES]

# Isaac Lab USD export loses hip axis direction; FR_hip and RL_hip are flipped
# relative to the Gazebo URDF — negate their actions before applying.
_HIP_FLIP_SIM = [_SIM_NAMES.index("FR_hip"), _SIM_NAMES.index("RL_hip")]


def _jtype(name: str) -> str:
    return name.split("_")[1]  # "hip" / "thigh" / "calf"


_Q_DEFAULT  = {"hip": 0.0,   "thigh": 0.7,   "calf": -1.2}
_SCALE      = {"hip": 0.15,  "thigh": 0.20,  "calf": 0.15}
_SOFT_MIN   = {"hip": -0.450, "thigh": -1.480, "calf": -2.295}
_SOFT_MAX   = {"hip":  0.450, "thigh":  0.680, "calf": -0.405}

_Q_DEF = np.array([_Q_DEFAULT[_jtype(n)] for n in _SIM_NAMES], dtype=np.float32)
_SCL   = np.array([_SCALE[_jtype(n)]     for n in _SIM_NAMES], dtype=np.float32)
_SMIN  = np.array([_SOFT_MIN[_jtype(n)]  for n in _SIM_NAMES], dtype=np.float32)
_SMAX  = np.array([_SOFT_MAX[_jtype(n)]  for n in _SIM_NAMES], dtype=np.float32)


def _assemble_obs(
    state_est: np.ndarray,
    cmd_vel: tuple[float, float, float],
    q_sim: np.ndarray,
    dq_sim: np.ndarray,
    last_action: np.ndarray,
    height_scan: np.ndarray,
) -> np.ndarray:
    q_rel = (q_sim - _Q_DEF)[_SIM_TO_POLICY]
    dq    = dq_sim[_SIM_TO_POLICY]
    return np.concatenate([
        state_est[:9],
        np.array(cmd_vel, dtype=np.float32),
        q_rel,
        dq,
        last_action,
        height_scan,
    ]).astype(np.float32)


def _decode_action(action_policy: np.ndarray) -> np.ndarray:
    action_sim = action_policy[_POLICY_TO_SIM].copy()
    for i in _HIP_FLIP_SIM:
        action_sim[i] *= -1.0
    q_target = _Q_DEF + action_sim * _SCL
    return np.clip(q_target, _SMIN, _SMAX)


# ── ROS node ──────────────────────────────────────────────────────────────────

import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from legged_control.kinematics import _smoothstep

_PASSIVE  = "PASSIVE"
_STANDUP  = "STANDUP"
_WAIT     = "WAIT"
_POLICY   = "POLICY"
_LIEDOWN  = "LIEDOWN"

_STANDUP_TOL     = 0.05
_LIEDOWN_TOL     = 0.05
_VEL_SETTLED     = 0.05
_LIEDOWN_TIMEOUT = 3.0


class PolicyNodeSim(Node):
    def __init__(self) -> None:
        super().__init__("policy_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("ramp_duration", 2.0)
        self.declare_parameter("lie_down_duration", 2.0)

        model_path = str(self.get_parameter("model_path").value or "").strip()
        if not model_path:
            share = get_package_share_directory("legged_control")
            model_path = os.path.join(share, "models", "policy.pt")

        self._policy = None
        if os.path.exists(model_path):
            try:
                import torch
                self._policy = torch.jit.load(model_path)
                self._policy.eval()
                self.get_logger().info(f"[policy_sim] model loaded: {model_path}")
            except Exception as e:
                self.get_logger().error(f"[policy_sim] load failed: {e}")
        else:
            self.get_logger().warn(f"[policy_sim] model not found: {model_path}")

        self._phase = _PASSIVE
        self._phase_start: float | None = None
        self._stand_requested = False
        self._lie_down_start: list[float] | None = None
        self._last_published: list[float] | None = None
        self._last_action = np.zeros(12, dtype=np.float32)

        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._state_est  = np.zeros(9,   dtype=np.float32)
        self._height_scan = np.zeros(325, dtype=np.float32)
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._joint_state_seen = False

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(
            Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(
            Float32MultiArray, "/height_scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/posture_command", self._on_posture, 10)
        self.create_timer(1.0 / 50.0, self._tick)

        self.get_logger().info(
            f"policy_node_sim ready  "
            f"model={'loaded' if self._policy else 'NOT LOADED'}"
        )

    # ── subscribers ──────────────────────────────────────────────────────────

    def _on_joints(self, msg: JointState) -> None:
        self._joint_state_seen = True
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_est = np.array(msg.data[:9], dtype=np.float32)

    def _on_scan(self, msg: Float32MultiArray) -> None:
        self._height_scan = np.array(msg.data[:325], dtype=np.float32)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        )

    def _on_posture(self, msg: Bool) -> None:
        if bool(msg.data):
            if self._phase == _PASSIVE:
                self._stand_requested = True
        else:
            if self._phase in (_WAIT, _POLICY, _STANDUP):
                self._phase = _LIEDOWN
                self._phase_start = time.monotonic()
                self._lie_down_start = list(
                    self._last_published or _Q_DEF.tolist()
                )

    # ── helpers ───────────────────────────────────────────────────────────────

    def _current_pos(self) -> list[float] | None:
        vals = [self._joint_pos.get(n) for n in _SIM_NAMES]
        return None if any(v is None for v in vals) else [float(v) for v in vals]

    def _current_vel(self) -> list[float] | None:
        vals = [self._joint_vel.get(n) for n in _SIM_NAMES]
        return None if any(v is None for v in vals) else [float(v) for v in vals]

    def _is_near(self, targets: list[float], tol: float) -> bool:
        pos = self._current_pos()
        return pos is not None and all(
            abs(p - t) <= tol for p, t in zip(pos, targets)
        )

    def _is_settled(self) -> bool:
        vel = self._current_vel()
        return vel is not None and all(abs(v) <= _VEL_SETTLED for v in vel)

    def _publish(self, positions: list[float]) -> None:
        self._last_published = list(positions)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_SIM_NAMES)
        msg.position = positions
        self._pub.publish(msg)

    def _standup_targets(self, elapsed: float) -> tuple[list[float], bool]:
        ramp = max(float(self.get_parameter("ramp_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / ramp)
        targets = [alpha * float(q) for q in _Q_DEF]
        return targets, elapsed >= ramp

    def _liedown_targets(self, elapsed: float) -> tuple[list[float], bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or _Q_DEF.tolist()
        return [(1.0 - alpha) * s for s in start], elapsed >= dur

    def _run_inference(self) -> list[float]:
        pos = self._current_pos()
        vel = self._current_vel()
        if pos is None or vel is None or self._policy is None:
            return _Q_DEF.tolist()

        obs = _assemble_obs(
            self._state_est,
            self._cmd_vel,
            np.array(pos, dtype=np.float32),
            np.array(vel, dtype=np.float32),
            self._last_action,
            self._height_scan,
        )
        try:
            import torch
            with torch.inference_mode():
                action = (
                    self._policy(torch.from_numpy(obs).unsqueeze(0))
                    .squeeze(0)
                    .numpy()
                )
        except Exception as e:
            self.get_logger().error(
                f"[policy_sim] inference error: {e}",
                throttle_duration_sec=1.0,
            )
            return _Q_DEF.tolist()

        self._last_action = action.copy()
        return _decode_action(action).tolist()

    # ── main loop ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()

        if self._phase == _PASSIVE:
            if self._stand_requested:
                self._phase = _STANDUP
                self._phase_start = now
                self._last_published = None
                self._last_action = np.zeros(12, dtype=np.float32)
                self._stand_requested = False
                self.get_logger().info("[policy_sim] posture=true -> STANDUP")
            return

        if self._phase_start is None:
            self._phase_start = now
        elapsed = now - self._phase_start

        if self._phase == _STANDUP:
            targets, done = self._standup_targets(elapsed)
            self._publish(targets)
            ramp = float(self.get_parameter("ramp_duration").value)
            if done and self._is_near(_Q_DEF.tolist(), _STANDUP_TOL) and self._is_settled():
                self._phase = _WAIT
                self._phase_start = now
                self.get_logger().info("[policy_sim] standup complete -> WAIT")
            elif done and elapsed > ramp + 5.0:
                self._phase = _WAIT
                self._phase_start = now
                self.get_logger().warn("[policy_sim] standup timeout -> WAIT")
            return

        if self._phase == _WAIT:
            self._publish(_Q_DEF.tolist())
            if self._joint_state_seen and any(abs(v) > 1e-4 for v in self._cmd_vel):
                self._phase = _POLICY
                self._phase_start = now
                self.get_logger().info("[policy_sim] cmd_vel -> POLICY")
            return

        if self._phase == _POLICY:
            if all(abs(v) <= 1e-4 for v in self._cmd_vel):
                self._phase = _WAIT
                self._phase_start = now
                return
            self._publish(self._run_inference())
            return

        if self._phase == _LIEDOWN:
            targets, done = self._liedown_targets(elapsed)
            self._publish(targets)
            dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
            near_zero = self._is_near([0.0] * 12, _LIEDOWN_TOL) and self._is_settled()
            timed_out = done and elapsed > dur + _LIEDOWN_TIMEOUT
            if (done and near_zero) or timed_out:
                self._phase = _PASSIVE
                self._phase_start = None
                self._last_action = np.zeros(12, dtype=np.float32)
                if timed_out and not near_zero:
                    self.get_logger().warn("[policy_sim] liedown timeout -> PASSIVE")
                else:
                    self.get_logger().info("[policy_sim] liedown complete -> PASSIVE")
            return


def main() -> None:
    rclpy.init()
    node = PolicyNodeSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
