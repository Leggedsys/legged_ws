"""policy_node_sim — B+C RL policy for Gazebo simulation (46-dim frame-stacked).

Works entirely in URDF frame. The Gazebo model is the dog_urdf training URDF, so:
  * sim joint order == policy joint order (no reordering)
  * NO hip_sign_flip (that correction only applies to the real-robot URDF)
Shared joint parameters (default_q, action_scale, soft limits) are loaded from
config/policy.yaml so sim and real stay in sync. obs scaling/layout matches the
real obs_assembler (46-dim single frame, no base_lin_vel), stacked 3 frames -> 138.

Single frame (46 dims, scaled — see processing/obs_assembler.py):
  [0:3]   base_ang_vel      * 0.25  from /state_estimate[3:6]
  [3:6]   projected_gravity * 1.0   from /state_estimate[6:9]
  [6:9]   velocity_commands * (2.0, 2.0, 0.25)
  [9]     height_command    * 1.0 (raw) from /height_command
  [10:22] joint_pos_rel     * 1.0   q_sim - q_default (sim == policy order)
  [22:34] joint_vel         * 0.05  dq_sim
  [34:46] last_action       previous action (policy order)

Action (12 dims, policy order):
  q_target = q_default + action * scale  (URDF frame, clipped to soft limits)
"""

from __future__ import annotations

import numpy as np

from legged_control.processing.obs_assembler import (
    SINGLE_OBS_DIM,
    _ANG_VEL_SCALE,
    _CMD_SCALE,
    _DOF_POS_SCALE,
    _DOF_VEL_SCALE,
)

STACK_FRAMES = 3
POLICY_INPUT_DIM = SINGLE_OBS_DIM * STACK_FRAMES  # 138

# Sim joint order == policy joint order (Gazebo controller + gazebo_control_bridge).
_SIM_NAMES = [
    "FL_hip",   "FL_thigh", "FL_calf",
    "FR_hip",   "FR_thigh", "FR_calf",
    "RL_hip",   "RL_thigh", "RL_calf",
    "RR_hip",   "RR_thigh", "RR_calf",
]

_DEFAULT_HEIGHT_CMD = 0.22


def _jtype(name: str) -> str:
    return name.split("_")[1]  # "hip" / "thigh" / "calf"


def _assemble_obs(
    state_est: np.ndarray,
    cmd_vel: tuple[float, float, float],
    height_cmd: float,
    q_sim: np.ndarray,
    dq_sim: np.ndarray,
    q_default: np.ndarray,
    last_action: np.ndarray,
    sign_flip_idx: list[int] | None = None,
) -> np.ndarray:
    state_est = np.asarray(state_est, dtype=np.float32)
    # state_est[0:3] is base_lin_vel — intentionally NOT used (B+C actor obs).
    ang_vel = state_est[3:6] * _ANG_VEL_SCALE
    proj_grav = state_est[6:9]
    cmd = np.asarray(cmd_vel, dtype=np.float32) * _CMD_SCALE

    # sim order == policy order, so no reordering is needed.
    q_rel = (np.asarray(q_sim, dtype=np.float32)
             - np.asarray(q_default, dtype=np.float32)) * _DOF_POS_SCALE
    dq = np.asarray(dq_sim, dtype=np.float32) * _DOF_VEL_SCALE
    if sign_flip_idx:
        for i in sign_flip_idx:
            q_rel[i] *= -1.0
            dq[i] *= -1.0

    return np.clip(np.concatenate([
        ang_vel,
        proj_grav,
        cmd,
        np.array([height_cmd], dtype=np.float32),
        q_rel,
        dq,
        np.asarray(last_action, dtype=np.float32),
    ]), -100.0, 100.0).astype(np.float32)


def _decode_action(
    action_policy: np.ndarray,
    q_default: np.ndarray,
    scale: np.ndarray,
    sign_flip_idx: list[int],
    soft_min: np.ndarray,
    soft_max: np.ndarray,
) -> np.ndarray:
    action = np.asarray(action_policy, dtype=np.float32).copy()
    for i in sign_flip_idx:
        action[i] *= -1.0
    # sim order == policy order, so no reordering is needed.
    q_target = np.asarray(q_default, dtype=np.float32) + action * np.asarray(scale, dtype=np.float32)
    return np.clip(q_target, soft_min, soft_max)


# ── ROS node ──────────────────────────────────────────────────────────────────

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray

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

        share = get_package_share_directory("legged_control")
        pcfg = self._load_policy_cfg(share)
        self._q_default = self._build_array(pcfg.get("joint_default_q_urdf", {}), 0.0)
        self._scale = self._build_array(pcfg.get("action_scale", {}), 0.25)
        self._soft_min, self._soft_max = self._build_soft_limits(pcfg)
        # Gazebo uses the dog_urdf training URDF directly → no hip sign flip.
        self._sign_flip_idx: list[int] = []

        model_path = str(self.get_parameter("model_path").value or "").strip()
        if not model_path:
            model_path = str(pcfg.get("model_path", "") or "")
            if model_path.startswith("__package__/"):
                model_path = os.path.join(share, model_path[len("__package__/"):])
        if not model_path:
            model_path = os.path.join(share, "models", "08_bc_mujoco_recovered_policy.pt")

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
        self._obs_history = np.zeros(POLICY_INPUT_DIM, dtype=np.float32)  # 3×46 frame stack

        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._state_est  = np.zeros(9, dtype=np.float32)
        self._height_cmd = _DEFAULT_HEIGHT_CMD
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._joint_state_seen = False

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(
            Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32, "/height_command", self._on_height, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/posture_command", self._on_posture, 10)
        self.create_timer(1.0 / 50.0, self._tick)

        self.get_logger().info(
            f"policy_node_sim ready  "
            f"model={'loaded' if self._policy else 'NOT LOADED'}  obs_dim={POLICY_INPUT_DIM}"
        )

    # ── config ─────────────────────────────────────────────────────────────────

    def _load_policy_cfg(self, share: str) -> dict:
        try:
            with open(os.path.join(share, "config", "policy.yaml")) as f:
                return yaml.safe_load(f).get("policy", {})
        except Exception:
            return {}

    def _build_array(self, m: dict, default: float) -> np.ndarray:
        return np.array([float(m.get(n, default)) for n in _SIM_NAMES], dtype=np.float32)

    def _build_soft_limits(self, pcfg: dict) -> tuple[np.ndarray, np.ndarray]:
        lims = pcfg.get("joint_soft_limits", {})
        tm = {
            "hip":   lims.get("hip",   [-0.45, 0.45]),
            "thigh": lims.get("thigh", [-1.709, 1.729]),
            "calf":  lims.get("calf",  [-2.674, -0.406]),
        }
        q_min = np.array([float(tm[_jtype(n)][0]) for n in _SIM_NAMES], dtype=np.float32)
        q_max = np.array([float(tm[_jtype(n)][1]) for n in _SIM_NAMES], dtype=np.float32)
        return q_min, q_max

    # ── subscribers ──────────────────────────────────────────────────────────

    def _on_joints(self, msg: JointState) -> None:
        self._joint_state_seen = True
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_est = np.array(msg.data[:9], dtype=np.float32)

    def _on_height(self, msg: Float32) -> None:
        self._height_cmd = float(msg.data)

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
                    self._last_published or self._q_default.tolist()
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
        targets = [alpha * float(q) for q in self._q_default]
        return targets, elapsed >= ramp

    def _liedown_targets(self, elapsed: float) -> tuple[list[float], bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or self._q_default.tolist()
        return [(1.0 - alpha) * s for s in start], elapsed >= dur

    def _run_inference(self) -> list[float]:
        pos = self._current_pos()
        vel = self._current_vel()
        if pos is None or vel is None or self._policy is None:
            return self._q_default.tolist()

        single = _assemble_obs(
            self._state_est,
            self._cmd_vel,
            self._height_cmd,
            np.array(pos, dtype=np.float32),
            np.array(vel, dtype=np.float32),
            self._q_default,
            self._last_action,
            self._sign_flip_idx,
        )
        # frame stacking: shift left one frame, append newest at the end
        self._obs_history[:-SINGLE_OBS_DIM] = self._obs_history[SINGLE_OBS_DIM:]
        self._obs_history[-SINGLE_OBS_DIM:] = single
        try:
            import torch
            with torch.inference_mode():
                action = (
                    self._policy(torch.from_numpy(self._obs_history).unsqueeze(0))
                    .squeeze(0)
                    .numpy()
                )
        except Exception as e:
            self.get_logger().error(
                f"[policy_sim] inference error: {e}",
                throttle_duration_sec=1.0,
            )
            return self._q_default.tolist()

        self._last_action = action.copy()
        return _decode_action(
            action, self._q_default, self._scale,
            self._sign_flip_idx, self._soft_min, self._soft_max,
        ).tolist()

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
            if done and self._is_near(self._q_default.tolist(), _STANDUP_TOL) and self._is_settled():
                self._phase = _WAIT
                self._phase_start = now
                self.get_logger().info("[policy_sim] standup complete -> WAIT")
            elif done and elapsed > ramp + 5.0:
                self._phase = _WAIT
                self._phase_start = now
                self.get_logger().warn("[policy_sim] standup timeout -> WAIT")
            return

        if self._phase == _WAIT:
            self._publish(self._q_default.tolist())
            if self._joint_state_seen and any(abs(v) > 1e-4 for v in self._cmd_vel):
                self._phase = _POLICY
                self._phase_start = now
                self._obs_history[:] = 0.0  # reset frame stack on POLICY entry
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
