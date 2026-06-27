"""policy_node — RL policy deployment node.

Runs a TorchScript policy at 50 Hz with the same PASSIVE->STANDUP->WAIT->POLICY->LIE_DOWN
state machine as gait_node. The POLICY phase replaces IK trot with neural-network inference.

B+C policy: actor input is a 3-frame stack of a 46-dim single frame (138 dims).
Single frame (no base_lin_vel; see processing/obs_assembler.py and
docs/deployment_guide.md):
  [0:3]   base_ang_vel      * 0.25  from /state_estimate[3:6]
  [3:6]   projected_gravity * 1.0   from /state_estimate[6:9]
  [6:9]   velocity_commands * (2.0, 2.0, 0.25)  [vx, vy, omega_z] from /cmd_vel
  [9]     height_command    * 1.0 (raw) from /height_command
  [10:22] joint_pos_rel     * 1.0   q_urdf - q_default_urdf (yaml -> policy order)
  [22:34] joint_vel         * 0.05  dq_urdf (yaml -> policy order)
  [34:46] last_action       previous raw policy output (policy order)

Frame stack (built here each policy step): obs_history = [oldest|mid|newest],
shifted left and the newest 46-dim frame appended; zeroed on POLICY entry.

Action: 12-dim (policy order) joint position residuals.
  q_target_urdf = q_default_urdf + sign_flip * action * scale
"""

from __future__ import annotations

import numpy as np

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

_POLICY_JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]

_POLICY_TO_YAML = [
    _POLICY_JOINT_NAMES.index(name) for name in _YAML_JOINT_NAMES
]


def _reorder_policy_to_yaml(policy_vec: np.ndarray) -> np.ndarray:
    return policy_vec[_POLICY_TO_YAML]


def _decode_action(
    action_policy: np.ndarray,
    q_default_urdf_yaml: np.ndarray,
    action_scale_yaml: np.ndarray,
    sign_flip_policy_idx: list[int],
) -> np.ndarray:
    action = action_policy.copy()
    for idx in sign_flip_policy_idx:
        action[idx] *= -1.0

    action_yaml = _reorder_policy_to_yaml(action)
    q_target_urdf = q_default_urdf_yaml + action_yaml * action_scale_yaml
    return q_target_urdf


def _is_fresh(age: float | None, max_age: float) -> bool:
    """True if a signal received `age` seconds ago is still usable."""
    return age is not None and age <= max_age


def _inputs_usable(
    obs_age: float | None,
    est_age: float | None,
    est_health: float,
    max_age: float,
) -> bool:
    """Policy may run only with fresh obs and a fresh, healthy state estimate."""
    return (
        _is_fresh(obs_age, max_age)
        and _is_fresh(est_age, max_age)
        and est_health >= 0.5
    )


# ── ROS node ──────────────────────────────────────────────────────────────────

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.parameter
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from legged_control.kinematics import _smoothstep

# ── obs validation (sanity bounds on the scaled 46-dim single frame) ─────────────
# Ranges are conservative sanity gates, not tight training 2σ bounds. Command and
# height bounds come from the dog_urdf command ranges (scaled); velocity/joint
# bounds are loose physical limits. Re-tighten from policy rollout stats if desired.

from legged_control.processing.obs_assembler import SINGLE_OBS_DIM

STACK_FRAMES = 3
POLICY_INPUT_DIM = SINGLE_OBS_DIM * STACK_FRAMES  # 138

_OBS_CHECKS = {
    "base_ang_vel":  [("wx", 0, -1.5, 1.5), ("wy", 1, -1.5, 1.5), ("wz", 2, -1.5, 1.5)],
    "proj_grav":     [("gx", 3, -1.05, 1.05), ("gy", 4, -1.05, 1.05), ("gz", 5, -1.05, 0.20)],
    "vel_cmd":       [("vx", 6, -3.1, 3.1), ("vy", 7, -1.1, 1.1), ("wz", 8, -0.27, 0.27)],
    "height_cmd":    [("h", 9, 0.10, 0.35)],
    "joint_pos":     [(f"pos_{_POLICY_JOINT_NAMES[i]}", 10 + i, -1.5, 1.5) for i in range(12)],
    "joint_vel":     [(f"vel_{_POLICY_JOINT_NAMES[i]}", 22 + i, -1.0, 1.0) for i in range(12)],
}


def _validate_obs(obs: np.ndarray) -> list[str]:
    """Validate a SINGLE 46-dim frame (not the stacked 138 vector)."""
    if obs.shape[0] != SINGLE_OBS_DIM:
        return [f"obs_dim={obs.shape[0]}!={SINGLE_OBS_DIM}"]
    bad = []
    if not bool(np.all(np.isfinite(obs))):
        bad.append("non_finite")
    for _group_name, checks in _OBS_CHECKS.items():
        for label, idx, lo, hi in checks:
            v = float(obs[idx])
            if not (lo <= v <= hi):
                bad.append(f"{label}={v:+.3f}[{lo:+.2f},{hi:+.2f}]")
    return bad


_PHASE_PASSIVE = "PASSIVE"
_PHASE_STANDUP = "STANDUP"
_PHASE_WAIT    = "WAIT"
_PHASE_POLICY  = "POLICY"
_PHASE_LIEDOWN = "LIEDOWN"
_PHASE_FAULT   = "FAULT"

_STANDUP_TOL  = 0.15
_LIEDOWN_TOL  = 0.05
_VEL_SETTLED  = 0.05
_LIEDOWN_TIMEOUT = 3.0  # seconds past lie_down_duration before forcing PASSIVE
_RAW_LIMIT    = 20.0    # raw_action divergence threshold
_RAW_FAULT_N  = 3       # consecutive frames above limit → FAULT
_INPUT_MAX_AGE = 0.1    # s — obs / state_estimate older than this is unusable
_ZERO_CMD_EPS  = 1e-3   # |cmd_vel| component below this counts as "no command"
_ZERO_CMD_STOP_S = 0.3  # sustained zero cmd_vel before holding default pose (stop)


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_node")

        self.declare_parameter("config_path", "")
        self.declare_parameter("policy_config_path", "")
        self.declare_parameter("model_path", "")

        cfg = self._load_robot_cfg()
        policy_cfg = self._load_policy_cfg()

        self._joint_names_yaml = [j["name"] for j in cfg["joints"]]

        self._q_default_urdf = self._build_q_default_urdf(policy_cfg)
        self._action_scale = self._build_action_scale(policy_cfg)
        self._sign_flip_policy_idx = self._build_sign_flip(policy_cfg)

        control_cfg = cfg["control"]
        standup_cfg = cfg.get("standup", {})
        kp = float(control_cfg["kp"])
        kd = float(control_cfg["kd"])
        self.declare_parameter("kp", kp)
        self.declare_parameter("kd", kd)
        self.declare_parameter("ramp_duration", float(standup_cfg.get("ramp_duration", 8.0)))
        self.declare_parameter("lie_down_duration", float(standup_cfg.get("lie_down_duration", 2.0)))
        self.declare_parameter("policy_dry_run", False)
        # policy_dry_run=True: POLICY phase runs inference but publishes q_default (safe)
        # policy_dry_run=False: POLICY phase publishes decoded action to motors

        loop_hz = float(control_cfg.get("gait_hz", 50.0))
        self._dt = 1.0 / loop_hz

        model_path = str(self.get_parameter("model_path").value or "").strip()
        if not model_path:
            model_path = str(policy_cfg.get("model_path", "") or "")
        if model_path.startswith("__package__/"):
            share = get_package_share_directory("legged_control")
            model_path = os.path.join(share, model_path[len("__package__/"):])
        self._policy = None
        self._model_path = ""
        if model_path:
            try:
                import torch
                self._policy = torch.jit.load(model_path)
                self._policy.eval()
                self._model_path = model_path
                self.get_logger().info(f"[policy] loaded model: {model_path}")
            except Exception as e:
                self.get_logger().error(f"[policy] failed to load model {model_path}: {e}")

        self._phase = _PHASE_PASSIVE
        self._phase_start: float | None = None
        self._stand_requested = False
        self._lie_down_start: list[float] | None = None
        self._standup_start: list[float] | None = None  # joint pos at STANDUP entry
        self._initial_pos: list[float] | None = None     # first joint pos seen in PASSIVE
        self._last_published: list[float] | None = None
        self._last_action = np.zeros(12, dtype=np.float32)
        self._passive_broadcast = False
        self._fault_broadcast = False
        self._raw_high_count = 0
        self._last_cmd_move_time: float = 0.0
        self._cmd_stopped = False

        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._joint_state_seen = False

        self._gain_clients = [
            self.create_client(SetParameters, "/motor_bus_front/set_parameters"),
            self.create_client(SetParameters, "/motor_bus_rear/set_parameters"),
        ]

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self._pub_raw = self.create_publisher(Float32MultiArray, "/raw_policy_action", 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/observation", self._on_observation, 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state_estimate, 10)
        self.create_subscription(Bool, "/posture_command", self._on_posture, 10)
        self.add_on_set_parameters_callback(self._on_gains_changed)
        self.create_timer(self._dt, self._tick)

        self._latest_obs: np.ndarray | None = None
        self._obs_history = np.zeros(POLICY_INPUT_DIM, dtype=np.float32)  # 3×46 frame stack
        self._last_stacked_stamp: float | None = None
        self._obs_stamp: float | None = None
        self._est_stamp: float | None = None
        self._est_health: float = 0.0

        self.get_logger().info(
            f"policy_node ready — {loop_hz:.0f} Hz  "
            f"model={'loaded' if self._policy else 'NOT LOADED'}  "
            f"kp={kp}  kd={kd}"
        )

    def _load_robot_cfg(self) -> dict:
        share = get_package_share_directory("legged_control")
        path = str(self.get_parameter("config_path").value or "").strip()
        if not path:
            path = os.path.join(share, "config", "robot.yaml")
        with open(path) as f:
            return yaml.safe_load(f)

    def _load_policy_cfg(self) -> dict:
        share = get_package_share_directory("legged_control")
        path = str(self.get_parameter("policy_config_path").value or "").strip()
        if not path:
            path = os.path.join(share, "config", "policy.yaml")
        with open(path) as f:
            return yaml.safe_load(f).get("policy", {})

    def _build_q_default_urdf(self, pcfg: dict) -> np.ndarray:
        m = pcfg.get("joint_default_q_urdf", {})
        return np.array([float(m.get(n, 0.0)) for n in _YAML_JOINT_NAMES], dtype=np.float32)

    def _build_action_scale(self, pcfg: dict) -> np.ndarray:
        m = pcfg.get("action_scale", {})
        return np.array([float(m.get(n, 0.1)) for n in _YAML_JOINT_NAMES], dtype=np.float32)

    def _build_sign_flip(self, pcfg: dict) -> list[int]:
        flip_names = pcfg.get("hip_sign_flip", [])
        result = []
        for name in flip_names:
            if name in _POLICY_JOINT_NAMES:
                result.append(_POLICY_JOINT_NAMES.index(name))
        return result

    def _reset_policy(self) -> None:
        """Reload model to reset GRU hidden state."""
        if not self._model_path:
            return
        try:
            import torch
            self._policy = torch.jit.load(self._model_path)
            self._policy.eval()
        except Exception as e:
            self.get_logger().error(f"[policy] failed to reset model: {e}")

    def _on_joints(self, msg: JointState) -> None:
        self._joint_state_seen = True
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _on_observation(self, msg: Float32MultiArray) -> None:
        self._latest_obs = np.array(msg.data[:SINGLE_OBS_DIM], dtype=np.float32)
        self._obs_stamp = time.monotonic()

    def _on_state_estimate(self, msg: Float32MultiArray) -> None:
        # data[9] (if present) = state-estimator health flag (1.0 ok / 0.0 not ready)
        self._est_stamp = time.monotonic()
        self._est_health = float(msg.data[9]) if len(msg.data) > 9 else 1.0

    def _inputs_ok(self) -> bool:
        now = time.monotonic()
        obs_age = None if self._obs_stamp is None else now - self._obs_stamp
        est_age = None if self._est_stamp is None else now - self._est_stamp
        return _inputs_usable(obs_age, est_age, self._est_health, _INPUT_MAX_AGE)

    def _on_posture(self, msg: Bool) -> None:
        if self._phase == _PHASE_FAULT:
            return
        if bool(msg.data):
            if self._phase == _PHASE_PASSIVE:
                self._stand_requested = True
        else:
            if self._phase in (_PHASE_WAIT, _PHASE_POLICY, _PHASE_STANDUP):
                self._phase = _PHASE_LIEDOWN
                self._phase_start = time.monotonic()
                self._lie_down_start = list(self._last_published or self._q_default_urdf.tolist())

    def _broadcast_gains(self, kp: float, kd: float) -> bool:
        """Send kp/kd to both motor_bus nodes. Returns True only if every gain
        service was ready and the request was issued (so callers can retry)."""
        if not all(c.service_is_ready() for c in self._gain_clients):
            return False
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name="kp", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp))),
            Parameter(name="kd", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd))),
        ]
        for client in self._gain_clients:
            client.call_async(req)
        return True

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        new_kp = next((p.value for p in params if p.name == "kp"), None)
        new_kd = next((p.value for p in params if p.name == "kd"), None)
        if new_kp is not None or new_kd is not None:
            kp = new_kp if new_kp is not None else self.get_parameter("kp").value
            kd = new_kd if new_kd is not None else self.get_parameter("kd").value
            self._broadcast_gains(float(kp), float(kd))
        return SetParametersResult(successful=True)

    def _current_pos(self) -> list[float] | None:
        vals = [self._joint_pos.get(n) for n in self._joint_names_yaml]
        if any(v is None for v in vals):
            return None
        return [float(v) for v in vals]

    def _current_vel(self) -> list[float] | None:
        vals = [self._joint_vel.get(n) for n in self._joint_names_yaml]
        if any(v is None for v in vals):
            return None
        return [float(v) for v in vals]

    def _is_near(self, targets: list[float], tol: float) -> bool:
        pos = self._current_pos()
        if pos is None:
            return False
        return all(abs(p - t) <= tol for p, t in zip(pos, targets))

    def _is_settled(self) -> bool:
        vel = self._current_vel()
        if vel is None:
            return False
        return all(abs(v) <= _VEL_SETTLED for v in vel)

    def _publish(self, positions: list[float]) -> None:
        self._last_published = list(positions)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names_yaml)
        msg.position = positions
        self._pub.publish(msg)

    def _standup_targets(self, elapsed: float) -> tuple[list[float], bool]:
        ramp = max(float(self.get_parameter("ramp_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / ramp)
        done = elapsed >= ramp
        start = self._standup_start or self._q_default_urdf.tolist()
        goal = self._q_default_urdf.tolist()
        targets = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        return targets, done

    def _liedown_targets(self, elapsed: float) -> tuple[list[float], bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or self._q_default_urdf.tolist()
        goal = self._initial_pos or [0.0] * 12
        targets = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
        return targets, elapsed >= dur

    def _run_inference(self) -> list[float]:
        if self._latest_obs is None or self._policy is None:
            return self._q_default_urdf.tolist()

        single = self._latest_obs.copy()
        # authoritative last_action (our previous output) in the newest frame
        single[34:46] = self._last_action
        bad = _validate_obs(single)
        if bad:
            self.get_logger().warn(
                f"obs anomaly: {' | '.join(bad[:6])}"
                + (f" ...+{len(bad)-6}" if len(bad) > 6 else ""),
                throttle_duration_sec=3.0,
            )
        # frame stacking: advance the stack only when a genuinely new single frame
        # arrived (obs_assembler runs on its own timer; gating avoids pushing a
        # duplicate or skipping a frame in the 3-frame history).
        if self._obs_stamp != self._last_stacked_stamp:
            self._obs_history[:-SINGLE_OBS_DIM] = self._obs_history[SINGLE_OBS_DIM:]
            self._obs_history[-SINGLE_OBS_DIM:] = single
            self._last_stacked_stamp = self._obs_stamp
        else:
            # no new frame this tick — refresh only the newest frame's last_action
            self._obs_history[-SINGLE_OBS_DIM:] = single
        try:
            import torch
            with torch.inference_mode():
                obs_t = torch.from_numpy(self._obs_history).unsqueeze(0)
                action = self._policy(obs_t).squeeze(0).numpy()
        except Exception as e:
            self.get_logger().error(f"[policy] inference error: {e}", throttle_duration_sec=1.0)
            return self._q_default_urdf.tolist()

        self._last_action = action.copy()
        self._pub_raw.publish(Float32MultiArray(data=action.tolist()))
        self.get_logger().info(
            f"[policy] raw_action[max={np.max(np.abs(action)):+.1f}]  "
            f"cmd_vel={[f'{x:+.2f}' for x in single[6:9]]}  "
            f"h={single[9]:.3f}",
            throttle_duration_sec=1.0,
        )
        q_urdf = _decode_action(
            action,
            self._q_default_urdf,
            self._action_scale,
            self._sign_flip_policy_idx,
        )
        return q_urdf.tolist()

    def _tick(self) -> None:
        now = time.monotonic()

        if self._phase == _PHASE_PASSIVE:
            if self._stand_requested:
                # Only enter STANDUP once the gains are actually delivered, else
                # the motors would stay passive and never stand up.
                if not self._broadcast_gains(
                    float(self.get_parameter("kp").value),
                    float(self.get_parameter("kd").value),
                ):
                    self.get_logger().warn(
                        "[policy] waiting for motor_bus param services before STANDUP",
                        throttle_duration_sec=2.0,
                    )
                    return
                self._phase = _PHASE_STANDUP
                self._phase_start = now
                snapshot = list(self._current_pos() or self._q_default_urdf.tolist())
                self._standup_start = snapshot
                if self._initial_pos is None:
                    self._initial_pos = snapshot
                self._last_published = None
                self._last_action = np.zeros(12, dtype=np.float32)
                self._stand_requested = False
                self._passive_broadcast = False
                self.get_logger().info("[policy] posture=true -> STANDUP")
            else:
                # Keep retrying the passive (kp=kd=0) broadcast until it lands,
                # so the motors never sit at launch-time stiffness.
                if not self._passive_broadcast:
                    if self._broadcast_gains(0.0, 0.0):
                        self._passive_broadcast = True
                    else:
                        self.get_logger().warn(
                            "[policy] waiting for motor_bus param services to go passive",
                            throttle_duration_sec=2.0,
                        )
            return

        if self._phase_start is None:
            self._phase_start = now
        elapsed = now - self._phase_start

        if self._phase == _PHASE_STANDUP:
            targets, done = self._standup_targets(elapsed)
            self._publish(targets)
            if done and self._is_near(self._q_default_urdf.tolist(), _STANDUP_TOL) and self._is_settled():
                self._phase = _PHASE_WAIT
                self._phase_start = now
                self.get_logger().info("[policy] standup complete -> WAIT")
            elif done and elapsed > float(self.get_parameter("ramp_duration").value) + 5.0:
                self._phase = _PHASE_WAIT
                self._phase_start = now
                self.get_logger().warn("[policy] standup timeout -> WAIT")
            return

        if self._phase == _PHASE_WAIT:
            self._publish(self._q_default_urdf.tolist())
            if self._latest_obs is not None and self._joint_state_seen:
                cmd_vel = self._latest_obs[6:9]
                if any(abs(v) > 1e-4 for v in cmd_vel):
                    if not self._is_near(self._q_default_urdf.tolist(), _STANDUP_TOL):
                        self.get_logger().warn(
                            "[policy] joints not near default pose — staying in WAIT",
                            throttle_duration_sec=2.0,
                        )
                        return
                    if not self._inputs_ok():
                        self.get_logger().warn(
                            "[policy] obs/state_estimate stale or IMU unhealthy "
                            "— staying in WAIT",
                            throttle_duration_sec=2.0,
                        )
                        return
                    self._phase = _PHASE_POLICY
                    self._phase_start = now
                    self._raw_high_count = 0
                    self._last_action = np.zeros(12, dtype=np.float32)
                    self._last_cmd_move_time = now
                    self._cmd_stopped = False
                    # Reset on POLICY entry: zero the frame stack AND reset the
                    # recurrent (GRU) hidden state so the new run starts clean and
                    # does not carry walking context from a previous session.
                    self._obs_history[:] = 0.0
                    self._last_stacked_stamp = None
                    self._reset_policy()
                    self.get_logger().info("[policy] cmd_vel received -> POLICY")
            return

        if self._phase == _PHASE_POLICY:
            if not self._inputs_ok():
                # Lost fresh obs / state estimate (e.g. IMU or a node dropped) —
                # hold the default pose instead of acting on stale/garbage input.
                self.get_logger().error(
                    "[policy] inputs stale/unhealthy during POLICY — holding default pose",
                    throttle_duration_sec=1.0,
                )
                self._publish(self._q_default_urdf.tolist())
                self._last_action = np.zeros(12, dtype=np.float32)
                return
            # Zero-command stop: the GRU retains walking context, so when the stick
            # returns to neutral the policy would keep moving. Once cmd_vel has been
            # ~zero for _ZERO_CMD_STOP_S, hold the default pose and reset the hidden
            # state (once) so the robot actually stops and resumes clean on next cmd.
            cmd_vel = self._latest_obs[6:9] if self._latest_obs is not None else (0.0, 0.0, 0.0)
            if any(abs(float(v)) > _ZERO_CMD_EPS for v in cmd_vel):
                self._last_cmd_move_time = now
                self._cmd_stopped = False
            elif now - self._last_cmd_move_time > _ZERO_CMD_STOP_S:
                if not self._cmd_stopped:
                    self._cmd_stopped = True
                    self._obs_history[:] = 0.0
                    self._last_stacked_stamp = None
                    self._last_action = np.zeros(12, dtype=np.float32)
                    self._reset_policy()
                    self.get_logger().info("[policy] cmd_vel ~0 → holding default pose (stop)")
                self._publish(self._q_default_urdf.tolist())
                return
            targets = self._run_inference()
            # divergence guard: if raw_action stays above limit, FAULT
            if np.max(np.abs(self._last_action)) > _RAW_LIMIT:
                self._raw_high_count += 1
                if self._raw_high_count >= _RAW_FAULT_N:
                    self.get_logger().error(
                        f"[policy] raw_action exceeded {_RAW_LIMIT} for {_RAW_FAULT_N} "
                        f"frames → FAULT (physical feedback lost?)"
                    )
                    self._phase = _PHASE_FAULT
                    self._phase_start = None
                    self._raw_high_count = 0
                    return
            else:
                self._raw_high_count = 0
            dry_run = bool(self.get_parameter("policy_dry_run").value)
            if dry_run:
                self._publish(self._q_default_urdf.tolist())
                self.get_logger().info(
                    "[policy] DRY RUN: holding default pose (policy_dry_run=true)",
                    throttle_duration_sec=5.0,
                )
            else:
                self._publish(targets)
            return

        if self._phase == _PHASE_LIEDOWN:
            targets, done = self._liedown_targets(elapsed)
            self._publish(targets)
            lie_down_dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
            near_zero = self._is_near(self._initial_pos or [0.0] * 12, _LIEDOWN_TOL) and self._is_settled()
            timed_out = done and elapsed > lie_down_dur + _LIEDOWN_TIMEOUT
            if (done and near_zero) or timed_out:
                self._phase = _PHASE_PASSIVE
                self._phase_start = None
                self._passive_broadcast = False
                self._last_action = np.zeros(12, dtype=np.float32)
                if timed_out and not near_zero:
                    self.get_logger().warn("[policy] liedown timeout -> PASSIVE")
                else:
                    self.get_logger().info("[policy] liedown complete -> PASSIVE")
            return

        if self._phase == _PHASE_FAULT:
            if not self._fault_broadcast:
                self._broadcast_gains(0.5, 0.1)
                self._fault_broadcast = True
            self._publish(self._last_published or self._q_default_urdf.tolist())


def main() -> None:
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
