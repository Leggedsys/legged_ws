"""policy_node — RL policy deployment node.

Runs a TorchScript policy at 50 Hz with the same PASSIVE->STANDUP->WAIT->POLICY->LIE_DOWN
state machine as gait_node. The POLICY phase replaces IK trot with neural-network inference.

Observation vector (373 dims) from /observation:
  [0:3]   base_lin_vel     from /state_estimate[0:3]
  [3:6]   base_ang_vel     from /state_estimate[3:6]
  [6:9]   projected_gravity from /state_estimate[6:9]
  [9:12]  velocity_commands [vx, vy, omega_z] from /cmd_vel
  [12:24] joint_pos_rel    q_urdf - q_default_urdf (yaml order -> policy order, hip_sign_flip applied)
  [24:36] joint_vel        dq_urdf (yaml order -> policy order, hip_sign_flip applied)
  [36:48] last_action      previous raw policy output (policy order)
  [48:373] height_scan     from /height_scan (325 floats)

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

_DEFAULT_HIP_SIGN_FLIP_POLICY_IDX = [
    _POLICY_JOINT_NAMES.index("FR_hip"),
    _POLICY_JOINT_NAMES.index("RL_hip"),
]


def _reorder_policy_to_yaml(policy_vec: np.ndarray) -> np.ndarray:
    return policy_vec[_POLICY_TO_YAML]


def _decode_action(
    action_policy: np.ndarray,
    q_default_urdf_yaml: np.ndarray,
    action_scale_yaml: np.ndarray,
    sign_flip_policy_idx: list[int],
    soft_q_min_urdf: np.ndarray,
    soft_q_max_urdf: np.ndarray,
) -> np.ndarray:
    action = action_policy.copy()
    for idx in sign_flip_policy_idx:
        action[idx] *= -1.0

    action_yaml = _reorder_policy_to_yaml(action)
    q_target_urdf = q_default_urdf_yaml + action_yaml * action_scale_yaml
    return np.clip(q_target_urdf, soft_q_min_urdf, soft_q_max_urdf)


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

# ── obs validation (training 2σ ranges) ────────────────────────────────────────

_OBS_CHECKS = {
    "base_lin_vel":  [("vx", 0, -1.00, 2.00), ("vy", 1, -0.50, 0.50), ("vz", 2, -0.50, 0.60)],
    "base_ang_vel":  [("wx", 3, -3.00, 3.00), ("wy", 4, -2.50, 2.50), ("wz", 5, -2.00, 2.00)],
    "proj_grav":     [("gx", 6, -0.25, 0.35), ("gy", 7, -0.30, 0.25), ("gz", 8, -1.10, -0.55)],
    "vel_cmd":       [("vx", 9, -0.68, 1.59), ("vy", 10, -0.33, 0.33), ("wz", 11, -1.10, 1.09)],
    "joint_pos":     [(f"pos_{_POLICY_JOINT_NAMES[i]}", 12 + i, -0.30, 0.30) for i in range(12)],
    "joint_vel":     [(f"vel_{_POLICY_JOINT_NAMES[i]}", 24 + i, -6.0, 6.0) for i in range(12)],
    "height_mean":   [("hs", 0, 0.08, 0.43)],
}


def _validate_obs(obs: np.ndarray, height_scan: np.ndarray) -> list[str]:
    bad = []
    for group_name, checks in _OBS_CHECKS.items():
        for label, idx, lo, hi in checks:
            if group_name == "height_mean":
                v = float(np.nanmean(height_scan))
            else:
                v = float(obs[idx])
            if not (lo <= v <= hi):
                bad.append(f"{label}={v:+.3f}[{lo:+.1f},{hi:+.1f}]")
    return bad


_PHASE_PASSIVE = "PASSIVE"
_PHASE_STANDUP = "STANDUP"
_PHASE_WAIT    = "WAIT"
_PHASE_POLICY  = "POLICY"
_PHASE_LIEDOWN = "LIEDOWN"
_PHASE_FAULT   = "FAULT"

_STANDUP_TOL  = 0.05
_LIEDOWN_TOL  = 0.05
_VEL_SETTLED  = 0.05
_LIEDOWN_TIMEOUT = 3.0  # seconds past lie_down_duration before forcing PASSIVE
_RAW_LIMIT    = 20.0    # raw_action divergence threshold
_RAW_FAULT_N  = 3       # consecutive frames above limit → FAULT


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
        self._soft_q_min_urdf, self._soft_q_max_urdf = self._build_soft_limits(policy_cfg)

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
        self.create_subscription(Bool, "/posture_command", self._on_posture, 10)
        self.add_on_set_parameters_callback(self._on_gains_changed)
        self.create_timer(self._dt, self._tick)

        self._latest_obs: np.ndarray | None = None

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

    def _build_soft_limits(
        self, pcfg: dict
    ) -> tuple[np.ndarray, np.ndarray]:
        lims = pcfg.get("joint_soft_limits", {})
        hip   = lims.get("hip",   [-0.45,  0.45])
        thigh = lims.get("thigh", [-1.48,  0.68])
        calf  = lims.get("calf",  [-2.295, -0.405])
        type_map = {"hip": hip, "thigh": thigh, "calf": calf}
        q_min, q_max = [], []
        for name in _YAML_JOINT_NAMES:
            jtype = name.split("_")[1]  # "hip" / "thigh" / "calf"
            lo, hi = type_map[jtype]
            q_min.append(float(lo))
            q_max.append(float(hi))
        return np.array(q_min, dtype=np.float32), np.array(q_max, dtype=np.float32)

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
        self._latest_obs = np.array(msg.data[:373], dtype=np.float32)

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

    def _broadcast_gains(self, kp: float, kd: float) -> None:
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name="kp", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp))),
            Parameter(name="kd", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd))),
        ]
        for client in self._gain_clients:
            if client.service_is_ready():
                client.call_async(req)

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

        obs = self._latest_obs.copy()
        obs[36:48] = np.clip(self._last_action, -5.0, 5.0)
        bad = _validate_obs(obs, obs[48:373])
        if bad:
            self.get_logger().warn(
                f"obs anomaly: {' | '.join(bad[:6])}"
                + (f" ...+{len(bad)-6}" if len(bad) > 6 else ""),
                throttle_duration_sec=3.0,
            )
        try:
            import torch
            with torch.inference_mode():
                obs_t = torch.from_numpy(obs).unsqueeze(0)
                action = self._policy(obs_t).squeeze(0).numpy()
        except Exception as e:
            self.get_logger().error(f"[policy] inference error: {e}", throttle_duration_sec=1.0)
            return self._q_default_urdf.tolist()

        self._last_action = action.copy()
        self._pub_raw.publish(Float32MultiArray(data=action.tolist()))
        hs = obs[48:373]
        self.get_logger().info(
            f"[policy] raw_action[max={np.max(np.abs(action)):+.1f}]  "
            f"cmd_vel={[f'{x:+.2f}' for x in obs[9:12]]}  "
            f"hs={np.mean(hs):.3f}",
            throttle_duration_sec=1.0,
        )
        q_urdf = _decode_action(
            action,
            self._q_default_urdf,
            self._action_scale,
            self._sign_flip_policy_idx,
            self._soft_q_min_urdf,
            self._soft_q_max_urdf,
        )
        return q_urdf.tolist()

    def _tick(self) -> None:
        now = time.monotonic()

        if self._phase == _PHASE_PASSIVE:
            if self._stand_requested:
                self._broadcast_gains(
                    float(self.get_parameter("kp").value),
                    float(self.get_parameter("kd").value),
                )
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
                if not self._passive_broadcast:
                    self._broadcast_gains(0.0, 0.0)
                    self._passive_broadcast = True
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
                cmd_vel = self._latest_obs[9:12]
                if any(abs(v) > 1e-4 for v in cmd_vel):
                    if not self._is_near(self._q_default_urdf.tolist(), _STANDUP_TOL):
                        self.get_logger().warn(
                            "[policy] joints not near default pose — staying in WAIT",
                            throttle_duration_sec=2.0,
                        )
                        return
                    obs = self._latest_obs.copy()
                    obs[36:48] = self._last_action
                    bad = _validate_obs(obs, obs[48:373])
                    if bad:
                        self.get_logger().error(
                            f"🚨 obs out of range: {', '.join(bad[:8])}"
                            + (f" ...+{len(bad)-8} more" if len(bad) > 8 else ""),
                            throttle_duration_sec=2.0,
                        )
                        return
                    self._phase = _PHASE_POLICY
                    self._phase_start = now
                    self._raw_high_count = 0
                    self._last_action = np.zeros(12, dtype=np.float32)
                    self._reset_policy()
                    self.get_logger().info("[policy] cmd_vel received -> POLICY")
            return

        if self._phase == _PHASE_POLICY:
            targets = self._run_inference()
            obs = self._latest_obs
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
            if obs is not None:
                obs_check = obs.copy()
                obs_check[36:48] = self._last_action
                bad = _validate_obs(obs_check, obs_check[48:373])
                if bad:
                    self.get_logger().error(
                        f"\U0001f6a8 POLICY obs out of range: {', '.join(bad[:6])}"
                        + (f" ...+{len(bad)-6} more" if len(bad) > 6 else ""),
                        throttle_duration_sec=2.0,
                    )
                    self._publish(self._q_default_urdf.tolist())
                    self._last_action = np.zeros(12, dtype=np.float32)
                    return
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
