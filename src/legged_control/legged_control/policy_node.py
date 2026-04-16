# src/legged_control/legged_control/policy_node.py
"""
policy_node

Startup sequence then locomotion policy at 50 Hz.

Phase 1  STANDUP — ramp all joints from 0 (power-on position) to default_q
                   using a smooth-step curve.  No sensor requirement.
Phase 2  WAIT    — hold at default_q until IMU, odometry, and joint states
                   are all fresh (< 0.5 s old).
Phase 3  RUN     — 50 Hz inference loop.

Observation vector (45 dims, fixed order):
  obs[0:3]   base angular velocity (rad/s)       from /odin1/imu
  obs[3:6]   projected gravity (body frame)       from /odin1/odometry quaternion
  obs[6:9]   velocity commands (vx, vy, yaw_rate) from /cmd_vel (zeros if absent)
  obs[9:21]  joint pos relative to default_q_urdf (URDF frame, rad)
  obs[21:33] joint velocity (URDF frame, rad/s)
  obs[33:45] last action (raw model output, dimensionless)

Action vector (12 dims):
  Per-joint position delta from default_q_urdf, scaled by action_scale per joint.
  Decoded to motor-frame target_q and published on /joint_commands.
  Clipping is applied in motor frame (q_min/q_max in robot.yaml are motor-frame values).

Standup timing is configured under the `standup` key in robot.yaml.
"""

import os
import time

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


# Observation joint order used by Isaac Lab joint_pos/joint_vel terms.
OBS_JOINT_ORDER = [
    "FL_hip",
    "FR_hip",
    "RL_hip",
    "RR_hip",
    "FL_thigh",
    "FR_thigh",
    "RL_thigh",
    "RR_thigh",
    "FL_calf",
    "FR_calf",
    "RL_calf",
    "RR_calf",
]

# Action joint order used by Isaac Lab JointPositionActionCfg expansion.
ACTION_JOINT_ORDER = [
    "FL_hip",
    "FR_hip",
    "FL_thigh",
    "FR_thigh",
    "FL_calf",
    "FR_calf",
    "RL_hip",
    "RR_hip",
    "RL_thigh",
    "RR_thigh",
    "RL_calf",
    "RR_calf",
]

_PHASE_STANDUP = 'STANDUP'
_PHASE_WAIT    = 'WAIT'
_PHASE_RUN     = 'RUN'


def _smoothstep(t: float) -> float:
    """S-curve: maps t∈[0,1] → [0,1] with zero velocity at both endpoints."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


_LEG_GROUP = {
    "FR": "front",
    "FL": "front",
    "RR": "rear",
    "RL": "rear",
}


_LEGACY_ACTION_SCALE = {
    "front": {"hip": 0.04, "thigh": 0.05, "calf": 0.03},
    "rear": {"hip": 0.05, "thigh": 0.07, "calf": 0.04},
}


def _quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q = [w, x, y, z].

    Equivalent to: rotate v from world frame into body frame.
    Formula matches Isaac Lab mdp.projected_gravity.
    """
    qw = q[0]
    qvec = q[1:]
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(qvec, v) * (2.0 * qw)
    c = qvec * np.dot(qvec, v) * 2.0
    return a - b + c


def _build_joint_params(cfg: dict, joint_order: list[str]) -> tuple:
    """Parse robot.yaml and return six (12,) arrays aligned to joint_order.

    Returns:
        directions    — +1 or -1 per joint
        zero_offsets  — static URDF frame offset (rad)
        default_q_urdf — standing pose in URDF frame (direction×default_q_motor + zero_offset)
        q_mins        — joint lower limit (motor frame, rad)
        q_maxs        — joint upper limit (motor frame, rad)
        scales        — action scale (rad/unit) per joint
    """
    joint_map = {j["name"]: j for j in cfg["joints"]}
    action_scale_cfg = cfg["control"]["action_scale"]

    directions = np.zeros(12)
    zero_offsets = np.zeros(12)
    default_q_urdf = np.zeros(12)
    q_mins = np.zeros(12)
    q_maxs = np.zeros(12)
    scales = np.zeros(12)

    for i, name in enumerate(joint_order):
        j = joint_map[name]
        leg = name.split("_")[0]  # 'FR', 'FL', 'RR', 'RL'
        jtype = name.split("_", 1)[1].lower()  # 'hip', 'thigh', 'calf'
        group = _LEG_GROUP[leg]  # 'front' or 'rear'

        d = float(j["direction"])
        zo = float(j["zero_offset"])
        dq_motor = float(j["default_q"])

        directions[i] = d
        zero_offsets[i] = zo
        default_q_urdf[i] = d * dq_motor + zo
        q_mins[i] = float(j["q_min"])
        q_maxs[i] = float(j["q_max"])
        scales[i] = _action_scale_lookup(action_scale_cfg, group, jtype)

    return directions, zero_offsets, default_q_urdf, q_mins, q_maxs, scales


def _action_scale_lookup(action_scale_cfg, group: str, jtype: str) -> float:
    """Support both per-joint action scales and legacy scalar config."""
    if isinstance(action_scale_cfg, dict):
        return float(action_scale_cfg[group][jtype])
    if isinstance(action_scale_cfg, (int, float)):
        return float(_LEGACY_ACTION_SCALE[group][jtype])
    raise TypeError(
        "policy_node: control.action_scale must be either a nested mapping "
        "or a numeric legacy scalar"
    )


def _motor_to_urdf(
    q_motor: np.ndarray,
    dq_motor: np.ndarray,
    directions: np.ndarray,
    zero_offsets: np.ndarray,
) -> tuple:
    """Convert motor-frame position/velocity to URDF frame.

    q_urdf  = direction × q_motor  + zero_offset
    dq_urdf = direction × dq_motor
    """
    q_urdf = directions * q_motor + zero_offsets
    dq_urdf = directions * dq_motor
    return q_urdf, dq_urdf


def _decode_action(
    action: np.ndarray,
    default_q_urdf: np.ndarray,
    scales: np.ndarray,
    q_mins_motor: np.ndarray,
    q_maxs_motor: np.ndarray,
    directions: np.ndarray,
    zero_offsets: np.ndarray,
) -> np.ndarray:
    """Decode raw model output to motor-frame target positions.

    q_min/q_max in robot.yaml are motor-frame values, so clipping is done
    AFTER converting from URDF frame to motor frame.

    target_q_urdf  = default_q_urdf + scale × action
    target_q_motor = clip((target_q_urdf - zero_offset) / direction, q_min, q_max)
    """
    target_q_urdf = default_q_urdf + scales * action
    target_q_motor = (target_q_urdf - zero_offsets) / directions
    return np.clip(target_q_motor, q_mins_motor, q_maxs_motor)


def _command_with_timeout(
    cmd_vel: np.ndarray, last_cmd_t: float | None, now: float, timeout: float
) -> np.ndarray:
    """Return zero command if /cmd_vel is absent or stale."""
    if last_cmd_t is None or now - last_cmd_t > timeout:
        return np.zeros(3, dtype=np.float32)
    return cmd_vel


def _reorder(
    values: np.ndarray, source_order: list[str], target_order: list[str]
) -> np.ndarray:
    """Reorder a vector between named joint orders."""
    source_idx = {name: i for i, name in enumerate(source_order)}
    return np.array(
        [values[source_idx[name]] for name in target_order], dtype=values.dtype
    )


class PolicyNode(Node):
    _SENSOR_TIMEOUT = 0.5  # seconds
    _CMD_TIMEOUT = 0.5

    def __init__(self) -> None:
        super().__init__("policy_node")

        # torch is a system dependency — import lazily so pure-function tests
        # can import this module without torch being installed.
        import torch  # noqa: PLC0415

        cfg = self._load_config()
        policy_cfg = cfg.get("policy", {})
        control_cfg = cfg["control"]

        (
            self._obs_directions,
            self._obs_zero_offsets,
            self._obs_default_q_urdf,
            _obs_q_mins,
            _obs_q_maxs,
            _obs_scales,
        ) = _build_joint_params(cfg, OBS_JOINT_ORDER)
        (
            self._action_directions,
            self._action_zero_offsets,
            self._action_default_q_urdf,
            self._action_q_mins,
            self._action_q_maxs,
            self._action_scales,
        ) = _build_joint_params(cfg, ACTION_JOINT_ORDER)

        # Standup phase config
        standup_cfg = cfg.get('standup', {})
        self._standup_hold = float(standup_cfg.get('hold_duration', 2.0))
        self._standup_ramp = float(standup_cfg.get('ramp_duration', 8.0))

        # Motor-frame default_q in ACTION order (used during standup output)
        _joint_map = {j['name']: j for j in cfg['joints']}
        self._action_default_q_motor = np.array(
            [float(_joint_map[name]['default_q']) for name in ACTION_JOINT_ORDER],
            dtype=np.float32,
        )

        self._phase: str = _PHASE_STANDUP
        self._phase_start: float | None = None

        # Observation normalisation (optional)
        obs_mean = policy_cfg.get("obs_mean", [])
        obs_std = policy_cfg.get("obs_std", [])
        if obs_mean and obs_std:
            if len(obs_mean) != 45 or len(obs_std) != 45:
                raise RuntimeError(
                    f"policy_node: obs_mean and obs_std must each have 45 elements, "
                    f"got obs_mean={len(obs_mean)} obs_std={len(obs_std)}"
                )
            self._obs_mean = np.array(obs_mean, dtype=np.float32)
            self._obs_std = np.array(obs_std, dtype=np.float32)
        else:
            self._obs_mean = None
            self._obs_std = None

        # Load TorchScript model
        model_path = policy_cfg.get("model_path", "")
        if not model_path:
            raise RuntimeError(
                "policy_node: policy.model_path is not set in robot.yaml. "
                "Set it to the path of your exported .pt file."
            )
        self._model = torch.jit.load(model_path, map_location="cpu")
        self._model.eval()
        with torch.no_grad():  # warm-up + shape check
            _warmup_out = self._model(torch.zeros(1, 45))
        if tuple(_warmup_out.shape) != (1, 12):
            raise RuntimeError(
                f"policy_node: model must output shape (1, 12), "
                f"got {tuple(_warmup_out.shape)}"
            )

        self._torch = torch  # keep reference for _tick

        self._last_action_action_order = np.zeros(12, dtype=np.float32)

        # Sensor state buffers
        self._ang_vel = np.zeros(3, dtype=np.float32)
        self._gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._cmd_vel = np.zeros(3, dtype=np.float32)
        self._last_cmd_t = None
        self._q_motor = np.zeros(12, dtype=np.float32)
        self._dq_motor = np.zeros(12, dtype=np.float32)
        self._last_imu_t = None
        self._last_odom_t = None
        self._last_joint_t = None

        self._joint_idx = {name: i for i, name in enumerate(OBS_JOINT_ORDER)}

        # Subscriptions
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joint_states, 10
        )
        self.create_subscription(Imu, "/odin1/imu", self._on_imu, 10)
        self.create_subscription(Odometry, "/odin1/odometry", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)

        policy_hz = float(control_cfg.get("policy_hz", 50.0))
        self.create_timer(1.0 / policy_hz, self._tick)

        self.get_logger().info(
            f"Policy node ready — standup hold={self._standup_hold:.1f}s "
            f"ramp={self._standup_ramp:.1f}s → wait for sensors → "
            f"{policy_hz} Hz inference  model={model_path}  "
            f"normalise={self._obs_mean is not None}"
        )

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        av = msg.angular_velocity
        self._ang_vel = np.array([av.x, av.y, av.z], dtype=np.float32)
        self._last_imu_t = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        ori = msg.pose.pose.orientation
        q = np.array([ori.w, ori.x, ori.y, ori.z])
        self._gravity = _quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0])).astype(
            np.float32
        )
        self._last_odom_t = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )
        self._last_cmd_t = time.monotonic()

    def _on_joint_states(self, msg: JointState) -> None:
        if len(msg.velocity) != len(msg.name):
            self.get_logger().warn(
                f"policy_node: /joint_states_aggregated has {len(msg.name)} names "
                f"but {len(msg.velocity)} velocity values — skipping velocity update",
                throttle_duration_sec=5.0,
            )
            return
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            idx = self._joint_idx.get(name)
            if idx is not None:
                self._q_motor[idx] = float(pos)
                self._dq_motor[idx] = float(vel)
        self._last_joint_t = time.monotonic()

    # ── tick ─────────────────────────────────────────────────────────────────

    def _publish_motor_targets(self, targets: np.ndarray) -> None:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(ACTION_JOINT_ORDER)
        out.position = targets.tolist()
        self._pub.publish(out)

    def _tick(self) -> None:
        now = time.monotonic()

        # ── Phase 1: STANDUP ──────────────────────────────────────────────────
        if self._phase == _PHASE_STANDUP:
            if self._phase_start is None:
                self._phase_start = now
            elapsed = now - self._phase_start

            if elapsed < self._standup_hold:
                targets = np.zeros(12, dtype=np.float32)
            elif elapsed < self._standup_hold + self._standup_ramp:
                t = (elapsed - self._standup_hold) / self._standup_ramp
                targets = _smoothstep(t) * self._action_default_q_motor
            else:
                self._phase = _PHASE_WAIT
                self._phase_start = now
                self.get_logger().info(
                    'policy_node: standup complete — holding default_q, waiting for sensors'
                )
                targets = self._action_default_q_motor.copy()

            self._publish_motor_targets(targets)
            return

        # ── Phase 2: WAIT ─────────────────────────────────────────────────────
        if self._phase == _PHASE_WAIT:
            sensors_ready = (
                self._last_imu_t is not None
                and now - self._last_imu_t <= self._SENSOR_TIMEOUT
                and self._last_odom_t is not None
                and now - self._last_odom_t <= self._SENSOR_TIMEOUT
                and self._last_joint_t is not None
                and now - self._last_joint_t <= self._SENSOR_TIMEOUT
            )
            if sensors_ready:
                self._phase = _PHASE_RUN
                self._phase_start = now
                self._last_action_action_order = np.zeros(12, dtype=np.float32)
                self.get_logger().info('policy_node: sensors ready — starting inference')
            else:
                self.get_logger().info(
                    'policy_node: waiting for sensors (IMU / odom / joints)…',
                    throttle_duration_sec=2.0,
                )
            self._publish_motor_targets(self._action_default_q_motor)
            return

        # ── Phase 3: RUN ──────────────────────────────────────────────────────
        # LiDAR timeout guard
        if (
            self._last_imu_t is None
            or now - self._last_imu_t > self._SENSOR_TIMEOUT
            or self._last_odom_t is None
            or now - self._last_odom_t > self._SENSOR_TIMEOUT
            or self._last_joint_t is None
            or now - self._last_joint_t > self._SENSOR_TIMEOUT
        ):
            self.get_logger().error(
                "policy_node: sensor timeout (IMU/odom/joints) — /joint_commands suspended",
                throttle_duration_sec=2.0,
            )
            return

        torch = self._torch
        cmd_vel = _command_with_timeout(
            self._cmd_vel, self._last_cmd_t, now, self._CMD_TIMEOUT
        )
        if self._last_cmd_t is None or now - self._last_cmd_t > self._CMD_TIMEOUT:
            self.get_logger().warn(
                "policy_node: /cmd_vel stale or absent — falling back to zero velocity command",
                throttle_duration_sec=2.0,
            )

        # Convert joint readings to URDF frame
        q_urdf, dq_urdf = _motor_to_urdf(
            self._q_motor,
            self._dq_motor,
            self._obs_directions,
            self._obs_zero_offsets,
        )
        last_action_obs_order = _reorder(
            self._last_action_action_order, ACTION_JOINT_ORDER, OBS_JOINT_ORDER
        )

        # Build 45-dim observation
        obs = np.concatenate(
            [
                self._ang_vel,  # [0:3]
                self._gravity,  # [3:6]
                cmd_vel,  # [6:9]
                (q_urdf - self._obs_default_q_urdf),  # [9:21]
                dq_urdf,  # [21:33]
                last_action_obs_order,  # [33:45]
            ]
        ).astype(np.float32)

        # Optional normalisation
        if self._obs_mean is not None:
            obs = (obs - self._obs_mean) / (self._obs_std + 1e-8)

        # Inference
        with torch.no_grad():
            action = (
                self._model(torch.from_numpy(obs).unsqueeze(0))
                .squeeze(0)
                .numpy()
                .astype(np.float32)
            )

        self._last_action_action_order = action.copy()

        # Decode to motor-frame targets (clip in motor frame)
        target_q_motor = _decode_action(
            action,
            self._action_default_q_urdf,
            self._action_scales,
            self._action_q_mins,
            self._action_q_maxs,  # motor-frame limits from robot.yaml
            self._action_directions,
            self._action_zero_offsets,
        )

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(ACTION_JOINT_ORDER)
        out.position = target_q_motor.tolist()
        self._pub.publish(out)

    # ── config ───────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)


def main() -> None:
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
