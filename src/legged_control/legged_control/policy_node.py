# src/legged_control/legged_control/policy_node.py
"""
policy_node

Runs the learned locomotion policy at 50 Hz.

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

Sensors: LiDAR (/odin1/imu, /odin1/odometry) must be live; if either is silent
for >0.5 s the tick loop stops publishing and logs an error.
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


# Canonical joint order — must match robot.yaml and training environment
JOINT_ORDER = [
    'FR_hip', 'FR_thigh', 'FR_calf',
    'FL_hip', 'FL_thigh', 'FL_calf',
    'RR_hip', 'RR_thigh', 'RR_calf',
    'RL_hip', 'RL_thigh', 'RL_calf',
]

_LEG_GROUP = {
    'FR': 'front', 'FL': 'front',
    'RR': 'rear',  'RL': 'rear',
}


def _quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q = [w, x, y, z].

    Equivalent to: rotate v from world frame into body frame.
    Formula matches Isaac Lab mdp.projected_gravity.
    """
    qw   = q[0]
    qvec = q[1:]
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(qvec, v) * (2.0 * qw)
    c = qvec * np.dot(qvec, v) * 2.0
    return a - b + c


def _build_joint_params(cfg: dict) -> tuple:
    """Parse robot.yaml and return six (12,) arrays aligned to JOINT_ORDER.

    Returns:
        directions    — +1 or -1 per joint
        zero_offsets  — static URDF frame offset (rad)
        default_q_urdf — standing pose in URDF frame (direction×default_q_motor + zero_offset)
        q_mins        — joint lower limit (motor frame, rad)
        q_maxs        — joint upper limit (motor frame, rad)
        scales        — action scale (rad/unit) per joint
    """
    joint_map        = {j['name']: j for j in cfg['joints']}
    action_scale_cfg = cfg['control']['action_scale']

    directions     = np.zeros(12)
    zero_offsets   = np.zeros(12)
    default_q_urdf = np.zeros(12)
    q_mins         = np.zeros(12)
    q_maxs         = np.zeros(12)
    scales         = np.zeros(12)

    for i, name in enumerate(JOINT_ORDER):
        j     = joint_map[name]
        leg   = name.split('_')[0]          # 'FR', 'FL', 'RR', 'RL'
        jtype = name.split('_', 1)[1].lower()  # 'hip', 'thigh', 'calf'
        group = _LEG_GROUP[leg]             # 'front' or 'rear'

        d        = float(j['direction'])
        zo       = float(j['zero_offset'])
        dq_motor = float(j['default_q'])

        directions[i]     = d
        zero_offsets[i]   = zo
        default_q_urdf[i] = d * dq_motor + zo
        q_mins[i]         = float(j['q_min'])
        q_maxs[i]         = float(j['q_max'])
        scales[i]         = float(action_scale_cfg[group][jtype])

    return directions, zero_offsets, default_q_urdf, q_mins, q_maxs, scales


def _motor_to_urdf(q_motor: np.ndarray, dq_motor: np.ndarray,
                   directions: np.ndarray, zero_offsets: np.ndarray) -> tuple:
    """Convert motor-frame position/velocity to URDF frame.

    q_urdf  = direction × q_motor  + zero_offset
    dq_urdf = direction × dq_motor
    """
    q_urdf  = directions * q_motor  + zero_offsets
    dq_urdf = directions * dq_motor
    return q_urdf, dq_urdf


def _decode_action(action: np.ndarray,
                   default_q_urdf: np.ndarray,
                   scales: np.ndarray,
                   q_mins_motor: np.ndarray,
                   q_maxs_motor: np.ndarray,
                   directions: np.ndarray,
                   zero_offsets: np.ndarray) -> np.ndarray:
    """Decode raw model output to motor-frame target positions.

    q_min/q_max in robot.yaml are motor-frame values, so clipping is done
    AFTER converting from URDF frame to motor frame.

    target_q_urdf  = default_q_urdf + scale × action
    target_q_motor = clip((target_q_urdf - zero_offset) / direction, q_min, q_max)
    """
    target_q_urdf  = default_q_urdf + scales * action
    target_q_motor = (target_q_urdf - zero_offsets) / directions
    return np.clip(target_q_motor, q_mins_motor, q_maxs_motor)


class PolicyNode(Node):
    _SENSOR_TIMEOUT = 0.5  # seconds

    def __init__(self) -> None:
        super().__init__('policy_node')

        # torch is a system dependency — import lazily so pure-function tests
        # can import this module without torch being installed.
        import torch  # noqa: PLC0415

        cfg         = self._load_config()
        policy_cfg  = cfg.get('policy', {})
        control_cfg = cfg['control']

        (self._directions, self._zero_offsets, self._default_q_urdf,
         self._q_mins, self._q_maxs, self._scales) = _build_joint_params(cfg)

        # Observation normalisation (optional)
        obs_mean = policy_cfg.get('obs_mean', [])
        obs_std  = policy_cfg.get('obs_std',  [])
        if obs_mean and obs_std:
            self._obs_mean = np.array(obs_mean, dtype=np.float32)
            self._obs_std  = np.array(obs_std,  dtype=np.float32)
        else:
            self._obs_mean = None
            self._obs_std  = None

        # Load TorchScript model
        model_path = policy_cfg.get('model_path', '')
        if not model_path:
            raise RuntimeError(
                'policy_node: policy.model_path is not set in robot.yaml. '
                'Set it to the path of your exported .pt file.')
        self._model = torch.jit.load(model_path, map_location='cpu')
        self._model.eval()
        with torch.no_grad():                        # warm-up
            self._model(torch.zeros(1, 45))

        self._torch = torch   # keep reference for _tick

        self._last_action = np.zeros(12, dtype=np.float32)

        # Sensor state buffers
        self._ang_vel    = np.zeros(3, dtype=np.float32)
        self._gravity    = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._cmd_vel    = np.zeros(3, dtype=np.float32)
        self._q_motor    = np.zeros(12, dtype=np.float32)
        self._dq_motor   = np.zeros(12, dtype=np.float32)
        self._last_imu_t  = None
        self._last_odom_t = None
        self._last_joint_t = None

        self._joint_idx = {name: i for i, name in enumerate(JOINT_ORDER)}

        # Subscriptions
        self.create_subscription(
            JointState, '/joint_states_aggregated', self._on_joint_states, 10)
        self.create_subscription(Imu, '/odin1/imu', self._on_imu, 10)
        self.create_subscription(
            Odometry, '/odin1/odometry', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)

        policy_hz = float(control_cfg.get('policy_hz', 50.0))
        self.create_timer(1.0 / policy_hz, self._tick)

        self.get_logger().info(
            f'Policy node ready — {policy_hz} Hz  '
            f'model={model_path}  '
            f'normalise={self._obs_mean is not None}')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        av = msg.angular_velocity
        self._ang_vel    = np.array([av.x, av.y, av.z], dtype=np.float32)
        self._last_imu_t = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        ori = msg.pose.pose.orientation
        q   = np.array([ori.w, ori.x, ori.y, ori.z])
        self._gravity     = _quat_rotate_inverse(
            q, np.array([0.0, 0.0, -1.0])).astype(np.float32)
        self._last_odom_t = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)

    def _on_joint_states(self, msg: JointState) -> None:
        if len(msg.velocity) != len(msg.name):
            self.get_logger().warn(
                f'policy_node: /joint_states_aggregated has {len(msg.name)} names '
                f'but {len(msg.velocity)} velocity values — skipping velocity update',
                throttle_duration_sec=5.0,
            )
            return
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            idx = self._joint_idx.get(name)
            if idx is not None:
                self._q_motor[idx]  = float(pos)
                self._dq_motor[idx] = float(vel)
        self._last_joint_t = time.monotonic()

    # ── tick ─────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()

        # LiDAR timeout guard
        if (self._last_imu_t  is None or
                now - self._last_imu_t  > self._SENSOR_TIMEOUT or
                self._last_odom_t is None or
                now - self._last_odom_t > self._SENSOR_TIMEOUT or
                self._last_joint_t is None or
                now - self._last_joint_t > self._SENSOR_TIMEOUT):
            self.get_logger().error(
                'policy_node: sensor timeout (IMU/odom/joints) — /joint_commands suspended',
                throttle_duration_sec=2.0,
            )
            return

        torch = self._torch

        # Convert joint readings to URDF frame
        q_urdf, dq_urdf = _motor_to_urdf(
            self._q_motor, self._dq_motor,
            self._directions, self._zero_offsets,
        )

        # Build 45-dim observation
        obs = np.concatenate([
            self._ang_vel,                    # [0:3]
            self._gravity,                    # [3:6]
            self._cmd_vel,                    # [6:9]
            (q_urdf - self._default_q_urdf),  # [9:21]
            dq_urdf,                          # [21:33]
            self._last_action,                # [33:45]
        ]).astype(np.float32)

        # Optional normalisation
        if self._obs_mean is not None:
            obs = (obs - self._obs_mean) / (self._obs_std + 1e-8)

        # Inference
        with torch.no_grad():
            action = self._model(
                torch.from_numpy(obs).unsqueeze(0)
            ).squeeze(0).numpy().astype(np.float32)

        self._last_action = action.copy()

        # Decode to motor-frame targets (clip in motor frame)
        target_q_motor = _decode_action(
            action, self._default_q_urdf, self._scales,
            self._q_mins, self._q_maxs,   # motor-frame limits from robot.yaml
            self._directions, self._zero_offsets,
        )

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name     = list(JOINT_ORDER)
        out.position = target_q_motor.tolist()
        self._pub.publish(out)

    # ── config ───────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)


def main() -> None:
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
