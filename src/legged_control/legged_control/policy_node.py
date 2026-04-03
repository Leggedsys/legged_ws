"""
policy_node

Runs the locomotion RL policy at 50 Hz.

Observation vector (45-dim, assembled each tick):
  [0:3]   velocity command  (vx, vy, yaw_rate)  — from /cmd_vel
  [3:6]   gravity vector in body frame           — from /odin1/odometry orientation
  [6:9]   base angular velocity in body frame    — from /odin1/imu
  [9:21]  joint positions minus default_q        — from /joint_states_aggregated
  [21:33] joint velocities                       — from /joint_states_aggregated
  [33:45] last action                            — stored internally

Action (12-dim):
  Per-joint delta from default_q, scaled by action_scale.
  target_q = clamp(default_q + action_scale * action, q_min, q_max)

Subscribed topics:
  /cmd_vel                  geometry_msgs/Twist
  /odin1/odometry           nav_msgs/Odometry
  /odin1/imu                sensor_msgs/Imu
  /joint_states_aggregated  sensor_msgs/JointState

Published topic:
  /joint_commands           sensor_msgs/JointState
    name:     [FR_hip, ..., RL_calf]
    position: target_q for each joint (rad)
"""

import os
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('policy_node')

        cfg = self._load_config()
        joints_cfg   = cfg['joints']
        control_cfg  = cfg['control']
        policy_cfg   = cfg['policy']

        self._joint_names:  list[str]   = [j['name']      for j in joints_cfg]
        self._default_q:    np.ndarray  = np.array([j['default_q'] for j in joints_cfg])
        self._q_min:        np.ndarray  = np.array([j['q_min']     for j in joints_cfg])
        self._q_max:        np.ndarray  = np.array([j['q_max']     for j in joints_cfg])
        self._action_scale: float       = control_cfg['action_scale']
        n = len(self._joint_names)  # 12

        # Observation normalization (optional)
        raw_mean = policy_cfg.get('obs_mean', [])
        raw_std  = policy_cfg.get('obs_std',  [])
        self._obs_mean = np.array(raw_mean, dtype=np.float32) if raw_mean else None
        self._obs_std  = np.array(raw_std,  dtype=np.float32) if raw_std  else None

        # Internal state
        self._last_action = np.zeros(n, dtype=np.float32)
        self._cmd    = np.zeros(3,  dtype=np.float32)   # vx, vy, yaw_rate
        self._grav   = np.array([0., 0., -1.], dtype=np.float32)  # gravity in body frame
        self._angvel = np.zeros(3,  dtype=np.float32)
        self._q      = self._default_q.copy().astype(np.float32)
        self._dq     = np.zeros(n,  dtype=np.float32)

        # Load TorchScript model if path provided
        self._model = None
        model_path = policy_cfg.get('model_path', '')
        if model_path:
            self._load_model(model_path)

        # Subscribers
        self.create_subscription(Twist,      '/cmd_vel',                 self._on_cmd,    10)
        self.create_subscription(Odometry,   '/odin1/odometry',          self._on_odom,   10)
        self.create_subscription(Imu,        '/odin1/imu',               self._on_imu,    10)
        self.create_subscription(JointState, '/joint_states_aggregated', self._on_joints, 10)

        # Publisher
        self._pub = self.create_publisher(JointState, '/joint_commands', 10)

        policy_hz = control_cfg['policy_hz']
        self.create_timer(1.0 / policy_hz, self._tick)

        status = f'model={model_path}' if self._model else 'placeholder (stand-still)'
        self.get_logger().info(f'Policy node ready at {policy_hz} Hz, {status}')

    # ------------------------------------------------------------------ config

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        path = os.path.join(share, 'config', 'robot.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    def _load_model(self, path: str) -> None:
        try:
            import torch
            self._model = torch.jit.load(path, map_location='cpu')
            self._model.eval()
            self.get_logger().info(f'Loaded policy model: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model {path}: {e}')
            self._model = None

    # --------------------------------------------------------------- callbacks

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd[0] = msg.linear.x
        self._cmd[1] = msg.linear.y
        self._cmd[2] = msg.angular.z

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._grav = _quat_rotate_inverse(
            np.array([0., 0., -1.], dtype=np.float32),
            np.array([q.x, q.y, q.z, q.w], dtype=np.float32))

    def _on_imu(self, msg: Imu) -> None:
        w = msg.angular_velocity
        self._angvel = np.array([w.x, w.y, w.z], dtype=np.float32)

    def _on_joints(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if name in self._joint_names:
                idx = self._joint_names.index(name)
                if idx < len(msg.position):
                    self._q[idx]  = msg.position[idx]
                if idx < len(msg.velocity):
                    self._dq[idx] = msg.velocity[idx]

    # -------------------------------------------------------------------- tick

    def _tick(self) -> None:
        obs = self._build_obs()
        action = self._infer(obs)
        self._last_action = action.copy()
        self._send_commands(action)

    def _build_obs(self) -> np.ndarray:
        q_delta = self._q - self._default_q.astype(np.float32)
        obs = np.concatenate([
            self._cmd,          # [0:3]   velocity command
            self._grav,         # [3:6]   gravity vector in body frame
            self._angvel,       # [6:9]   angular velocity
            q_delta,            # [9:21]  joint positions (relative to default)
            self._dq,           # [21:33] joint velocities
            self._last_action,  # [33:45] last action
        ]).astype(np.float32)

        if self._obs_mean is not None and self._obs_std is not None:
            obs = (obs - self._obs_mean) / (self._obs_std + 1e-8)

        return obs

    def _infer(self, obs: np.ndarray) -> np.ndarray:
        if self._model is None:
            # Stand-still placeholder: zero action = hold default pose
            return np.zeros(len(self._joint_names), dtype=np.float32)

        import torch
        with torch.no_grad():
            t = torch.from_numpy(obs).unsqueeze(0)  # [1, 45]
            out = self._model(t)
            return out.squeeze(0).numpy()           # [12]

    def _send_commands(self, action: np.ndarray) -> None:
        target_q = self._default_q + self._action_scale * action
        target_q = np.clip(target_q, self._q_min, self._q_max).astype(np.float32)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = self._joint_names
        msg.position = target_q.tolist()
        self._pub.publish(msg)


# ------------------------------------------------------------------ math util

def _quat_rotate_inverse(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q=[x,y,z,w].

    Equivalent to expressing world-frame vector v in body frame,
    given q is the rotation from body to world.
    """
    q_vec = q[:3]
    a = v * (2.0 * q[3] ** 2 - 1.0)
    b = np.cross(q_vec, v) * q[3] * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return (a - b + c).astype(np.float32)


def main() -> None:
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
