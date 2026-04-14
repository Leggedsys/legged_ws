"""
policy_monitor_node

Terminal status panel for policy mode. Reads existing topics and refreshes a
compact dashboard without affecting control logic.
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

from legged_control.policy_node import (
    ACTION_JOINT_ORDER,
    OBS_JOINT_ORDER,
    _build_joint_params,
    _motor_to_urdf,
)


def _fresh_label(last_t: float | None, now: float, timeout: float) -> str:
    if last_t is None:
        return "--"
    return "OK" if now - last_t <= timeout else "STALE"


def _clip_count(
    target_q: np.ndarray, q_mins: np.ndarray, q_maxs: np.ndarray, tol: float = 1e-4
) -> int:
    at_limit = (target_q <= q_mins + tol) | (target_q >= q_maxs - tol)
    return int(np.count_nonzero(at_limit))


def _leg_abs_mean(q_rel: np.ndarray, joint_order: list[str]) -> dict:
    grouped = {"FL": [], "FR": [], "RL": [], "RR": []}
    for name, value in zip(joint_order, q_rel):
        grouped[name.split("_")[0]].append(abs(float(value)))
    return {
        leg: (sum(values) / len(values) if values else float("nan"))
        for leg, values in grouped.items()
    }


def _format_panel(
    model_name: str,
    imu_label: str,
    odom_label: str,
    joints_label: str,
    cmd_label: str,
    cmd_vel: np.ndarray,
    ang_vel: np.ndarray,
    gravity: np.ndarray,
    q_rel_abs: dict,
    clip_count: int,
) -> str:
    return "\n".join(
        [
            (
                f"[policy] model={model_name}  imu={imu_label}  odom={odom_label}  "
                f"joints={joints_label}  cmd={cmd_label}  clip={clip_count}/12"
            ),
            (
                f"         cmd_vel  vx={cmd_vel[0]:6.2f}  vy={cmd_vel[1]:6.2f}  "
                f"yaw={cmd_vel[2]:6.2f}"
            ),
            (
                f"         ang_vel  wx={ang_vel[0]:6.2f}  wy={ang_vel[1]:6.2f}  "
                f"wz={ang_vel[2]:6.2f}"
            ),
            (
                f"         gravity gx={gravity[0]:6.2f}  gy={gravity[1]:6.2f}  "
                f"gz={gravity[2]:6.2f}"
            ),
            (
                f"         |q-default|  FL={q_rel_abs['FL']:.3f}  FR={q_rel_abs['FR']:.3f}  "
                f"RL={q_rel_abs['RL']:.3f}  RR={q_rel_abs['RR']:.3f}"
            ),
        ]
    )


class PolicyMonitorNode(Node):
    _TIMEOUT = 0.5

    def __init__(self) -> None:
        super().__init__("policy_monitor_node")

        cfg = self._load_config()
        policy_cfg = cfg.get("policy", {})
        self._model_name = (
            os.path.basename(policy_cfg.get("model_path", "")) or "<unset>"
        )
        (
            self._obs_directions,
            self._obs_zero_offsets,
            self._obs_default_q,
            _obs_q_mins,
            _obs_q_maxs,
            _obs_scales,
        ) = _build_joint_params(cfg, OBS_JOINT_ORDER)
        (
            _action_directions,
            _action_zero_offsets,
            _action_default_q,
            self._action_q_mins,
            self._action_q_maxs,
            _action_scales,
        ) = _build_joint_params(cfg, ACTION_JOINT_ORDER)

        self._ang_vel = np.zeros(3, dtype=np.float32)
        self._gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._cmd_vel = np.zeros(3, dtype=np.float32)
        self._q_motor = np.zeros(12, dtype=np.float32)
        self._dq_motor = np.zeros(12, dtype=np.float32)
        self._target_q = np.zeros(12, dtype=np.float32)
        self._last_imu_t = None
        self._last_odom_t = None
        self._last_joint_t = None
        self._last_cmd_t = None
        self._last_target_t = None

        self._joint_idx = {name: i for i, name in enumerate(OBS_JOINT_ORDER)}
        self._target_idx = {name: i for i, name in enumerate(ACTION_JOINT_ORDER)}

        self.create_subscription(Imu, "/odin1/imu", self._on_imu, 10)
        self.create_subscription(Odometry, "/odin1/odometry", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joint_states, 10
        )
        self.create_subscription(
            JointState, "/joint_commands", self._on_joint_commands, 10
        )

        try:
            self._tty = open("/dev/tty", "w")
        except OSError:
            self._tty = None

        self.create_timer(0.5, self._display)
        self.get_logger().info("Policy monitor ready — terminal panel enabled")

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_imu(self, msg: Imu) -> None:
        av = msg.angular_velocity
        self._ang_vel = np.array([av.x, av.y, av.z], dtype=np.float32)
        self._last_imu_t = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        ori = msg.pose.pose.orientation
        q = np.array([ori.w, ori.x, ori.y, ori.z], dtype=np.float32)
        from legged_control.policy_node import _quat_rotate_inverse

        self._gravity = _quat_rotate_inverse(
            q, np.array([0.0, 0.0, -1.0], dtype=np.float32)
        ).astype(np.float32)
        self._last_odom_t = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )
        self._last_cmd_t = time.monotonic()

    def _on_joint_states(self, msg: JointState) -> None:
        if len(msg.velocity) != len(msg.name):
            return
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            idx = self._joint_idx.get(name)
            if idx is not None:
                self._q_motor[idx] = float(pos)
                self._dq_motor[idx] = float(vel)
        self._last_joint_t = time.monotonic()

    def _on_joint_commands(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            idx = self._target_idx.get(name)
            if idx is not None:
                self._target_q[idx] = float(pos)
        self._last_target_t = time.monotonic()

    def _display(self) -> None:
        now = time.monotonic()
        q_urdf, _dq_urdf = _motor_to_urdf(
            self._q_motor, self._dq_motor, self._obs_directions, self._obs_zero_offsets
        )
        q_rel = q_urdf - self._obs_default_q
        q_rel_abs = _leg_abs_mean(q_rel, OBS_JOINT_ORDER)
        clip_count = 0
        if (
            self._last_target_t is not None
            and now - self._last_target_t <= self._TIMEOUT
        ):
            clip_count = _clip_count(
                self._target_q, self._action_q_mins, self._action_q_maxs
            )

        text = _format_panel(
            model_name=self._model_name,
            imu_label=_fresh_label(self._last_imu_t, now, self._TIMEOUT),
            odom_label=_fresh_label(self._last_odom_t, now, self._TIMEOUT),
            joints_label=_fresh_label(self._last_joint_t, now, self._TIMEOUT),
            cmd_label=_fresh_label(self._last_cmd_t, now, self._TIMEOUT),
            cmd_vel=self._cmd_vel,
            ang_vel=self._ang_vel,
            gravity=self._gravity,
            q_rel_abs=q_rel_abs,
            clip_count=clip_count,
        )
        if self._tty:
            self._tty.write(f"\033[2J\033[H{text}\n")
            self._tty.flush()
        else:
            print(text, flush=True)


def main() -> None:
    rclpy.init()
    node = PolicyMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
