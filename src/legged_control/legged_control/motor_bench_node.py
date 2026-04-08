"""
motor_bench_node

Single-leg joystick position test. Sweeps one of three joints through its
configured q_min/q_max range via left stick vertical. B/X buttons cycle
through joints; switching locks the departing joint at its last feedback
position.

Subscriptions:
  /joy                        sensor_msgs/Joy
  /<ns>/joint_states × 3      sensor_msgs/JointState

Publication:
  /joint_commands             sensor_msgs/JointState  (50 Hz)

Parameters:
  leg  string  default 'FR'   leg prefix, case-insensitive (FR/FL/RR/RL)
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

_AXIS_POS = 1    # left stick vertical (inverted inside callback)
_BTN_NEXT = 1    # B button — next joint
_BTN_PREV = 2    # X button — previous joint

_NS_MAP = {
    'FR': ['fr/hip', 'fr/thigh', 'fr/calf'],
    'FL': ['fl/hip', 'fl/thigh', 'fl/calf'],
    'RR': ['rr/hip', 'rr/thigh', 'rr/calf'],
    'RL': ['rl/hip', 'rl/thigh', 'rl/calf'],
}


def _apply_deadzone(v: float, dz: float = 0.05) -> float:
    """Return v rescaled so that |v| <= dz maps to 0, full range stays ±1."""
    if abs(v) <= dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _map_to_range(v: float, q_min: float, q_max: float) -> float:
    """Map v ∈ [-1, 1] linearly to [q_min, q_max]."""
    return q_min + (v + 1.0) / 2.0 * (q_max - q_min)


def _select_joints(joints_cfg: list, leg: str) -> list:
    """Return the joints whose name starts with leg (case-insensitive)."""
    prefix = leg.upper()
    return [j for j in joints_cfg if j['name'].upper().startswith(prefix)]


class MotorBenchNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_bench_node')
        self.declare_parameter('leg', 'FR')
        leg = self.get_parameter('leg').get_parameter_value().string_value.upper()

        cfg = self._load_config()
        self._joints = _select_joints(cfg['joints'], leg)
        if len(self._joints) != 3:
            raise RuntimeError(
                f"Expected 3 joints for leg '{leg}', "
                f"found {len(self._joints)}: {[j['name'] for j in self._joints]}"
            )

        self._dz = float(cfg['teleop']['deadzone'])
        self._active_idx = 0
        self._locked_q   = [float(j['default_q']) for j in self._joints]
        self._feedback_q = [float(j['default_q']) for j in self._joints]
        self._prev_buttons: list[int] = []

        # Feedback subscriptions — one per joint namespace
        ns_list = _NS_MAP[leg]
        self._subs_fb = []
        for i, ns in enumerate(ns_list):
            sub = self.create_subscription(
                JointState, f'/{ns}/joint_states',
                self._make_fb_cb(i), 10)
            self._subs_fb.append(sub)

        self._sub_joy = self.create_subscription(
            Joy, '/joy', self._on_joy, 10)
        self._pub = self.create_publisher(
            JointState, '/joint_commands', 10)
        self._timer = self.create_timer(1.0 / 50.0, self._publish_cmd)

        self.get_logger().info(
            f'Motor bench ready  leg={leg}  '
            f'active={self._joints[0]["name"]}  '
            f'(B=next joint  X=prev joint)')

    # ------------------------------------------------------------------

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _make_fb_cb(self, idx: int):
        def cb(msg: JointState) -> None:
            if msg.position:
                self._feedback_q[idx] = float(msg.position[0])
        return cb

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        prev    = self._prev_buttons

        def fell(i: int) -> bool:
            """True on the first frame a button is pressed (falling edge)."""
            return (i < len(buttons) and buttons[i] == 1
                    and (i >= len(prev) or prev[i] == 0))

        # Left stick vertical → position of active joint
        # axis 1 is inverted: push up = negative raw value
        raw = -msg.axes[_AXIS_POS]
        v   = _apply_deadzone(raw, self._dz)
        if v != 0.0:
            j = self._joints[self._active_idx]
            self._locked_q[self._active_idx] = _map_to_range(
                v, j['q_min'], j['q_max'])

        # B → next joint
        if fell(_BTN_NEXT):
            self._locked_q[self._active_idx] = self._feedback_q[self._active_idx]
            self._active_idx = (self._active_idx + 1) % 3
            self.get_logger().info(
                f'Active joint → {self._joints[self._active_idx]["name"]}')

        # X → previous joint
        if fell(_BTN_PREV):
            self._locked_q[self._active_idx] = self._feedback_q[self._active_idx]
            self._active_idx = (self._active_idx - 1) % 3
            self.get_logger().info(
                f'Active joint → {self._joints[self._active_idx]["name"]}')

        self._prev_buttons = buttons

    def _publish_cmd(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = [j['name'] for j in self._joints]
        msg.position = list(self._locked_q)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotorBenchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
