"""
standup_node

Three-phase stand-up sequence (0 → default_q):

  Phase 1  HOLD   hold all joints at 0 for `hold_duration` seconds.
                  Lets motors finish initialising before any movement.
  Phase 2  RAMP   smooth-step interpolation from 0 → default_q over
                  `ramp_duration` seconds (zero velocity at both ends).
  Phase 3  STAND  hold at default_q indefinitely.

Typical usage:
  1. Place robot in starting pose and power on.
  2. ros2 launch legged_control robot.launch.py mode:=standup
  3. Watch terminal — it prints phase transitions and a live progress bar.

Runtime-tunable parameters (no relaunch required):
  ros2 param set /standup_node ramp_duration 10.0
  ros2 param set /standup_node kp 20.0
  ros2 param set /standup_node kd 1.0

kp/kd changes are immediately broadcast to both motor bus nodes.
"""
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState

_HOLD_DURATION_DEFAULT = 2.0   # seconds at 0 before starting ramp
_RAMP_DURATION_DEFAULT = 8.0   # seconds for 0 → default_q transition
_LOG_INTERVAL = 1.0            # seconds between progress log lines during ramp


def _smoothstep(t: float) -> float:
    """S-curve: maps t∈[0,1] → [0,1] with zero velocity at both endpoints."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _load_joint_defaults(joints_cfg: list) -> tuple[list, list]:
    names    = [j['name'] for j in joints_cfg]
    defaults = [float(j['default_q']) for j in joints_cfg]
    return names, defaults


class StandupNode(Node):
    def __init__(self) -> None:
        super().__init__('standup_node')

        cfg = self._load_config()
        self._names, self._default_q = _load_joint_defaults(cfg['joints'])

        kp_init = float(cfg['control']['kp'])
        kd_init = float(cfg['control']['kd'])
        self.declare_parameter('kp', kp_init)
        self.declare_parameter('kd', kd_init)
        self.declare_parameter('hold_duration', _HOLD_DURATION_DEFAULT)
        self.declare_parameter('ramp_duration', _RAMP_DURATION_DEFAULT)

        self._start_time: float | None = None
        self._phase = 'HOLD'          # 'HOLD' | 'RAMP' | 'STAND'
        self._last_log_t: float = 0.0

        self._gain_clients = [
            self.create_client(SetParameters, '/motor_bus_front/set_parameters'),
            self.create_client(SetParameters, '/motor_bus_rear/set_parameters'),
        ]

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(1.0 / 50.0, self._publish)
        self.add_on_set_parameters_callback(self._on_gains_changed)

        hold = self.get_parameter('hold_duration').value
        ramp = self.get_parameter('ramp_duration').value
        self.get_logger().info(
            f'[standup] ready — hold {hold:.1f}s then ramp {ramp:.1f}s to default_q  '
            f'kp={kp_init}  kd={kd_init}'
        )

    # ── config ───────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    # ── gain broadcast ────────────────────────────────────────────────────────

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        new_kp = next((p.value for p in params if p.name == 'kp'), None)
        new_kd = next((p.value for p in params if p.name == 'kd'), None)
        if new_kp is not None or new_kd is not None:
            kp = new_kp if new_kp is not None else self.get_parameter('kp').value
            kd = new_kd if new_kd is not None else self.get_parameter('kd').value
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name='kp', value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp))),
                Parameter(name='kd', value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd))),
            ]
            for client in self._gain_clients:
                if client.service_is_ready():
                    client.call_async(req)
            self.get_logger().info(f'[standup] gains updated → kp={kp}  kd={kd}')
        return SetParametersResult(successful=True)

    # ── main loop ─────────────────────────────────────────────────────────────

    def _publish(self) -> None:
        now = time.monotonic()
        if self._start_time is None:
            self._start_time = now

        hold_dur = self.get_parameter('hold_duration').value
        ramp_dur = self.get_parameter('ramp_duration').value
        elapsed  = now - self._start_time

        if elapsed < hold_dur:
            # ── Phase 1: HOLD ──────────────────────────────────────────────
            if self._phase != 'HOLD':
                self._phase = 'HOLD'
            targets = [0.0] * len(self._names)

        elif elapsed < hold_dur + ramp_dur:
            # ── Phase 2: RAMP ──────────────────────────────────────────────
            if self._phase != 'RAMP':
                self._phase = 'RAMP'
                self.get_logger().info('[standup] RAMP started — rising to default_q')

            t     = (elapsed - hold_dur) / ramp_dur
            alpha = _smoothstep(t)
            targets = [alpha * dq for dq in self._default_q]

            if now - self._last_log_t >= _LOG_INTERVAL:
                pct = int(alpha * 100)
                bar = '█' * (pct // 5) + '░' * (20 - pct // 5)
                self.get_logger().info(f'[standup] [{bar}] {pct:3d}%')
                self._last_log_t = now

        else:
            # ── Phase 3: STAND ─────────────────────────────────────────────
            if self._phase != 'STAND':
                self._phase = 'STAND'
                self.get_logger().info('[standup] STAND reached — holding at default_q')
            targets = list(self._default_q)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self._names)
        msg.position = targets
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StandupNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
