"""
passive_monitor_node

Subscribes to all 12 joint_states topics. Every 0.5 s overwrites the
terminal with a 4-line position readout (one line per leg). Motors are
expected to run at zero torque so the operator can move joints by hand
to observe limits and default positions.

No command publishing, no joystick, no file writes.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_LEGS = ['FR', 'FL', 'RR', 'RL']
_SLOTS = ['hip', 'thigh', 'calf']


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' → 'fr/hip'"""
    leg, joint = name.split('_', 1)
    return f'{leg.lower()}/{joint.lower()}'


def _format_display(positions: dict) -> str:
    """
    Format all 12 positions as a 4-line string.

    positions: mapping of joint_name → float (nan if not yet received).
    """
    lines = []
    for i, leg in enumerate(_LEGS):
        prefix = '[passive] ' if i == 0 else '          '
        values = '  '.join(
            f'{slot}={positions.get(f"{leg}_{slot}", float("nan")):7.3f}'
            for slot in _SLOTS
        )
        lines.append(f'{prefix}{leg}  {values}')
    return '\n'.join(lines)


class PassiveMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('passive_monitor_node')

        cfg = self._load_config()
        self._positions: dict = {j['name']: float('nan') for j in cfg['joints']}

        for j in cfg['joints']:
            ns = _ns_from_joint_name(j['name'])
            self.create_subscription(
                JointState, f'/{ns}/joint_states',
                self._make_cb(j['name']), 10,
            )

        # Write directly to /dev/tty to bypass ros2 launch stdout capture
        # (launch pipes stdout and prepends [node-N] to every line,
        #  which breaks ANSI cursor control and multi-line overwrite)
        try:
            self._tty = open('/dev/tty', 'w')
        except OSError:
            self._tty = None

        self.create_timer(0.5, self._display)
        self.get_logger().info('Passive monitor ready — move joints by hand')

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _make_cb(self, joint_name: str):
        def cb(msg: JointState) -> None:
            if msg.position:
                self._positions[joint_name] = float(msg.position[0])
        return cb

    def _display(self) -> None:
        text = _format_display(self._positions)
        if self._tty:
            self._tty.write(f'\033[2J\033[H{text}\n')
            self._tty.flush()
        else:
            print(text, flush=True)


def main() -> None:
    rclpy.init()
    node = PassiveMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
