"""
stand_node

Publishes default_q for all 12 joints at 50 Hz on /joint_commands.
Motors run with PD gains from robot.yaml, holding the robot in its
default standing pose.

Gain tuning — change kp/kd at runtime without restarting:

  ros2 param set /stand_node kp 5.0
  ros2 param set /stand_node kd 0.3

The new values are immediately broadcast to both motor bus nodes via the
ROS2 parameter service. No rebuild or relaunch required.

No joystick input, no feedback subscription.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState


def _load_joint_defaults(joints_cfg: list) -> tuple:
    """Return (names, default_q_values) from joints config list."""
    names    = [j['name'] for j in joints_cfg]
    defaults = [float(j['default_q']) for j in joints_cfg]
    return names, defaults



class StandNode(Node):
    def __init__(self) -> None:
        super().__init__('stand_node')

        # Deferred import: rclpy.parameter_client requires a full ROS2
        # environment and is not needed for unit-testing pure helpers.
        from rclpy.parameter_client import AsyncParametersClient

        cfg = self._load_config()
        self._names, self._default_q = _load_joint_defaults(cfg['joints'])

        kp_init = float(cfg['control']['kp'])
        kd_init = float(cfg['control']['kd'])
        self.declare_parameter('kp', kp_init)
        self.declare_parameter('kd', kd_init)

        # One async parameter client per bus node.
        # set_parameters calls are fire-and-forget; absent nodes fail silently.
        self._gain_clients = [
            AsyncParametersClient(self, '/motor_bus_front'),
            AsyncParametersClient(self, '/motor_bus_rear'),
        ]

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(1.0 / 50.0, self._publish)
        self.add_on_set_parameters_callback(self._on_gains_changed)

        self.get_logger().info(
            f'Stand node ready — {len(self._names)} joints at default_q  '
            f'kp={kp_init}  kd={kd_init}'
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        new_kp = next((p.value for p in params if p.name == 'kp'), None)
        new_kd = next((p.value for p in params if p.name == 'kd'), None)

        if new_kp is not None or new_kd is not None:
            kp = new_kp if new_kp is not None else self.get_parameter('kp').value
            kd = new_kd if new_kd is not None else self.get_parameter('kd').value
            gain_params = [
                Parameter('kp', Parameter.Type.DOUBLE, kp),
                Parameter('kd', Parameter.Type.DOUBLE, kd),
            ]
            for client in self._gain_clients:
                client.set_parameters(gain_params)
            self.get_logger().info(f'Gains updated → kp={kp}  kd={kd}')

        return SetParametersResult(successful=True)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self._names)
        msg.position = list(self._default_q)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
