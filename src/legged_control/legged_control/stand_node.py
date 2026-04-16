"""
stand_node

Holds all 12 joints at 0 (power-on position) at 50 Hz on /joint_commands.
Place the robot in the desired pose, power on, then launch stand mode —
it will hold that pose. Use this to tune kp/kd.

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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState


def _load_joint_names(joints_cfg: list) -> list:
    return [j['name'] for j in joints_cfg]


class StandNode(Node):
    def __init__(self) -> None:
        super().__init__('stand_node')

        cfg = self._load_config()
        self._names = _load_joint_names(cfg['joints'])

        kp_init = float(cfg['control']['kp'])
        kd_init = float(cfg['control']['kd'])
        self.declare_parameter('kp', kp_init)
        self.declare_parameter('kd', kd_init)

        # Service clients for pushing kp/kd to bus nodes (fire-and-forget).
        self._gain_clients = [
            self.create_client(SetParameters, '/motor_bus_front/set_parameters'),
            self.create_client(SetParameters, '/motor_bus_rear/set_parameters'),
        ]

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(1.0 / 50.0, self._publish)
        self.add_on_set_parameters_callback(self._on_gains_changed)

        self.get_logger().info(
            f'Stand node ready — holding {len(self._names)} joints at 0 (power-on position)  '
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
            self.get_logger().info(f'Gains updated → kp={kp}  kd={kd}')

        return SetParametersResult(successful=True)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self._names)
        msg.position = [0.0] * len(self._names)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
