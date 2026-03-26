from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

from .sdk_loader import load_sdk


class GoM80106Node(Node):
    def __init__(self) -> None:
        super().__init__('go_m8010_6_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('motor_id', 3)
        self.declare_parameter('loop_hz', 1000.0)
        self.declare_parameter('target_dq', 6.28)
        self.declare_parameter('target_q', 0.0)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('tau', 0.0)

        sdk = load_sdk()
        self._sdk = sdk
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self._serial = sdk.SerialPort(serial_port)
        self._cmd = sdk.MotorCmd()
        self._data = sdk.MotorData()
        self._state_pub = self.create_publisher(JointState, 'joint_states', 10)

        loop_hz = self.get_parameter('loop_hz').value
        if loop_hz <= 0.0:
            raise ValueError('loop_hz must be > 0')

        self._timer = self.create_timer(1.0 / loop_hz, self._tick)
        self.get_logger().info(f'Opened Unitree motor on {serial_port}')

    def _tick(self) -> None:
        sdk = self._sdk
        self._data.motorType = sdk.MotorType.GO_M8010_6
        self._cmd.motorType = sdk.MotorType.GO_M8010_6
        self._cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
        self._cmd.id = self.get_parameter('motor_id').value
        self._cmd.q = self.get_parameter('target_q').value
        self._cmd.dq = self.get_parameter('target_dq').value * sdk.queryGearRatio(sdk.MotorType.GO_M8010_6)
        self._cmd.kp = self.get_parameter('kp').value
        self._cmd.kd = self.get_parameter('kd').value
        self._cmd.tau = self.get_parameter('tau').value

        self._serial.sendRecv(self._cmd, self._data)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['go_m8010_6']
        msg.position = [float(self._data.q)]
        msg.velocity = [float(self._data.dq)]
        msg.effort = [float(self._data.tau)]
        self._state_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = GoM80106Node()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
