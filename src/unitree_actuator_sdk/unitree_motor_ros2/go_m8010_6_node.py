import threading

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
        self.declare_parameter('joint_name', 'unknown')
        # Fallback parameters used when no /joint_commands message has arrived
        self.declare_parameter('target_dq', 0.0)
        self.declare_parameter('target_q', 0.0)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('tau', 0.0)

        sdk = load_sdk()
        self._sdk = sdk
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self._serial = sdk.SerialPort(serial_port)
        self._cmd = sdk.MotorCmd()
        self._data = sdk.MotorData()
        self._state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Command cache updated by /joint_commands subscriber.
        # None means fall back to ROS parameters.
        self._cmd_lock = threading.Lock()
        self._cached_q: float | None = None

        self._cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self._on_joint_cmd, 10)

        loop_hz = self.get_parameter('loop_hz').value
        if loop_hz <= 0.0:
            raise ValueError('loop_hz must be > 0')

        self._timer = self.create_timer(1.0 / loop_hz, self._tick)
        joint_name = self.get_parameter('joint_name').value
        self.get_logger().info(
            f'Motor [{joint_name}] id={self.get_parameter("motor_id").value} '
            f'on {serial_port}')

    def _on_joint_cmd(self, msg: JointState) -> None:
        joint_name = self.get_parameter('joint_name').value
        if joint_name not in msg.name:
            return
        idx = msg.name.index(joint_name)
        if idx < len(msg.position):
            with self._cmd_lock:
                self._cached_q = float(msg.position[idx])

    def _tick(self) -> None:
        sdk = self._sdk

        with self._cmd_lock:
            target_q = (self._cached_q
                        if self._cached_q is not None
                        else self.get_parameter('target_q').value)

        self._data.motorType = sdk.MotorType.GO_M8010_6
        self._cmd.motorType = sdk.MotorType.GO_M8010_6
        self._cmd.mode = sdk.queryMotorMode(sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
        self._cmd.id = self.get_parameter('motor_id').value
        self._cmd.q = target_q
        self._cmd.dq = self.get_parameter('target_dq').value * sdk.queryGearRatio(sdk.MotorType.GO_M8010_6)
        self._cmd.kp = self.get_parameter('kp').value
        self._cmd.kd = self.get_parameter('kd').value
        self._cmd.tau = self.get_parameter('tau').value

        self._serial.sendRecv(self._cmd, self._data)

        joint_name = self.get_parameter('joint_name').value
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_name]
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
