"""motor_command_bridge

Subscribes /joint_commands (URDF frame, YAML order) and converts
to motor frame for motor_bus_node.  Applies hardware angle limits
and joint speed limits.

q_motor = direction * (q_urdf - zero_offset)

Publishes /joint_commands_motor (motor frame) — motor_bus_node subscribes
to this via launch remap.
"""

from __future__ import annotations

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MotorCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("motor_command_bridge")

        self.declare_parameter("config_path", "")
        cfg = self._load_config()
        self._joint_cfg = {j["name"]: j for j in cfg["joints"]}
        self._names = [j["name"] for j in cfg["joints"]]
        control = cfg.get("control", {})
        self._max_joint_speed = float(control.get("max_joint_speed", 3.0))
        # Policy runs at 50 Hz
        self._max_delta = self._max_joint_speed * 0.02
        self._last_cmd: dict[str, float] = {n: 0.0 for n in self._names}
        self._last_time: float | None = None

        self._pub = self.create_publisher(
            JointState, "/joint_commands_motor", 10
        )
        self.create_subscription(
            JointState, "/joint_commands", self._on_command, 10,
        )
        self.get_logger().info(
            "motor_command_bridge ready — "
            f"/joint_commands → /joint_commands_motor, max_speed={self._max_joint_speed} rad/s"
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        path = str(self.get_parameter("config_path").value or "").strip()
        if not path:
            path = os.path.join(share, "config", "robot.yaml")
        with open(path) as f:
            return yaml.safe_load(f)

    def _on_command(self, msg: JointState) -> None:
        pos_map = dict(zip(msg.name, msg.position))
        now = time.monotonic()
        dt = now - self._last_time if self._last_time else 0.02
        self._last_time = now
        max_step = self._max_joint_speed * dt

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self._names)
        out.position = []
        calfs = {}  # debug
        for name in self._names:
            cfg = self._joint_cfg[name]
            direction = float(cfg["direction"])
            zero_offset = float(cfg["zero_offset"])
            q_urdf = float(pos_map.get(name, 0.0))
            q_motor = direction * (q_urdf - zero_offset)
            q_motor = float(
                max(float(cfg["q_min"]), min(float(cfg["q_max"]), q_motor))
            )
            # Speed limit
            prev = self._last_cmd.get(name, q_motor)
            delta = q_motor - prev
            if abs(delta) > max_step:
                q_motor = prev + max_step * (1.0 if delta > 0 else -1.0)
            self._last_cmd[name] = q_motor
            out.position.append(q_motor)
            if name.endswith("_calf"):
                calfs[name] = (q_urdf, q_motor, int(cfg.get("motor_id", -1)))
        # Debug: log calf conversions every 2s
        if calfs:
            self.get_logger().info(
                f"calves: " + " | ".join(
                    f"{n}: urdf={u:+.3f}→motor={m:+.3f} id={mid}"
                    for n, (u, m, mid) in calfs.items()
                ),
                throttle_duration_sec=2.0,
            )
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MotorCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
