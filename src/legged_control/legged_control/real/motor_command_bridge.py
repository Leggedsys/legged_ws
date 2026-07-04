"""motor_command_bridge

Subscribes /joint_commands (URDF frame, YAML order) and converts
to motor frame for motor_bus_node.  Applies hardware angle limits
and joint speed limits.

q_motor = direction * (q_urdf - zero_offset)

Publishes /joint_commands_motor (motor frame) — motor_bus_node subscribes
to this via launch remap.

Dry-run mode (ros2 param set /motor_command_bridge dry_run true):
  Writes both URDF and motor-frame commands to a log file instead of
  publishing to /joint_commands_motor.  Motors stay passive for safety.
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
        self.declare_parameter("dry_run", False)
        self.declare_parameter("log_path", "/tmp/motor_command_log.csv")
        cfg = self._load_config()
        self._joint_cfg = {j["name"]: j for j in cfg["joints"]}
        self._names = [j["name"] for j in cfg["joints"]]
        control = cfg.get("control", {})
        self._max_joint_speed = float(control.get("max_joint_speed", 12.0))
        self._last_cmd: dict[str, float] = {n: 0.0 for n in self._names}
        self._last_time: float | None = None

        self._pub = self.create_publisher(
            JointState, "/joint_commands_motor", 10
        )
        self.create_subscription(
            JointState, "/joint_commands", self._on_command, 10,
        )
        self._log_file = None
        dry = self.get_parameter("dry_run").value
        if dry:
            log_path = str(self.get_parameter("log_path").value)
            self._log_file = open(log_path, "w", buffering=1)
            self._log_file.write(
                "t,phase," +
                ",".join(f"{n}_urdf" for n in self._names) + "," +
                ",".join(f"{n}_motor" for n in self._names) + "," +
                ",".join(f"{n}_tau" for n in self._names) + "\n"
            )
        self.get_logger().info(
            "motor_command_bridge ready — "
            f"/joint_commands → /joint_commands_motor, max_speed={self._max_joint_speed} rad/s"
            + (" [DRY-RUN: logging to file, motors passive]" if dry else "")
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
        has_prev = self._last_time is not None
        dt = (now - self._last_time) if has_prev else 0.02
        self._last_time = now

        effort_map: dict[str, float] = {}
        if len(msg.effort) == len(msg.name):
            for _n, _e in zip(msg.name, msg.effort):
                effort_map[_n] = float(_e)

        q_urdf_list:   list[float] = []
        q_motor_list:  list[float] = []
        dq_motor_list: list[float] = []
        tau_motor_list: list[float] = []
        for name in self._names:
            cfg = self._joint_cfg[name]
            direction   = float(cfg["direction"])
            zero_offset = float(cfg["zero_offset"])
            gear_ratio  = float(cfg["gear_ratio"])
            q_urdf = float(pos_map.get(name, 0.0))
            q_urdf_clipped = float(
                max(float(cfg["q_min"]), min(float(cfg["q_max"]), q_urdf))
            )
            q_motor = direction * (q_urdf_clipped - zero_offset)
            if has_prev:
                dq = (q_motor - self._last_cmd[name]) / dt
                dq = max(-self._max_joint_speed, min(self._max_joint_speed, dq))
            else:
                dq = 0.0
            tau_urdf  = float(effort_map.get(name, 0.0))
            tau_motor = direction * tau_urdf / gear_ratio
            self._last_cmd[name] = q_motor
            q_urdf_list.append(q_urdf)
            q_motor_list.append(q_motor)
            dq_motor_list.append(dq)
            tau_motor_list.append(tau_motor)

        if self._log_file is not None:
            self._log_file.write(
                f"{now:.6f},," +
                ",".join(f"{v:.6f}" for v in q_urdf_list)    + "," +
                ",".join(f"{v:.6f}" for v in q_motor_list)   + "," +
                ",".join(f"{v:.6f}" for v in tau_motor_list) + "\n"
            )
            return  # dry-run: skip publishing, motors stay passive

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name     = list(self._names)
        out.position  = q_motor_list
        out.velocity  = dq_motor_list
        out.effort    = tau_motor_list
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MotorCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        if node._log_file is not None:
            node._log_file.close()
            node.get_logger().info(f"[dry-run] log written to {node.get_parameter('log_path').value}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
