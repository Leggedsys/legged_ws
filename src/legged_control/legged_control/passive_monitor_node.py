"""passive_monitor_node

Subscribes:
  /joint_states_aggregated  (sensor_msgs/JointState)
  /state_estimate           (std_msgs/Float32MultiArray, 9 floats)
  /height_scan              (std_msgs/Float32MultiArray, 325 floats)
  /cmd_vel                  (geometry_msgs/Twist)

Displays the full RL observation space in the terminal at 2 Hz.
Used for sensor calibration and data-link verification in passive mode.
"""

from __future__ import annotations

import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


class PassiveMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("passive_monitor_node")

        self.declare_parameter("policy_config_path", "")
        share = get_package_share_directory("legged_control")

        pcfg_path = str(self.get_parameter("policy_config_path").value or "").strip()
        if not pcfg_path:
            pcfg_path = os.path.join(share, "config", "policy.yaml")
        try:
            with open(pcfg_path) as f:
                pcfg = yaml.safe_load(f).get("policy", {})
            qd = pcfg.get("joint_default_q_urdf", {})
            self._q_default = np.array([float(qd.get(n, 0.0)) for n in _YAML_JOINT_NAMES])
        except Exception:
            self._q_default = np.zeros(12)

        self._joint_pos = np.full(12, float("nan"))
        self._joint_vel = np.full(12, float("nan"))
        self._state_estimate = np.zeros(9)
        self._height_scan = np.zeros(325)
        self._cmd_vel = (0.0, 0.0, 0.0)

        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32MultiArray, "/height_scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        try:
            self._tty = open("/dev/tty", "w")
        except OSError:
            self._tty = None

        self.create_timer(0.5, self._display)
        self.get_logger().info("passive_monitor_node ready")

    def _on_joints(self, msg: JointState) -> None:
        pos_map = dict(zip(msg.name, msg.position))
        vel_map = dict(zip(msg.name, msg.velocity))
        for i, n in enumerate(_YAML_JOINT_NAMES):
            if n in pos_map:
                self._joint_pos[i] = float(pos_map[n])
            if n in vel_map:
                self._joint_vel[i] = float(vel_map[n])

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_estimate = np.array(msg.data[:9])

    def _on_scan(self, msg: Float32MultiArray) -> None:
        self._height_scan = np.array(msg.data[:325])

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _display(self) -> None:
        lv = self._state_estimate[0:3]
        av = self._state_estimate[3:6]
        pg = self._state_estimate[6:9]
        vx, vy, wz = self._cmd_vel
        pos_rel = self._joint_pos - self._q_default

        lines = [
            "┌─── PASSIVE MONITOR ──────────────────────────────────────┐",
            "│ JOINTS       pos_rel(rad)   vel(rad/s)                   │",
        ]
        for i, name in enumerate(_YAML_JOINT_NAMES):
            lines.append(f"│  {name:<12}  {pos_rel[i]:+7.3f}       {self._joint_vel[i]:+7.3f}            │")
        lines += [
            "│ IMU                                                      │",
            f"│  lin_vel    vx={lv[0]:+6.3f}  vy={lv[1]:+6.3f}  vz={lv[2]:+6.3f}     │",
            f"│  ang_vel    wx={av[0]:+6.3f}  wy={av[1]:+6.3f}  wz={av[2]:+6.3f}     │",
            f"│  proj_grav  gx={pg[0]:+6.3f}  gy={pg[1]:+6.3f}  gz={pg[2]:+6.3f}     │",
            "│ COMMAND                                                  │",
            f"│  vx={vx:+6.3f}  vy={vy:+6.3f}  wz={wz:+6.3f}                   │",
            "│ HEIGHT SCAN                                              │",
            f"│  mean={np.nanmean(self._height_scan):+6.3f}  std={np.nanstd(self._height_scan):5.3f}  "
            f"min={np.nanmin(self._height_scan):+6.3f}  max={np.nanmax(self._height_scan):+6.3f}  │",
            "└──────────────────────────────────────────────────────────┘",
        ]
        text = "\n".join(lines)
        if self._tty:
            self._tty.write(f"\033[2J\033[H{text}\n")
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
        if rclpy.ok():
            rclpy.shutdown()
