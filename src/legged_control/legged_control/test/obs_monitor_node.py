"""passive_monitor_node

Subscribes:
  /joint_states_aggregated  (sensor_msgs/JointState)
  /state_estimate           (std_msgs/Float32MultiArray, 9 floats)
  /height_command           (std_msgs/Float32, target stance height)
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
from std_msgs.msg import Float32, Float32MultiArray

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
        self._height_cmd = 0.0
        self._cmd_vel = (0.0, 0.0, 0.0)

        # Load hardware limits (motor frame) and convert to URDF frame
        self._q_urdf_min = np.full(12, -float("inf"))
        self._q_urdf_max = np.full(12, float("inf"))
        try:
            with open(os.path.join(share, "config", "robot.yaml")) as f:
                rcfg = yaml.safe_load(f)
            for j in rcfg.get("joints", []):
                name = j["name"]
                if name in _YAML_JOINT_NAMES:
                    idx = _YAML_JOINT_NAMES.index(name)
                    direction = float(j.get("direction", 1))
                    zero_offset = float(j.get("zero_offset", 0.0))
                    qm_min = float(j.get("q_min", -999))
                    qm_max = float(j.get("q_max", 999))
                    # Convert motor-frame limits to URDF frame
                    self._q_urdf_min[idx] = direction * qm_min + zero_offset
                    self._q_urdf_max[idx] = direction * qm_max + zero_offset
                    # Ensure min < max after direction flip
                    if self._q_urdf_min[idx] > self._q_urdf_max[idx]:
                        self._q_urdf_min[idx], self._q_urdf_max[idx] = (
                            self._q_urdf_max[idx], self._q_urdf_min[idx])
        except Exception:
            pass

        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32, "/height_command", self._on_height, 10)
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

    def _on_height(self, msg: Float32) -> None:
        self._height_cmd = float(msg.data)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _display(self) -> None:
        lv = self._state_estimate[0:3]
        av = self._state_estimate[3:6]
        pg = self._state_estimate[6:9]
        vx, vy, wz = self._cmd_vel
        pos_rel = self._joint_pos - self._q_default

        lines = [
            "┌─── PASSIVE MONITOR ──────────────────────────────────────────────┐",
            "│ JOINTS       q_urdf(rad)   q_min / q_max         pos_rel(rad)  │",
        ]
        for i, name in enumerate(_YAML_JOINT_NAMES):
            flag = " "
            pos = self._joint_pos[i]
            lo = self._q_urdf_min[i]
            hi = self._q_urdf_max[i]
            if not np.isnan(pos) and hi > lo:
                rng = hi - lo
                margin = 0.05 * rng
                if pos <= lo + margin:
                    flag = "*"  # near low limit
                elif pos >= hi - margin:
                    flag = "!"  # near high limit
            lines.append(
                f"│{flag} {name:<12}  {pos:+8.4f}    [{lo:+5.2f}, {hi:+5.2f}]    {pos_rel[i]:+8.4f}      │"
            )
        lines += [
            "│ IMU                                                      │",
            f"│  lin_vel    vx={lv[0]:+6.3f}  vy={lv[1]:+6.3f}  vz={lv[2]:+6.3f}     │",
            f"│  ang_vel    wx={av[0]:+6.3f}  wy={av[1]:+6.3f}  wz={av[2]:+6.3f}     │",
            f"│  proj_grav  gx={pg[0]:+6.3f}  gy={pg[1]:+6.3f}  gz={pg[2]:+6.3f}     │",
            "│ COMMAND                                                  │",
            f"│  vx={vx:+6.3f}  vy={vy:+6.3f}  wz={wz:+6.3f}                   │",
            "│ HEIGHT CMD                                               │",
            f"│  target={self._height_cmd:+6.3f} m                                  │",
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
