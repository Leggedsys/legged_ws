"""
monitor_node — terminal dashboard + frame logger for policy debugging.

Subscribes to key topics, displays a 2 Hz dashboard, and writes per-frame
data + errors to ~/.legged_logs/<timestamp>/.
"""

from __future__ import annotations

import csv
import os
import threading
import time as _time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log as RosLog
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

_YAML_JOINT_NAMES = [
    "FR_hip","FR_thigh","FR_calf","FL_hip","FL_thigh","FL_calf",
    "RR_hip","RR_thigh","RR_calf","RL_hip","RL_thigh","RL_calf",
]

_POLICY_JOINT_NAMES = [
    "FL_hip","FR_hip","FL_thigh","FR_thigh","FL_calf","FR_calf",
    "RL_hip","RR_hip","RL_thigh","RR_thigh","RL_calf","RR_calf",
]

_YAML_TO_POLICY = [_YAML_JOINT_NAMES.index(n) for n in _POLICY_JOINT_NAMES]


def _make_log_dir() -> str:
    ts = _time.strftime("%Y%m%d_%H%M%S")
    d = os.path.expanduser(f"~/.legged_logs/{ts}")
    os.makedirs(d, exist_ok=True)
    return d


class MonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("monitor_node")

        self._state_estimate = np.zeros(9, dtype=np.float32)
        self._height_scan = np.zeros(325, dtype=np.float32)
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._raw_action = np.zeros(12, dtype=np.float32)
        self._q_target = np.zeros(12, dtype=np.float32)
        self._joint_pos = np.zeros(12, dtype=np.float32)
        self._obs_pos_rel = np.zeros(12, dtype=np.float32)
        self._obs_joint_vel = np.zeros(12, dtype=np.float32)
        self._obs_last_action = np.zeros(12, dtype=np.float32)
        self._phase = "----"
        self._errors: deque[tuple[str, str, str]] = deque(maxlen=20)  # (time, level, msg)

        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32MultiArray, "/height_scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, "/raw_policy_action", self._on_raw, 10)
        self.create_subscription(JointState, "/joint_commands", self._on_q_target, 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/observation", self._on_obs, 10)
        self.create_subscription(RosLog, "/rosout", self._on_rosout, 10)

        self._log_dir = _make_log_dir()
        self._frame_csv = open(os.path.join(self._log_dir, "frames.csv"), "w", buffering=1)
        self._err_csv = open(os.path.join(self._log_dir, "errors.csv"), "w", buffering=1)
        self._write_frame_header()
        self._write_error_header()

        try:
            self._tty = open("/dev/tty", "w")
        except OSError:
            self._tty = None

        self.create_timer(0.5, self._display)
        self.create_timer(0.02, self._log_frame)
        self.get_logger().info(f"monitor_node ready — logging to {self._log_dir}")

    def _write_frame_header(self) -> None:
        hdr = ["t","phase","cmd_vx","cmd_vy","cmd_wz",
               "est_vx","est_vy","est_vz","gx","gy","gz"] + \
              [f"raw_{i}" for i in range(12)] + \
              [f"q_urg_{i}" for i in range(12)] + \
              [f"pos_rel_{i}" for i in range(12)] + \
              [f"jvel_{i}" for i in range(12)] + \
              [f"last_a_{i}" for i in range(12)] + \
              ["hs_mean","hs_min","hs_max"]
        self._frame_csv.write(",".join(hdr) + "\n")

    def _write_error_header(self) -> None:
        self._err_csv.write("t,level,msg\n")

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_estimate = np.array(msg.data[:9], dtype=np.float32)

    def _on_scan(self, msg: Float32MultiArray) -> None:
        self._height_scan = np.array(msg.data[:325], dtype=np.float32)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_raw(self, msg: Float32MultiArray) -> None:
        self._raw_action = np.array(msg.data[:12], dtype=np.float32)

    def _on_q_target(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            if n in _YAML_JOINT_NAMES:
                self._q_target[_YAML_JOINT_NAMES.index(n)] = float(p)

    def _on_joints(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            if n in _YAML_JOINT_NAMES:
                self._joint_pos[_YAML_JOINT_NAMES.index(n)] = float(p)

    def _on_obs(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 48:
            return
        d = msg.data
        self._obs_pos_rel = np.array(d[12:24], dtype=np.float32)
        self._obs_joint_vel = np.array(d[24:36], dtype=np.float32)
        self._obs_last_action = np.array(d[36:48], dtype=np.float32)

    def _on_rosout(self, msg: RosLog) -> None:
        if "policy_node" not in msg.name:
            return
        if msg.level >= RosLog.LEVEL_ERROR:
            level = "ERR"
        elif msg.level >= RosLog.LEVEL_WARN:
            level = "WRN"
        else:
            return
        t = _time.strftime("%H:%M:%S")
        text = msg.msg.strip()
        self._errors.appendleft((t, level, text))
        self._err_csv.write(f"{t},{level},\"{text}\"\n")

    def _log_frame(self) -> None:
        est = self._state_estimate
        hs = self._height_scan
        row = [
            f"{_time.monotonic():.6f}", self._phase,
            f"{self._cmd_vel[0]:.4f}", f"{self._cmd_vel[1]:.4f}", f"{self._cmd_vel[2]:.4f}",
            f"{est[0]:.4f}", f"{est[1]:.4f}", f"{est[2]:.4f}",
            f"{est[6]:.4f}", f"{est[7]:.4f}", f"{est[8]:.4f}",
        ] + [f"{self._raw_action[i]:.4f}" for i in range(12)] \
          + [f"{self._q_target[i]:.4f}" for i in range(12)] \
          + [f"{self._obs_pos_rel[i]:.4f}" for i in range(12)] \
          + [f"{self._obs_joint_vel[i]:.4f}" for i in range(12)] \
          + [f"{self._obs_last_action[i]:.4f}" for i in range(12)] \
          + [f"{np.mean(hs):.4f}", f"{np.min(hs):.4f}", f"{np.max(hs):.4f}"]
        self._frame_csv.write(",".join(row) + "\n")

    def _display(self) -> None:
        est = self._state_estimate
        hs = self._height_scan
        raw = self._raw_action

        # Detect phase from raw_action activity
        if np.max(np.abs(raw)) > 0.01:
            self._phase = "POLICY"
        elif np.max(np.abs(self._q_target)) > 0.01:
            self._phase = "WAIT"
        else:
            self._phase = "PASSV"

        lines = [
            "┌─── MONITOR ──" + f" PHASE: {self._phase:<6}" + "─" * 48 + "┐",
            f"│ VEL   cmd→      est→      raw(max→ {np.max(np.abs(raw)):+.2f})" + " " * 28 + "│",
            f"│  vx   {self._cmd_vel[0]:+8.3f}  {est[0]:+8.3f}  {raw[0]:+7.3f} {raw[1]:+7.3f} {raw[2]:+7.3f} {raw[3]:+7.3f} {raw[4]:+7.3f} {raw[5]:+7.3f}  │",
            f"│  vy   {self._cmd_vel[1]:+8.3f}  {est[1]:+8.3f}  {raw[6]:+7.3f} {raw[7]:+7.3f} {raw[8]:+7.3f} {raw[9]:+7.3f} {raw[10]:+7.3f} {raw[11]:+7.3f}  │",
            f"│  wz   {self._cmd_vel[2]:+8.3f}  {est[2]:+8.3f}" + " " * 42 + "│",
            f"│ IMU   gx={est[6]:+7.3f}  gy={est[7]:+7.3f}  gz={est[8]:+7.3f}  lin=({est[0]:+5.2f},{est[1]:+5.2f},{est[2]:+5.2f}) │",
        ]

        # Joints: YAML order, show pos_rel from obs
        jline = "│ JOINTS rel="
        for i, name in enumerate(_YAML_JOINT_NAMES):
            if i % 3 == 0:
                jline += f"  {name[:2]}.{name[3:5]}={self._obs_pos_rel[i]:+6.3f}"
        jline += " " * (67 - len(jline) + len("│ JOINTS")) + " │"
        lines.append(jline)
        tgt_line = "│ TARGET "
        for i, name in enumerate(_YAML_JOINT_NAMES):
            if i % 3 == 0:
                tgt_line += f"  {name[:2]}.{name[3:5]}={self._q_target[i]:+6.3f}"
        tgt_line += " " * (67 - len(tgt_line) + len("│ TARGET")) + " │"
        lines.append(tgt_line)

        lines += [
            f"│ HS     mean={np.mean(hs):.3f}  min={np.min(hs):.3f}  max={np.max(hs):.3f}" + " " * 24 + "│",
            "│" + "─" * 67 + "│",
        ]

        # Last 3 errors
        shown = 0
        for t, lv, msg in self._errors:
            if shown >= 3:
                break
            prefix = f"  [{t}] {lv}: "
            max_len = 64 - len(prefix)
            text = prefix + msg[:max_len]
            lines.append(f"│{text:<67}│")
            shown += 1
        if shown == 0:
            lines.append("│  (no errors)" + " " * 54 + "│")

        lines.append("└" + "─" * 67 + "┘")
        text = "\n".join(lines)

        if self._tty:
            self._tty.write(f"\033[2J\033[H{text}\n")
            self._tty.flush()
        else:
            print(text, flush=True)

    def destroy_node(self) -> None:
        self._frame_csv.close()
        self._err_csv.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
