"""
joint_track_node — logs policy-commanded joint positions vs measured motor
positions to a CSV file, for tracking-error analysis while running the policy.

Both topics are in the URDF joint frame, so the values are directly comparable:
  /joint_commands            (sensor_msgs/JointState) — policy output target  (URDF)
  /joint_states_aggregated   (sensor_msgs/JointState) — measured motor pos    (URDF)

One CSV row is written per /joint_commands message (i.e. per policy output),
pairing the commanded position with the latest measured position.

Output: ~/.legged_logs/joint_track_<timestamp>/joint_track.csv
Columns: t_mono, t_wall, cmd_<joint>×12, act_<joint>×12, err_<joint>×12
"""

from __future__ import annotations

import os
import time as _time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf",
]


def _make_log_dir() -> str:
    ts = _time.strftime("%Y%m%d_%H%M%S")
    d = os.path.expanduser(f"~/.legged_logs/joint_track_{ts}")
    os.makedirs(d, exist_ok=True)
    return d


class JointTrackNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_track_node")

        self._cmd = np.full(12, np.nan, dtype=np.float64)
        self._act = np.full(12, np.nan, dtype=np.float64)
        self._act_seen = False

        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_actual, 10
        )
        self.create_subscription(
            JointState, "/joint_commands", self._on_command, 10
        )

        self._log_dir = _make_log_dir()
        self._csv = open(os.path.join(self._log_dir, "joint_track.csv"), "w", buffering=1)
        self._write_header()
        self.get_logger().info(f"joint_track_node ready — logging to {self._log_dir}")

    def _write_header(self) -> None:
        hdr = ["t_mono", "t_wall"]
        hdr += [f"cmd_{n}" for n in _YAML_JOINT_NAMES]
        hdr += [f"act_{n}" for n in _YAML_JOINT_NAMES]
        hdr += [f"err_{n}" for n in _YAML_JOINT_NAMES]
        self._csv.write(",".join(hdr) + "\n")

    def _on_actual(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            if n in _YAML_JOINT_NAMES:
                self._act[_YAML_JOINT_NAMES.index(n)] = float(p)
                self._act_seen = True

    def _on_command(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            if n in _YAML_JOINT_NAMES:
                self._cmd[_YAML_JOINT_NAMES.index(n)] = float(p)
        self._log_row()

    def _log_row(self) -> None:
        err = self._cmd - self._act
        row = [f"{_time.monotonic():.6f}", f"{_time.time():.6f}"]
        row += [f"{v:.6f}" for v in self._cmd]
        row += [f"{v:.6f}" for v in self._act]
        row += [f"{v:.6f}" for v in err]
        self._csv.write(",".join(row) + "\n")

    def destroy_node(self) -> None:
        try:
            self._csv.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = JointTrackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
