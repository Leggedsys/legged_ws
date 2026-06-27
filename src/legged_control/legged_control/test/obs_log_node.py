"""
obs_log_node — logs the deployment-side policy observation (/observation) to CSV.

/observation is the single 46-dim frame the policy consumes (already scaled and
clipped, in POLICY joint order). Use this to compare the deployment obs against
the training-side obs distribution.

Layout (matches obs_assembler._build_obs):
   0:3   base_ang_vel (scaled)
   3:6   projected_gravity
   6:9   vel_cmd (scaled)
   9     height_cmd
  10:22  joint_pos_rel (policy order, scaled)
  22:34  joint_vel     (policy order, scaled)
  34:46  last_action   (policy order)

Output: ~/.legged_logs/obs_<timestamp>/obs.csv  (one row per /observation msg)
"""

from __future__ import annotations

import os
import time as _time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

SINGLE_OBS_DIM = 46

_POLICY_JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]


def _make_log_dir() -> str:
    ts = _time.strftime("%Y%m%d_%H%M%S")
    d = os.path.expanduser(f"~/.legged_logs/obs_{ts}")
    os.makedirs(d, exist_ok=True)
    return d


class ObsLogNode(Node):
    def __init__(self) -> None:
        super().__init__("obs_log_node")

        self.create_subscription(
            Float32MultiArray, "/observation", self._on_obs, 50
        )

        self._log_dir = _make_log_dir()
        self._csv = open(os.path.join(self._log_dir, "obs.csv"), "w", buffering=1)
        self._write_header()
        self._count = 0
        self.get_logger().info(f"obs_log_node ready — logging /observation to {self._log_dir}")

    def _write_header(self) -> None:
        hdr = ["t_mono", "t_wall"]
        hdr += ["ang_vel_x", "ang_vel_y", "ang_vel_z"]
        hdr += ["grav_x", "grav_y", "grav_z"]
        hdr += ["cmd_vx", "cmd_vy", "cmd_wz"]
        hdr += ["height_cmd"]
        hdr += [f"qpos_{n}" for n in _POLICY_JOINT_NAMES]
        hdr += [f"qvel_{n}" for n in _POLICY_JOINT_NAMES]
        hdr += [f"act_{n}" for n in _POLICY_JOINT_NAMES]
        self._csv.write(",".join(hdr) + "\n")

    def _on_obs(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < SINGLE_OBS_DIM:
            return
        obs = msg.data[:SINGLE_OBS_DIM]
        row = [f"{_time.monotonic():.6f}", f"{_time.time():.6f}"]
        row += [f"{float(v):.6f}" for v in obs]
        self._csv.write(",".join(row) + "\n")
        self._count += 1
        if self._count % 250 == 0:
            self.get_logger().info(f"obs_log_node: {self._count} frames logged",
                                   throttle_duration_sec=5.0)

    def destroy_node(self) -> None:
        try:
            self._csv.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ObsLogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
