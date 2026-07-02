"""
timing_diag_node — control pipeline latency profiler.

Subscribes to the key topics in the control chain and records monotonic
arrival time for each. Logs per-cycle timing breakdown to CSV.

Measured stages:
  t_motor   — /joint_states_aggregated  arrival (motor feedback ready)
  t_obs     — /observation              arrival (obs assembled)
  t_action  — /raw_policy_action        arrival (policy inference done)
  t_cmd     — /joint_commands           arrival (motor command published)

Derived deltas:
  obs_latency     = t_obs - t_motor      (obs assembly time)
  inference       = t_action - t_obs     (policy inference + decode)
  cmd_latency     = t_cmd - t_action     (decode + publish)
  cycle_time      = t_cmd - prev_t_cmd   (full control cycle)
  feedback_gap    = t_motor - prev_t_cmd (command → next feedback)

Output: ~/.legged_logs/timing_<ts>/timing.csv
Also prints periodic summary to log.
"""

from __future__ import annotations

import os
import time as _time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray


def _make_log_dir() -> str:
    ts = _time.strftime("%Y%m%d_%H%M%S")
    d = os.path.expanduser(f"~/.legged_logs/timing_{ts}")
    os.makedirs(d, exist_ok=True)
    return d


class TimingDiagNode(Node):
    def __init__(self) -> None:
        super().__init__("timing_diag_node")

        self._t_motor: float = 0.0
        self._t_obs: float = 0.0
        self._t_inf_start: float = 0.0
        self._t_action: float = 0.0
        self._t_cmd: float = 0.0
        self._t_motor_ros: int = 0
        self._t_obs_ros: int = 0
        self._t_action_ros: int = 0
        self._t_cmd_ros: int = 0

        self._last_t_cmd: float = 0.0
        self._cycle = 0
        self._hist: dict[str, list[float]] = {
            "obs_latency": [],
            "inference": [],
            "cmd_latency": [],
            "cycle_time": [],
            "feedback_gap": [],
        }

        self.create_subscription(JointState, "/joint_states_aggregated", self._on_motor, 10)
        self.create_subscription(Float32MultiArray, "/observation", self._on_obs, 10)
        self.create_subscription(Bool, "/policy_inference_start", self._on_inf_start, 10)
        self.create_subscription(Float32MultiArray, "/raw_policy_action", self._on_action, 10)
        self.create_subscription(JointState, "/joint_commands", self._on_cmd, 10)

        self._csv_file = os.path.join(_make_log_dir(), "timing.csv")
        self._csv = open(self._csv_file, "w", buffering=1)
        self._csv.write(
            "cycle,t_motor,t_obs,t_inf_start,t_action,t_cmd,"
            "obs_latency_ms,inf_prep_ms,model_ms,cmd_latency_ms,cycle_ms,feedback_gap_ms\n"
        )
        self.create_timer(2.0, self._report)
        self.get_logger().info(f"timing_diag_node ready → {self._csv_file}")

    def _on_motor(self, msg: JointState) -> None:
        self._t_motor = _time.monotonic()
        self._t_motor_ros = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _on_obs(self, msg: Float32MultiArray) -> None:
        self._t_obs = _time.monotonic()

    def _on_inf_start(self, msg: Bool) -> None:
        self._t_inf_start = _time.monotonic()

    def _on_action(self, msg: Float32MultiArray) -> None:
        self._t_action = _time.monotonic()

    def _on_cmd(self, msg: JointState) -> None:
        now = _time.monotonic()
        self._t_cmd = now
        self._t_cmd_ros = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self._t_motor == 0 or self._t_obs == 0 or self._t_action == 0:
            return

        obs_latency_ms = (self._t_obs - self._t_motor) * 1000.0
        inf_prep_ms = (self._t_inf_start - self._t_obs) * 1000.0
        model_ms = (self._t_action - self._t_inf_start) * 1000.0
        cmd_latency_ms = (self._t_cmd - self._t_action) * 1000.0
        cycle_ms = (self._t_cmd - self._last_t_cmd) * 1000.0 if self._last_t_cmd > 0 else 0.0
        feedback_gap_ms = (self._t_motor - self._last_t_cmd) * 1000.0 if self._last_t_cmd > 0 else 0.0

        self._cycle += 1
        self._last_t_cmd = self._t_cmd
        self._hist["obs_latency"].append(obs_latency_ms)
        self._hist["inference"].append(inf_prep_ms + model_ms)
        self._hist["cmd_latency"].append(cmd_latency_ms)
        self._hist["cycle_time"].append(cycle_ms)
        self._hist["feedback_gap"].append(feedback_gap_ms)

        self._csv.write(
            f"{self._cycle},"
            f"{self._t_motor:.6f},{self._t_obs:.6f},{self._t_inf_start:.6f},{self._t_action:.6f},{self._t_cmd:.6f},"
            f"{obs_latency_ms:.3f},{inf_prep_ms:.3f},{model_ms:.3f},{cmd_latency_ms:.3f},{cycle_ms:.3f},{feedback_gap_ms:.3f}\n"
        )

    def _report(self) -> None:
        if self._cycle < 10:
            return
        lines = [
            "── TIMING DIAG (2s summary) ──",
            f" cycles: {self._cycle}",
        ]
        for key, label in [
            ("obs_latency", "obs assembly (motor→obs)"),
            ("inference", "inference total (obs→action)"),
            ("cmd_latency", "decode+publish (action→cmd)"),
            ("cycle_time", "full cycle (cmd→cmd)"),
            ("feedback_gap", "feedback gap (cmd→motor)"),
        ]:
            vals = self._hist[key][-500:]
            if vals:
                a = np.array(vals)
                lines.append(f" {label:35s}  mean={a.mean():7.3f}ms  max={a.max():7.3f}ms  std={a.std():6.3f}ms")
        self.get_logger().info("\n".join(lines))

    def destroy_node(self) -> None:
        try:
            self._csv.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TimingDiagNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
