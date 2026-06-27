"""
leg_track_node — drives one leg through a sinusoidal trajectory and logs
commanded vs measured joint positions, for PD tracking analysis.

Does NOT run the policy. Useful for:
  - characterizing motor bandwidth (frequency sweep / chirp)
  - verifying PD gain tuning with a known reference

Publishes /joint_commands (URDF frame, all 12 joints) at ~50 Hz.
  Non-target legs are held at their observed initial positions.

Modes (param `mode`):
  sine  — fixed-frequency sinusoid (uses `freq`)
  chirp — linear frequency sweep from `freq` to `freq_end` over `duration`
          (best for bandwidth: see where tracking amplitude/phase degrades)

Output (per run): ~/.legged_logs/leg_track_<ts>/
    leg_track.csv   — t_mono, cmd_<joint>×12, act_<joint>×12
    leg_track.png   — cmd vs act over time, one subplot per moving joint

Params (override with -p):
    leg          default "FR"
    mode         default "chirp"   sine | chirp
    freq         default 0.5       Hz  — start freq (sine: fixed; chirp: sweep from)
    freq_end     default 8.0       Hz  — chirp end freq (ignored in sine mode)
    hip_amp      default 0.25      rad
    thigh_amp    default 0.25      rad
    calf_amp     default 0.25      rad
    duration     default 15.0      s   — must be > 0 in chirp mode
"""

from __future__ import annotations

import os
import time as _time

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


def _load_joint_limits() -> dict[str, tuple[float, float]]:
    share = get_package_share_directory("legged_control")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        cfg = yaml.safe_load(f)
    limits: dict[str, tuple[float, float]] = {}
    for j in cfg.get("joints", []):
        name = j.get("name", "")
        if name in _YAML_JOINT_NAMES:
            limits[name] = (float(j.get("q_min", -10.0)), float(j.get("q_max", 10.0)))
    return limits


def _make_log_dir() -> str:
    ts = _time.strftime("%Y%m%d_%H%M%S")
    d = os.path.expanduser(f"~/.legged_logs/leg_track_{ts}")
    os.makedirs(d, exist_ok=True)
    return d


class LegTrackNode(Node):
    def __init__(self) -> None:
        super().__init__("leg_track_node")

        self.declare_parameter("leg", "FR")
        self.declare_parameter("mode", "chirp")
        self.declare_parameter("freq", 0.5)
        self.declare_parameter("freq_end", 8.0)
        self.declare_parameter("hip_amp", 0.25)
        self.declare_parameter("thigh_amp", 0.25)
        self.declare_parameter("calf_amp", 0.25)
        self.declare_parameter("duration", 15.0)

        _leg = str(self.get_parameter("leg").value).upper()
        if _leg not in ("FR", "FL", "RR", "RL"):
            self.get_logger().error(f"invalid leg '{_leg}', using FR")
            _leg = "FR"

        self._target_leg = _leg
        self._mode = str(self.get_parameter("mode").value)
        self._freq = float(self.get_parameter("freq").value)
        self._freq_end = float(self.get_parameter("freq_end").value)
        self._amps = {
            f"{_leg}_hip":   float(self.get_parameter("hip_amp").value),
            f"{_leg}_thigh": float(self.get_parameter("thigh_amp").value),
            f"{_leg}_calf":  float(self.get_parameter("calf_amp").value),
        }
        self._duration = float(self.get_parameter("duration").value)

        self._q_limits = _load_joint_limits()
        lo = self._q_limits.get(f"{_leg}_hip", (-10, 10))
        self.get_logger().info(
            f"hardware limits for {_leg}: "
            f"hip=[{lo[0]:.1f},{lo[1]:.1f}] "
            f"thigh=[{self._q_limits.get(f'{_leg}_thigh',(-10,10))[0]:.1f},{self._q_limits.get(f'{_leg}_thigh',(-10,10))[1]:.1f}] "
            f"calf=[{self._q_limits.get(f'{_leg}_calf',(-10,10))[0]:.1f},{self._q_limits.get(f'{_leg}_calf',(-10,10))[1]:.1f}]"
        )

        self._act = np.full(12, np.nan, dtype=np.float64)
        self._act_seen = False
        self._centers: dict[str, float] = {}

        self._start_time: float | None = None
        self._rows: list[list[str]] = []

        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joints, 10,
        )

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f"leg_track_node ready — leg={_leg} mode={self._mode} "
            f"f={self._freq}→{self._freq_end}Hz "
            f"amp=({self._amps[f'{_leg}_hip']},{self._amps[f'{_leg}_thigh']},{self._amps[f'{_leg}_calf']}) "
            f"dur={self._duration}s"
        )

        if self._duration > 0:
            self.create_timer(0.1, self._check_duration)

    def _on_joints(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            if n in _YAML_JOINT_NAMES:
                self._act[_YAML_JOINT_NAMES.index(n)] = float(p)
                self._act_seen = True
                if n not in self._centers:
                    self._centers[n] = float(p)

    def _tick(self) -> None:
        if not self._act_seen:
            return

        if self._start_time is None:
            self._start_time = _time.monotonic()
            # lock centers from first seen position
            for n in _YAML_JOINT_NAMES:
                if n not in self._centers:
                    self._centers[n] = float(self._act[_YAML_JOINT_NAMES.index(n)])

        t = _time.monotonic() - self._start_time
        dur = max(self._duration, 1e-6)
        if self._mode == "chirp":
            f0 = self._freq
            f1 = self._freq_end
            phase = f0 * t + (f1 - f0) * t * t / (2.0 * dur)
        else:
            phase = self._freq * t
        omega_phase = 2.0 * np.pi * phase

        cmd = self._act.copy()
        for joint_name in _YAML_JOINT_NAMES:
            idx = _YAML_JOINT_NAMES.index(joint_name)
            amp = self._amps.get(joint_name, 0.0)
            if amp > 0.0:
                center = self._centers.get(joint_name, 0.0)
                raw = center + amp * np.sin(omega_phase)
                lo, hi = self._q_limits.get(joint_name, (-10.0, 10.0))
                cmd[idx] = np.clip(raw, lo + 0.01, hi - 0.01)

        msg = JointState()
        msg.name = list(_YAML_JOINT_NAMES)
        msg.position = [float(v) for v in cmd]
        self._pub.publish(msg)

        row = [f"{_time.monotonic():.6f}"]
        row += [f"{v:.6f}" for v in cmd]
        row += [f"{self._act[i]:.6f}" for i in range(12)]
        self._rows.append(row)

    def _check_duration(self) -> None:
        if self._start_time is not None and _time.monotonic() - self._start_time >= self._duration:
            self.get_logger().info(f"duration {self._duration}s reached — stopping")
            self._finish()
            rclpy.shutdown()

    def _finish(self) -> None:
        log_dir = _make_log_dir()

        csv_path = os.path.join(log_dir, "leg_track.csv")
        with open(csv_path, "w") as f:
            hdr = ["t_mono"] + [f"cmd_{n}" for n in _YAML_JOINT_NAMES] + [f"act_{n}" for n in _YAML_JOINT_NAMES]
            f.write(",".join(hdr) + "\n")
            for row in self._rows:
                f.write(",".join(row) + "\n")
        self.get_logger().info(f"CSV saved: {csv_path}")

        try:
            self._plot(log_dir)
        except Exception as e:
            self.get_logger().error(f"plot failed (no matplotlib?): {e}")

    def _plot(self, log_dir: str) -> None:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if not self._rows:
            return

        data = np.array([[float(v) for v in r] for r in self._rows])
        t = data[:, 0] - data[0, 0]
        n_joints = len(_YAML_JOINT_NAMES)
        cmd = data[:, 1 : 1 + n_joints]
        act = data[:, 1 + n_joints : 1 + 2 * n_joints]

        moving = [i for i, jname in enumerate(_YAML_JOINT_NAMES) if jname in self._amps and self._amps[jname] > 0.0]
        n = max(len(moving), 1)

        fig, axes = plt.subplots(n, 1, figsize=(12, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]

        for ax_i, jidx in enumerate(moving):
            jname = _YAML_JOINT_NAMES[jidx]
            ax = axes[ax_i]
            ax.plot(t, cmd[:, jidx], "b-", alpha=0.7, linewidth=0.8, label="cmd")
            ax.plot(t, act[:, jidx], "r-", alpha=0.7, linewidth=0.8, label="act")
            err = cmd[:, jidx] - act[:, jidx]
            ax.set_ylabel(f"{jname} (rad)")
            rms = np.sqrt(np.mean(err**2))
            max_e = np.max(np.abs(err))
            freq_str = f"{self._freq}→{self._freq_end}Hz" if self._mode == "chirp" else f"{self._freq}Hz"
            ax.set_title(f"{jname}  {freq_str}  RMS={rms:.4f}  max|e|={max_e:.4f}")

            # plot a vertical span highlighting high-freq region (last 20% for chirp)
            if self._mode == "chirp":
                ax.axvspan(t[-1] * 0.8, t[-1], alpha=0.06, color="red")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel("time (s)")
        mode_str = f"{self._mode}: {self._freq}→{self._freq_end}Hz" if self._mode == "chirp" else f"fixed {self._freq}Hz"
        fig.suptitle(
            f"leg_track  {self._target_leg}  {mode_str}  "
            f"amp=({self._amps.get(f'{self._target_leg}_hip',0)}, "
            f"{self._amps.get(f'{self._target_leg}_thigh',0)}, "
            f"{self._amps.get(f'{self._target_leg}_calf',0)})",
            fontsize=11,
        )
        fig.tight_layout(rect=[0, 0, 1, 0.96])

        png_path = os.path.join(log_dir, "leg_track.png")
        fig.savefig(png_path, dpi=120)
        plt.close(fig)
        self.get_logger().info(f"plot saved: {png_path}")

    def destroy_node(self) -> None:
        self._finish()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LegTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
