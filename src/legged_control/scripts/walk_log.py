#!/usr/bin/env python3
"""walk_log — record commanded vs measured joints while walking, then analyze.

Run it, walk the robot for 10–20 s, Ctrl+C. It saves CSVs to
~/.legged_logs/walk_<ts>/ and prints:

  * per-joint RMS / peak tracking error (cmd q vs measured q)
  * jolt events: spikes of measured joint acceleration, with timing
  * whether jolts coincide with tau_ff hand-off slews (contact flips)
    or happen mid-swing — separates "force hand-off kicks the body"
    from "trajectory/PD can't track"
  * command-hold detection (repeated identical cmd → IK failures)

Usage (no rebuild needed):
    source install/setup.bash
    /usr/bin/python3 src/legged_control/scripts/walk_log.py
"""

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf",
]


class WalkLog(Node):
    def __init__(self) -> None:
        super().__init__("walk_log")
        self.cmd_rows: list[list[float]] = []   # t, q*12, dq*12, tau*12
        self.meas_rows: list[list[float]] = []  # t, q*12, dq*12
        self.create_subscription(JointState, "/joint_commands", self._on_cmd, 50)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_meas, 50
        )
        self._t0 = time.monotonic()
        self.get_logger().info("recording — walk now, Ctrl+C to stop & analyze")

    def _row(self, msg: JointState, with_tau: bool) -> list[float] | None:
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q = [float(msg.position[idx[j]]) for j in _JOINTS]
            dq = [float(msg.velocity[idx[j]]) for j in _JOINTS] \
                if len(msg.velocity) == len(msg.name) else [0.0] * 12
            row = [time.monotonic() - self._t0] + q + dq
            if with_tau:
                tau = [float(msg.effort[idx[j]]) for j in _JOINTS] \
                    if len(msg.effort) == len(msg.name) else [0.0] * 12
                row += tau
            return row
        except (KeyError, IndexError):
            return None

    def _on_cmd(self, msg: JointState) -> None:
        row = self._row(msg, with_tau=True)
        if row:
            self.cmd_rows.append(row)

    def _on_meas(self, msg: JointState) -> None:
        row = self._row(msg, with_tau=False)
        if row:
            self.meas_rows.append(row)


def _analyze(cmd: np.ndarray, meas: np.ndarray) -> None:
    tc, tm = cmd[:, 0], meas[:, 0]
    qc, dqc, tauc = cmd[:, 1:13], cmd[:, 13:25], cmd[:, 25:37]
    qm, dqm = meas[:, 1:13], meas[:, 13:25]

    # resample measured onto command timestamps
    qm_i = np.stack([np.interp(tc, tm, qm[:, j]) for j in range(12)], axis=1)
    dqm_i = np.stack([np.interp(tc, tm, dqm[:, j]) for j in range(12)], axis=1)
    err = qc - qm_i

    print("\n=== tracking error (cmd − meas), rad ===")
    print(f"{'joint':<10} {'RMS':>7} {'peak':>7}")
    for j, name in enumerate(_JOINTS):
        print(f"{name:<10} {np.sqrt(np.mean(err[:, j]**2)):7.4f} "
              f"{np.max(np.abs(err[:, j])):7.4f}")

    # command-hold detection: identical consecutive cmd q on any joint
    holds = np.sum(np.all(np.diff(qc, axis=0) == 0.0, axis=1))
    print(f"\ncommand holds (all-12 identical consecutive cmds): {holds} "
          f"of {len(qc)-1} ticks" + ("  ← IK failures?" if holds > 5 else ""))

    # jolt events: measured acceleration spikes (finite diff of measured dq)
    dt = np.diff(tc).clip(1e-4)
    acc = np.diff(dqm_i, axis=0) / dt[:, None]
    jolt = np.max(np.abs(acc), axis=1)
    # tau hand-off slew: max per-tick |Δtau| across joints
    tau_slew = np.max(np.abs(np.diff(tauc, axis=0)), axis=1) / dt

    thr = np.percentile(jolt, 99)
    events = np.where(jolt > thr)[0]
    # merge events closer than 50 ms
    merged = []
    for e in events:
        if not merged or tc[e] - tc[merged[-1]] > 0.05:
            merged.append(e)
    print(f"\n=== top jolt events (|q̈| > p99 = {thr:.0f} rad/s²) ===")
    print(f"{'t(s)':>7} {'|q̈|max':>9} {'joint':<10} {'tau_slew(Nm/s)':>14}  near_flip")
    n_near = 0
    for e in merged[:15]:
        j = int(np.argmax(np.abs(acc[e])))
        # tau slew within ±40 ms of the jolt
        w = (tc[:-1] > tc[e] - 0.04) & (tc[:-1] < tc[e] + 0.04)
        s = float(np.max(tau_slew[w])) if np.any(w) else 0.0
        near = s > 0.5 * np.percentile(tau_slew, 99)
        n_near += bool(near)
        print(f"{tc[e]:7.2f} {jolt[e]:9.0f} {_JOINTS[j]:<10} {s:14.0f}  "
              f"{'YES' if near else 'no'}")
    if merged:
        print(f"\n{n_near}/{len(merged[:15])} jolts coincide with tau hand-off "
              f"slews → {'force hand-off is the kick' if n_near > len(merged[:15])//2 else 'jolts are NOT force-flip driven'}")


def main() -> None:
    rclpy.init()
    node = WalkLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    out = os.path.expanduser(
        f"~/.legged_logs/walk_{datetime.now():%Y%m%d_%H%M%S}"
    )
    os.makedirs(out, exist_ok=True)
    for name, rows, hdr in (
        ("cmd", node.cmd_rows,
         ["t"] + [f"q_{j}" for j in _JOINTS] + [f"dq_{j}" for j in _JOINTS]
         + [f"tau_{j}" for j in _JOINTS]),
        ("meas", node.meas_rows,
         ["t"] + [f"q_{j}" for j in _JOINTS] + [f"dq_{j}" for j in _JOINTS]),
    ):
        with open(f"{out}/{name}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(rows)
    print(f"\nsaved {len(node.cmd_rows)} cmd / {len(node.meas_rows)} meas rows → {out}")

    if len(node.cmd_rows) > 100 and len(node.meas_rows) > 100:
        _analyze(np.array(node.cmd_rows), np.array(node.meas_rows))
    else:
        print("not enough data for analysis (need a few seconds of walking)")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
