#!/usr/bin/env python3
"""impact_check — record a walk, then answer WHAT the clatter is:

  * touchdown slap?  impact events cluster within ±30 ms of a contact
    flip (from /mpc_debug fz) and repeat at the step rate (~4 Hz)
  * command staircase?  the commanded trajectory itself steps — per-tick
    Δq large enough that kp turns each tick into a torque impulse
  * tracking breakdown?  swing joints lag the command by a large error
    that discharges at touchdown (gain problem, not trajectory problem)
  * dq feedforward sanity: published dq vs finite-diff of published q —
    verifies the bridge fix is actually live on this build

Run, walk 10-20 s, Ctrl+C. Raw CSVs saved to
~/.legged_logs/impact_check_<ts>/ for offline digging.

Usage:
    source install/setup.bash
    /usr/bin/python3 src/legged_control/scripts/impact_check.py
"""

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

_JOINTS = [
    f"{leg}_{part}"
    for leg in ("FR", "FL", "RR", "RL")
    for part in ("hip", "thigh", "calf")
]
_FZ_LEGS = ["FR", "FL", "RR", "RL"]  # /mpc_debug [4:8]


class ImpactCheck(Node):
    def __init__(self) -> None:
        super().__init__("impact_check")
        self.cmd_rows: list[list[float]] = []    # t, q*12, dq*12, tau*12
        self.meas_rows: list[list[float]] = []   # t, q*12, dq*12
        self.dbg_rows: list[list[float]] = []    # t, fz*4, tau_blend, vmeas*3
        self._cmd_idx: list[int] | None = None
        self._meas_idx: list[int] | None = None
        self._t0 = time.monotonic()
        self.create_subscription(JointState, "/joint_commands", self._on_cmd, 50)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_meas, 200
        )
        self.create_subscription(Float32MultiArray, "/mpc_debug", self._on_dbg, 50)
        self.get_logger().info("recording — walk now, Ctrl+C to analyze")

    @staticmethod
    def _index(msg: JointState) -> list[int] | None:
        try:
            return [list(msg.name).index(j) for j in _JOINTS]
        except ValueError:
            return None

    def _on_cmd(self, msg: JointState) -> None:
        if self._cmd_idx is None:
            self._cmd_idx = self._index(msg)
        if self._cmd_idx is None:
            return
        t = time.monotonic() - self._t0
        q = [float(msg.position[i]) for i in self._cmd_idx]
        dq = (
            [float(msg.velocity[i]) for i in self._cmd_idx]
            if len(msg.velocity) == len(msg.name) else [0.0] * 12
        )
        tau = (
            [float(msg.effort[i]) for i in self._cmd_idx]
            if len(msg.effort) == len(msg.name) else [0.0] * 12
        )
        self.cmd_rows.append([t] + q + dq + tau)

    def _on_meas(self, msg: JointState) -> None:
        if self._meas_idx is None:
            self._meas_idx = self._index(msg)
        if self._meas_idx is None:
            return
        t = time.monotonic() - self._t0
        q = [float(msg.position[i]) for i in self._meas_idx]
        dq = (
            [float(msg.velocity[i]) for i in self._meas_idx]
            if len(msg.velocity) == len(msg.name) else [0.0] * 12
        )
        self.meas_rows.append([t] + q + dq)

    def _on_dbg(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 9:
            t = time.monotonic() - self._t0
            vmeas = (
                [float(v) for v in msg.data[14:17]]
                if len(msg.data) >= 17 else [0.0, 0.0, 0.0]
            )
            self.dbg_rows.append(
                [t] + [float(v) for v in msg.data[4:8]]
                + [float(msg.data[8])] + vmeas
            )


def _flip_times(dbg: np.ndarray) -> np.ndarray:
    """Times where any leg's commanded fz crosses zero (either direction)."""
    t = dbg[:, 0]
    flips = []
    for i in range(4):
        on = dbg[:, 1 + i] > 1.0
        idx = np.nonzero(on[1:] != on[:-1])[0]
        flips.extend(t[idx + 1].tolist())
    return np.sort(np.array(flips))


def _analyze(cmd: np.ndarray, meas: np.ndarray, dbg: np.ndarray) -> None:
    tc = cmd[:, 0]
    dt_cmd = np.diff(tc)
    print(f"\n== command stream ({len(cmd)} msgs) ==")
    print(f"arrival interval: median {np.median(dt_cmd)*1e3:.1f} ms, "
          f"p99 {np.percentile(dt_cmd, 99)*1e3:.1f} ms")

    # per-tick command steps, worst joint
    dq_cmd_fd = np.diff(cmd[:, 1:13], axis=0)  # rad per tick
    worst = np.unravel_index(np.argmax(np.abs(dq_cmd_fd)), dq_cmd_fd.shape)
    print(f"command staircase: p95 |Δq|/tick {np.percentile(np.abs(dq_cmd_fd), 95)*1e3:.1f} mrad, "
          f"max {np.abs(dq_cmd_fd[worst])*1e3:.1f} mrad ({_JOINTS[worst[1]]})")

    # dq feedforward sanity: published dq vs finite diff on nominal 10 ms
    dq_pub = cmd[1:, 13:25]
    dt_col = dt_cmd[:, None]
    resid = dq_pub - dq_cmd_fd / np.where(dt_col > 1e-4, dt_col, np.nan)
    ok = np.nanpercentile(np.abs(resid), 95)
    if np.all(np.abs(dq_pub) < 1e-9):
        print("dq feedforward: published dq is all-zero — bridge fix NOT live "
              "(old build, or publisher not filling velocity)")
    else:
        print(f"dq feedforward: published vs finite-diff p95 residual {ok:.2f} rad/s "
              f"({'consistent' if ok < 1.5 else 'INCONSISTENT — check bridge'})")

    tm = meas[:, 0]
    dt_m = float(np.median(np.diff(tm)))
    print(f"\n== measured stream ({len(meas)} msgs, {1.0/dt_m:.0f} Hz) ==")

    # impact events: robust spikes in Δdq (deceleration proxy), any joint
    ddq = np.abs(np.diff(meas[:, 13:25], axis=0))
    mag = ddq.max(axis=1)  # worst joint per sample
    thresh = np.median(mag) + 8.0 * (np.median(np.abs(mag - np.median(mag))) + 1e-9)
    hits = mag > thresh
    # merge hits closer than 30 ms into one event
    events = []
    for i in np.nonzero(hits)[0]:
        if not events or tm[i + 1] - events[-1] > 0.03:
            events.append(float(tm[i + 1]))
    ev = np.array(events)
    dur = tm[-1] - tm[0]
    print(f"impact events (|Δdq| > {thresh:.2f} rad/s/sample): "
          f"{len(ev)} in {dur:.1f} s = {len(ev)/max(dur,1e-9):.1f}/s")
    if len(ev) >= 4:
        iei = np.diff(ev)
        print(f"  repetition: median interval {np.median(iei)*1e3:.0f} ms "
              f"→ ~{1.0/np.median(iei):.1f} Hz  "
              f"(step rate for 0.5 s trot ≈ 4 Hz; 100 Hz ≈ command tick)")
        joint_hits = ddq[hits].argmax(axis=1)
        top = np.bincount(joint_hits, minlength=12).argmax()
        print(f"  loudest joint: {_JOINTS[top]}")
        if len(dbg) > 10:
            flips = _flip_times(dbg)
            if len(flips):
                d = np.min(np.abs(ev[:, None] - flips[None, :]), axis=1)
                near = float(np.mean(d < 0.03))
                print(f"  {near*100:.0f}% of events within ±30 ms of a contact flip "
                      f"({'touchdown slap dominates' if near > 0.6 else 'NOT touchdown-locked'})")

    # tau_ff churn — the calf is geared, so every sign reversal of its net
    # torque knocks through the backlash; commanded tau_ff stepping at each
    # contact flip is the prime suspect for a step-rate clack.
    if cmd.shape[1] >= 37:
        dtau = np.diff(cmd[:, 25:37], axis=0)
        worst_t = np.unravel_index(np.argmax(np.abs(dtau)), dtau.shape)
        calves = [2, 5, 8, 11]
        sign_x = 0.0
        for j in calves:
            s = np.sign(cmd[:, 25 + j])
            sign_x += float(np.sum(s[1:] * s[:-1] < 0))
        print("\n== commanded tau_ff ==")
        print(f"per-tick |Δtau|: p95 {np.percentile(np.abs(dtau), 95):.2f} Nm, "
              f"max {np.abs(dtau[worst_t]):.2f} Nm ({_JOINTS[worst_t[1]]})")
        print(f"calf tau sign crossings: {sign_x / max(tc[-1]-tc[0], 1e-9):.1f}/s "
              f"(each one is a backlash knock candidate)")

    # leg-odometry health — capture-point landing offset and braking steps
    # are inert (or actively harmful) if this reads ~0 while walking.
    if len(dbg) > 10 and dbg.shape[1] >= 9:
        walking = dbg[:, 5] > 0.9  # tau_blend
        if walking.sum() > 50:
            vxy = np.hypot(dbg[walking, 6], dbg[walking, 7])
            print("\n== leg-odometry v_meas (while tau_blend > 0.9) ==")
            print(f"|v_xy|: p50 {np.percentile(vxy, 50):.3f}  "
                  f"p95 {np.percentile(vxy, 95):.3f} m/s   "
                  f"wz range [{dbg[walking, 8].min():.2f}, {dbg[walking, 8].max():.2f}] rad/s")
            if np.percentile(vxy, 95) < 0.05:
                print("  ~ZERO while the robot moved → leg odometry broken on "
                      "hardware; set k_raibert 0 until fixed")

    # tracking error per joint group (interp command onto measured clock)
    print("\n== tracking (measured − commanded) ==")
    for gi, gname in ((0, "hip"), (1, "thigh"), (2, "calf")):
        errs = []
        for leg in range(4):
            j = leg * 3 + gi
            qc = np.interp(tm, tc, cmd[:, 1 + j])
            errs.append(np.abs(meas[:, 1 + j] - qc))
        e = np.concatenate(errs)
        print(f"  {gname:6s} p95 |err| {np.percentile(e, 95)*1e3:.0f} mrad")


def main() -> None:
    rclpy.init()
    node = ImpactCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(node.cmd_rows) < 100 or len(node.meas_rows) < 100:
        print("not enough data — is the robot walking and the stack running?")
        return

    out = os.path.expanduser(
        f"~/.legged_logs/impact_check_{datetime.now():%Y%m%d_%H%M%S}"
    )
    os.makedirs(out, exist_ok=True)
    for name, rows, hdr in (
        ("commands", node.cmd_rows,
         ["t"] + [f"q_{j}" for j in _JOINTS] + [f"dq_{j}" for j in _JOINTS]
         + [f"tau_{j}" for j in _JOINTS]),
        ("measured", node.meas_rows,
         ["t"] + [f"q_{j}" for j in _JOINTS] + [f"dq_{j}" for j in _JOINTS]),
        ("mpc_debug", node.dbg_rows,
         ["t"] + [f"fz_{l}" for l in _FZ_LEGS] + ["tau_blend", "vx", "vy", "wz"]),
    ):
        with open(os.path.join(out, f"{name}.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(rows)
    print(f"saved CSVs to {out}")

    _analyze(
        np.array(node.cmd_rows),
        np.array(node.meas_rows),
        np.array(node.dbg_rows) if node.dbg_rows else np.zeros((0, 9)),
    )


if __name__ == "__main__":
    main()
