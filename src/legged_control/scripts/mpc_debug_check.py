#!/usr/bin/env python3
"""mpc_debug_check — record /mpc_debug while walking, then answer:

  * how deep is the per-step sag really (h_meas dips below stance_h)
  * does commanded Σfz respond to the z error, at what effective N/m
  * how LATE is the response (cross-correlation lag, ms) — a stiffness
    knob that "does nothing" usually means the loop is lag-limited,
    not gain-limited
  * f_max saturation: ticks where any leg is pinned at the force clamp

Run, walk 15-20 s (include some standing), Ctrl+C. Raw CSV saved to
~/.legged_logs/mpc_debug_<ts>/ for offline digging.

Usage (rebuild needed once for the /mpc_debug publisher):
    source install/setup.bash
    /usr/bin/python3 src/legged_control/scripts/mpc_debug_check.py
"""

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

_FIELDS = [
    "h_meas", "vz_meas", "z_err", "sum_fz",
    "fz_FR", "fz_FL", "fz_RR", "fz_RL",
    "tau_blend", "roll", "pitch", "wx", "wy", "stance_h",
]
_F_MAX = 200.0  # SRBDMPC f_max — keep in sync with srbd_mpc.py


class MpcDebugCheck(Node):
    def __init__(self) -> None:
        super().__init__("mpc_debug_check")
        self.rows: list[list[float]] = []
        self.create_subscription(Float32MultiArray, "/mpc_debug", self._on_dbg, 50)
        self._t0 = time.monotonic()
        self.get_logger().info("recording /mpc_debug — walk now, Ctrl+C to analyze")

    def _on_dbg(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= len(_FIELDS):
            self.rows.append(
                [time.monotonic() - self._t0] + [float(v) for v in msg.data[: len(_FIELDS)]]
            )


def _analyze(d: np.ndarray) -> None:
    t = d[:, 0]
    col = {n: d[:, i + 1] for i, n in enumerate(_FIELDS)}
    dt = float(np.median(np.diff(t)))
    active = col["tau_blend"] > 0.9
    if active.sum() < 200:
        print("tau_blend was <0.9 almost the whole time — record with tau_ff fully on")
        return
    h, z_err, sfz = col["h_meas"][active], col["z_err"][active], col["sum_fz"][active]
    ta = t[active]

    print(f"\n{len(d)} ticks over {t[-1]:.1f}s (dt≈{dt*1000:.0f}ms), "
          f"{active.sum()} with tau fully blended")

    # ── sag depth: h_meas relative to stance_h ──
    sag = col["stance_h"][active] - h
    print(f"\nheight error (stance_h − h_meas), + = sagging:")
    print(f"  mean {np.mean(sag)*1000:+6.1f} mm   p95 {np.percentile(sag, 95)*1000:+6.1f} mm"
          f"   max {np.max(sag)*1000:+6.1f} mm   min {np.min(sag)*1000:+6.1f} mm")

    # ── does Σfz respond to z_err, and how late ──
    ze = z_err - np.mean(z_err)
    fzc = sfz - np.mean(sfz)
    if np.std(ze) < 1e-4:
        print("\nz_err ≈ constant — z_fb disabled or robot never sagged; no gain fit")
    else:
        gain = float(np.dot(fzc, -ze) / np.dot(ze, ze))
        print(f"\neffective z response: {gain:.0f} N/m "
              f"(z_err std {np.std(ze)*1000:.1f} mm, Σfz std {np.std(fzc):.1f} N)")
        max_shift = int(0.3 / dt)
        lags, corrs = [], []
        for s in range(-max_shift, max_shift + 1):
            a = -ze[max_shift : len(ze) - max_shift]
            b = fzc[max_shift + s : len(fzc) - max_shift + s]
            n = min(len(a), len(b))
            if n < 100 or np.std(a[:n]) < 1e-9 or np.std(b[:n]) < 1e-9:
                continue
            lags.append(s * dt * 1000)
            corrs.append(float(np.corrcoef(a[:n], b[:n])[0, 1]))
        if corrs:
            i = int(np.argmax(corrs))
            print(f"  best correlation {corrs[i]:+.2f} at lag {lags[i]:+.0f} ms "
                  f"(+ = force lags the error)")
            if corrs[i] < 0.3:
                print("  ↳ weak correlation: Σfz is NOT tracking z_err — "
                      "loop broken or something else dominates the force")
            elif lags[i] > 60:
                print("  ↳ big lag: loop is delay-limited — raising z_fb_weight "
                      "cannot fix the per-step dip")

    # ── saturation ──
    fz_all = np.stack([col[f"fz_{l}"][active] for l in ["FR", "FL", "RR", "RL"]], axis=1)
    sat = np.any(fz_all >= _F_MAX - 1.0, axis=1)
    print(f"\nf_max saturation: {sat.sum()} / {len(fz_all)} ticks "
          f"({100.0*sat.mean():.1f}%)" + ("  ← clamped, weight increases are no-ops there"
                                          if sat.mean() > 0.05 else ""))

    # ── attitude while walking ──
    print(f"\nattitude (rad): roll std {np.std(col['roll'][active]):.3f}  "
          f"pitch std {np.std(col['pitch'][active]):.3f}  "
          f"wx std {np.std(col['wx'][active]):.2f}  wy std {np.std(col['wy'][active]):.2f}")
    if np.std(col["wx"][active]) < 1e-4 and np.std(col["wy"][active]) < 1e-4:
        print("  ↳ wx/wy ≈ 0 the whole time — rate feedback is seeing a dead gyro!")


def main() -> None:
    rclpy.init()
    node = MpcDebugCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    out = os.path.expanduser(f"~/.legged_logs/mpc_debug_{datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(out, exist_ok=True)
    with open(f"{out}/mpc_debug.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + _FIELDS)
        w.writerows(node.rows)
    print(f"\nsaved {len(node.rows)} rows → {out}")

    if len(node.rows) > 300:
        _analyze(np.array(node.rows))
    else:
        print("not enough data (need ~15 s with tau_ff on)")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
