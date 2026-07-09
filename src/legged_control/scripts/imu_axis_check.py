#!/usr/bin/env python3
"""imu_axis_check — verify IMU angular-velocity axis mapping against gravity.

The MPC needs body rates for force-level attitude damping, but raw gyro axes
are mounting-dependent — a flipped sign turns damping into positive feedback
(observed on this robot). This tool derives ground truth from the gravity
vector itself: roll/pitch computed from projected_gravity, differentiated,
then correlated against each measured ang_vel channel.

Procedure:
  1. ros2 launch legged_control robot.launch.py mode:=passive   (zero torque)
  2. run this script; it records for 30 s
  3. tilt the robot by hand, slowly and repeatedly: nose down/up several
     times, then right side down/up several times (~1 s per tilt)
  4. read the verdict:
       ang_vel[i] ↔ roll_rate  corr +0.9x  → wx = +ang_vel[i]
       ang_vel[j] ↔ pitch_rate corr −0.9x  → wy = −ang_vel[j]
     |corr| < 0.7 → data too noisy / motion too fast, redo slower.

Usage:
    source install/setup.bash
    /usr/bin/python3 src/legged_control/scripts/imu_axis_check.py
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

_DURATION = 30.0


class ImuAxisCheck(Node):
    def __init__(self) -> None:
        super().__init__("imu_axis_check")
        self.rows: list[list[float]] = []  # t, wx?, wy?, wz?, roll, pitch
        self.create_subscription(
            Float32MultiArray, "/state_estimate", self._on_est, 50
        )
        self._t0 = time.monotonic()
        self.get_logger().info(
            f"recording {_DURATION:.0f}s — tilt the robot slowly: "
            "nose down/up ×3, then right side down/up ×3"
        )

    def _on_est(self, msg: Float32MultiArray) -> None:
        d = msg.data
        if len(d) < 9:
            return
        g = np.array(d[6:9], dtype=float)
        n = np.linalg.norm(g)
        if n < 0.5:
            return
        g /= n
        roll = float(np.arctan2(-g[1], -g[2]))
        pitch = float(np.arctan2(g[0], float(np.hypot(g[1], g[2]))))
        self.rows.append(
            [time.monotonic() - self._t0, d[3], d[4], d[5], roll, pitch]
        )


def main() -> None:
    rclpy.init()
    node = ImuAxisCheck()
    try:
        end = time.monotonic() + _DURATION
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    data = np.array(node.rows)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if len(data) < 200:
        print(f"only {len(data)} samples — is /state_estimate publishing?")
        return

    t = data[:, 0]
    w = data[:, 1:4]
    # ground-truth rates from gravity-derived angles (smoothed finite diff)
    dt = np.gradient(t)
    rates = {
        "roll_rate": np.gradient(data[:, 4]) / dt,
        "pitch_rate": np.gradient(data[:, 5]) / dt,
    }
    # light smoothing to tame differentiation noise
    k = np.ones(15) / 15.0
    for key in rates:
        rates[key] = np.convolve(rates[key], k, mode="same")
    w_s = np.stack([np.convolve(w[:, i], k, mode="same") for i in range(3)], axis=1)

    print(f"\n{len(data)} samples over {t[-1]:.1f}s")
    print(f"{'':>12} {'ang_vel[0]':>11} {'ang_vel[1]':>11} {'ang_vel[2]':>11}")
    best: dict[str, tuple[int, float]] = {}
    for name, r in rates.items():
        if np.std(r) < 0.02:
            print(f"{name:>12}  (not excited — tilt about this axis and redo)")
            continue
        corrs = [float(np.corrcoef(r, w_s[:, i])[0, 1]) for i in range(3)]
        print(f"{name:>12} " + " ".join(f"{c:+11.2f}" for c in corrs))
        i = int(np.argmax(np.abs(corrs)))
        best[name] = (i, corrs[i])

    print("\nverdict:")
    ok = True
    for name, axis in (("roll_rate", "wx"), ("pitch_rate", "wy")):
        if name not in best:
            ok = False
            continue
        i, c = best[name]
        if abs(c) < 0.7:
            print(f"  {axis}: ambiguous (|corr| {abs(c):.2f} < 0.7) — redo slower")
            ok = False
        else:
            sign = "+" if c > 0 else "-"
            print(f"  {axis} = {sign}ang_vel[{i}]   (corr {c:+.2f})")
    if ok and best:
        print("\npaste this verdict to Claude → wire body rates into the MPC "
              "state for force-level attitude damping.")


if __name__ == "__main__":
    main()
