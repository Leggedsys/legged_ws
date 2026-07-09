#!/usr/bin/env python3
"""height_check — live commanded vs measured body height from joint angles.

Subscribes /joint_commands (commanded q, URDF frame) and
/joint_states_aggregated (measured q, URDF frame), runs FK on both, and
prints per-leg body height (−foot z) twice a second:

    cmd  FR 0.270 FL 0.270 RR 0.270 RL 0.270 | mean 0.270
    meas FR 0.281 FL 0.279 RR 0.276 RL 0.277 | mean 0.278   Δ +8.3 mm

Mass calibration procedure (robot standing in WALK balance, no cmd_vel):
  1. tau_ff on  → read mean measured height H_on
  2. ros2 param set /mpc_node tau_ff_enabled false → wait 1 s → read H_off
  3. true mass = mass_param × (H_cmd − H_off) / (H_on − H_off)
     (H_cmd = commanded mean height, normally 0.270)
  Correctly calibrated: H_on ≈ H_cmd. H_on above H_cmd → mass param too big.

Run (no rebuild needed):
    source install/setup.bash
    /usr/bin/python3 src/legged_control/scripts/height_check.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from legged_control.kinematics import forward_kinematics

_LEGS = ["FR", "FL", "RR", "RL"]
_JOINTS = ["hip", "thigh", "calf"]


def _heights(q: dict[str, float]) -> dict[str, float] | None:
    out = {}
    for leg in _LEGS:
        try:
            joints = tuple(q[f"{leg}_{j}"] for j in _JOINTS)
        except KeyError:
            return None
        p = forward_kinematics(leg, joints)
        out[leg] = -float(p[2])
    return out


class HeightCheck(Node):
    def __init__(self) -> None:
        super().__init__("height_check")
        self._cmd: dict[str, float] = {}
        self._meas: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_commands", self._on_cmd, 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_meas, 10
        )
        self.create_timer(0.5, self._print)

    def _on_cmd(self, msg: JointState) -> None:
        self._cmd = dict(zip(msg.name, msg.position))

    def _on_meas(self, msg: JointState) -> None:
        self._meas = dict(zip(msg.name, msg.position))

    def _print(self) -> None:
        hc = _heights(self._cmd)
        hm = _heights(self._meas)
        if hc is None or hm is None:
            self.get_logger().info("waiting for /joint_commands + /joint_states_aggregated ...")
            return
        mc = float(np.mean(list(hc.values())))
        mm = float(np.mean(list(hm.values())))
        line_c = " ".join(f"{leg} {hc[leg]:.3f}" for leg in _LEGS)
        line_m = " ".join(f"{leg} {hm[leg]:.3f}" for leg in _LEGS)
        print(f"cmd  {line_c} | mean {mc:.3f}")
        print(f"meas {line_m} | mean {mm:.3f}   Δ {(mm - mc) * 1000:+.1f} mm\n")


def main() -> None:
    rclpy.init()
    node = HeightCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
