"""terrain_estimator — ground-plane estimate from stance-foot kinematics.

The gait otherwise assumes flat ground at −stance_height in the body frame;
on a slope that assumption fails hardest at the transition: walking onto an
incline the leading feet strike the ground early (mid-descent impact),
walking off it they reach the planned height and find nothing (the body
falls into the step). This estimator closes that hole with data the stack
already has — forward kinematics of the loaded feet.

Frames:
  * Anchors are stored in the HEADING frame (body frame rotated by the
    IMU roll/pitch, yaw = 0): feet planted on the ground barely move there
    while the body rocks, so anchors sampled at different times stay
    mutually consistent. The fitted plane's slope is therefore a WORLD
    slope — directly usable as the MPC attitude reference.
  * Foot targets live in the body/hip frame, so the plane is converted
    back through the current body rotation for the per-leg z offsets.

Steady state on a slope is deliberately a no-op for the foot offsets: once
the body rides parallel to the incline, the feet are coplanar with the
BODY-frame ground plane and dz → 0 — flat-ground behavior, which is the
hardware-validated baseline, is recovered bit-for-bit (slope ≈ 0 ⇒ every
output of this module is ≈ 0).
"""

from __future__ import annotations

import math

import numpy as np

# Slope clamp: tan(20°). The robot is asked to handle 15°; anything the fit
# reports beyond 20° is far more likely estimator trouble (kinematic error,
# a slipping foot) than real terrain — cap it before it steers the feet.
_SLOPE_MAX = 0.364
# Per-foot z offset clamp. At the 20° slope cap the diagonal-most foot
# (|x|+|y| ≈ 0.25 m from center) needs ~6.7 cm; beyond that the IK runs out
# of leg on the extended side.
_DZ_MAX = 0.07
# Anchors only update while the leg's stance load ramp says the foot is
# truly planted — the same trust weighting the z feedback uses.
_W_TRUST = 0.5
# Minimum anchor spread for a trustworthy fit: guards the lstsq against a
# degenerate geometry (should not happen with 4 legs, but a NaN here would
# steer every foot).
_MIN_SPREAD = 0.03


def dz_on_plane(a_b: float, b_b: float, x: float, y: float) -> float:
    """Foot-target z offset at body-frame (x, y) for plane coefficients
    (a_b, b_b), clamped so the IK can always reach the extended side."""
    return float(np.clip(a_b * x + b_b * y, -_DZ_MAX, _DZ_MAX))


class TerrainEstimator:
    """Rolling ground-plane fit z = a·x + b·y + c over per-leg foot anchors."""

    def __init__(
        self,
        foot_xy: dict[str, tuple[float, float]],
        stance_height: float = 0.27,
        lp_tau: float = 0.4,
    ) -> None:
        """foot_xy: nominal body-frame (x, y) per leg — the flat-ground
        anchor pattern used at reset; also fixes the leg-name set."""
        self._foot_xy = {leg: (float(x), float(y)) for leg, (x, y) in foot_xy.items()}
        self._lp_tau = float(lp_tau)
        self._anchors: dict[str, np.ndarray] = {}
        self._coeff = np.zeros(2)  # LP'd world-frame [a, b]
        self.reset(stance_height)

    def reset(self, stance_height: float) -> None:
        """Back to flat ground at the given height; slope decays via the LP
        (outputs are continuous through a reset, not stepped)."""
        for leg, (x, y) in self._foot_xy.items():
            self._anchors[leg] = np.array([x, y, -float(stance_height)])

    def update(
        self,
        foot_body: dict[str, np.ndarray],
        weights: dict[str, float],
        R_body: np.ndarray,
        dt: float,
    ) -> None:
        """Refresh anchors from loaded feet and low-pass the plane fit.

        foot_body: measured FK foot position per leg (body frame)
        weights:   per-leg stance load ramp (0..1); anchors update > 0.5
        R_body:    body→world rotation, yaw-free (from projected gravity)
        """
        for leg, w in weights.items():
            if float(w) > _W_TRUST and leg in foot_body:
                self._anchors[leg] = R_body @ np.asarray(foot_body[leg], dtype=float)

        pts = np.array([self._anchors[leg] for leg in self._foot_xy])
        if pts[:, 0].std() < _MIN_SPREAD or pts[:, 1].std() < _MIN_SPREAD:
            return  # hold previous slope rather than fit a degenerate cloud
        A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
        sol, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
        target = np.clip(sol[:2], -_SLOPE_MAX, _SLOPE_MAX)
        a_lp = min(1.0, float(dt) / self._lp_tau)
        self._coeff += a_lp * (target - self._coeff)

    @property
    def world_slope(self) -> np.ndarray:
        """LP'd world-frame slope coefficients [a, b] of z = a·x + b·y + c."""
        return self._coeff.copy()

    def _normal_world(self) -> np.ndarray:
        a, b = self._coeff
        n = np.array([-a, -b, 1.0])
        return n / np.linalg.norm(n)

    def ref_attitude(self) -> tuple[float, float]:
        """(roll, pitch) that lay the body parallel to the terrain.

        Solves R(roll, pitch, yaw=0)·ẑ = n̂ for the ZYX Euler convention of
        _euler_to_R (third column [sp·cr, −sr, cp·cr]). Uphill along +x
        (a > 0) yields pitch < 0 = nose up in this convention.
        """
        n = self._normal_world()
        roll = -math.asin(float(np.clip(n[1], -1.0, 1.0)))
        cr = math.cos(roll)
        pitch = math.asin(float(np.clip(n[0] / max(cr, 1e-6), -1.0, 1.0)))
        return roll, pitch

    def body_plane(self, R_body: np.ndarray) -> tuple[float, float]:
        """Terrain slope coefficients (a_b, b_b) in the CURRENT body frame.

        The world plane normal rotated into the body frame; with the body
        already parallel to the terrain this returns ≈ (0, 0) and the foot
        offsets vanish — flat-ground behavior.
        """
        n_b = R_body.T @ self._normal_world()
        nz = max(float(n_b[2]), 0.5)  # plane can't tilt > 60° vs body — clamp
        return -float(n_b[0]) / nz, -float(n_b[1]) / nz

    def dz(self, x: float, y: float, R_body: np.ndarray) -> float:
        """Per-foot z offset (body frame) putting the target on the terrain
        plane, relative to the body-center ground level (the mean height
        stays owned by stance_height / the z feedback — no double control)."""
        a_b, b_b = self.body_plane(R_body)
        return dz_on_plane(a_b, b_b, float(x), float(y))
