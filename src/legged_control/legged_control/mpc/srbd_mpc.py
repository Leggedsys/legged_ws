"""srbd_mpc — Single Rigid Body Dynamics MPC.

Model (continuous time, 12 states):
  x = [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]
  u = [f0x,f0y,f0z, f1x,f1y,f1z, f2x,f2y,f2z, f3x,f3y,f3z]
      (ground reaction forces for legs FR,FL,RR,RL; zero if in swing)

Discretised with forward Euler at dt.
QP solved with scipy.optimize.minimize (SLSQP) over horizon N.

Reference:
  Di Carlo et al., "Dynamic Locomotion in the MIT Cheetah 3
  Through Convex Model-Predictive Control", IROS 2018.
"""

from __future__ import annotations

import numpy as np


_G = 9.81  # m/s²
_GRAVITY = np.array([0.0, 0.0, -_G])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0,  -v[2],  v[1]],
        [v[2],  0.0,  -v[0]],
        [-v[1], v[0],  0.0],
    ])


def _euler_to_R(rpy: np.ndarray) -> np.ndarray:
    """ZYX Euler angles → rotation matrix (body→world)."""
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,              cp*cr],
    ])


def _build_Ab(
    rpy: np.ndarray,
    inertia_body: np.ndarray,
    mass: float,
    foot_positions_world: np.ndarray,
    contact_mask: list[bool | float],
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Build discrete-time state matrix Ad (12×12) and input matrix Bd (12×12).

    contact_mask entries may be fractional (0..1): the share of this
    prediction step the foot spends in contact. The input block scales with
    it, so a contact flip sliding across a step boundary changes the model
    continuously instead of toggling a whole dt-wide bucket at once (which
    showed up as a periodic ~3 Nm/tick torque step at every hand-off once
    dt grew past the tick period). Booleans still work: True == 1.0.
    """
    R = _euler_to_R(rpy)
    I_world = R @ inertia_body @ R.T
    I_world_inv = np.linalg.inv(I_world)

    Ac = np.zeros((12, 12))
    # Θ̇ ≈ R_z^{-1} * ω  (yaw-only approximation for small roll/pitch)
    cy, sy = np.cos(float(rpy[2])), np.sin(float(rpy[2]))
    Rz_inv = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])
    Ac[0:3, 6:9] = Rz_inv
    # ṗ = v
    Ac[3:6, 9:12] = np.eye(3)

    Bc = np.zeros((12, 12))
    for i, (in_contact, r_foot) in enumerate(zip(contact_mask, foot_positions_world)):
        frac = float(in_contact)
        if frac <= 1e-6:
            continue
        Bc[6:9, 3*i:3*i+3] = frac * (I_world_inv @ _skew(r_foot))
        Bc[9:12, 3*i:3*i+3] = frac * np.eye(3) / mass

    # Forward Euler discretisation
    Ad = np.eye(12) + Ac * dt
    Bd = Bc * dt
    return Ad, Bd


class SRBDMPC:
    """Simplified SRBD-MPC for quadruped locomotion.

    Args:
        mass:          robot body mass (kg)
        inertia_body:  3×3 body-frame inertia tensor (kg⋅m²)
        dt:            control timestep (s)
        horizon:       MPC prediction horizon (steps)
        q_weights:     12-element state cost weights [rpy, pos, ang_vel, lin_vel]
        r_weight:      scalar GRF cost weight (per force component)
        f_min:         minimum normal contact force (N)
        f_max:         maximum normal contact force (N)
        mu:            friction coefficient
    """

    def __init__(
        self,
        mass: float,
        inertia_body: np.ndarray,
        dt: float = 0.02,
        horizon: int = 10,
        q_weights: list[float] | None = None,
        r_weight: float = 1e-4,
        f_min: float = 10.0,
        f_max: float = 200.0,
        mu: float = 0.6,
        slew_weight: float = 0.0,
    ) -> None:
        self._mass = mass
        self._I_body = np.asarray(inertia_body, dtype=float)
        self._dt = dt
        self._N = horizon
        self._r = r_weight
        self._f_min = f_min
        self._f_max = f_max
        self._mu = mu
        # Δu continuity: cost slew_weight·‖u₀ − u_prev‖² tying the first-step
        # force to the previously applied one. With a long horizon the QP
        # otherwise concentrates its hand-off pre-load into the last few
        # ticks before a contact flip (pump-then-drop, ~40 N/tick per leg);
        # this makes late slamming expensive so the transfer starts earlier.
        # Active only when solve() is given u_prev.
        self._slew_weight = slew_weight

        if q_weights is None:
            # roll/pitch/yaw, x/y/z, wx/wy/wz, vx/vy/vz
            q_weights = [200, 200, 100,  0, 0, 200,  1, 1, 1,  5, 5, 10]
        # float dtype is load-bearing: the node pokes _Q[i,i] at runtime and
        # an int array would silently truncate fractional weights to 0.
        self._Q = np.diag(np.asarray(q_weights, dtype=float))
        self._R = np.eye(12) * r_weight

    def solve(
        self,
        state: np.ndarray,
        state_ref: np.ndarray,
        foot_positions_world: np.ndarray,
        contact_schedule: list[list[bool]],
        u_prev: np.ndarray | None = None,
    ) -> np.ndarray:
        """Solve MPC and return optimal GRF for the first step.

        Args:
            state:                 current 12-dim state [rpy, pos, ang_vel, lin_vel]
            state_ref:             desired 12-dim state (held constant over horizon)
            foot_positions_world:  4×3 foot positions in world frame (relative to CoM)
            contact_schedule:      list of N lists of 4 contact shares — bool
                                   or float 0..1 (fraction of the step in
                                   contact); True == 1.0
            u_prev:                previous solve's 12-dim GRF; enables the
                                   Δu continuity cost (see slew_weight)

        Returns:
            12-element GRF vector [f0x,f0y,f0z, ..., f3x,f3y,f3z] for step 0
        """
        N = self._N
        nx, nu = 12, 12

        # Build prediction matrices
        Ad_list, Bd_list = [], []
        for k in range(N):
            rpy_k = state[:3] if k == 0 else state_ref[:3]
            Ad, Bd = _build_Ab(
                rpy_k, self._I_body, self._mass,
                foot_positions_world, contact_schedule[k], self._dt,
            )
            Ad_list.append(Ad)
            Bd_list.append(Bd)

        # Gravity correction: add gravity acceleration to state propagation
        g_vec = np.zeros(nx)
        g_vec[9:12] = _GRAVITY * self._dt  # applied each step

        # Condensed QP: decision variable u = [u0, u1, ..., u_{N-1}] ∈ R^{N*nu}
        # Cost: Σ (Φk*x0 + Γ*u + g_bias - x_ref)^T Q (…) + u^T blkdiag(R) u
        # Build Φ (N*nx × nx) and Γ (N*nx × N*nu)

        Phi = np.zeros((N * nx, nx))
        Gamma = np.zeros((N * nx, N * nu))
        g_bias = np.zeros(N * nx)

        # Build Phi and g_bias with O(N) recursion:
        #   x_{k+1} = Ad_k @ x_k + Bd_k @ u_k + g_vec
        #   Phi_k   = Ad_k @ Phi_{k-1}
        #   g_bias_k = Ad_k @ g_bias_{k-1} + g_vec
        A_prod = np.eye(nx)
        g_bias_k = np.zeros(nx)
        for k in range(N):
            A_prod   = Ad_list[k] @ A_prod
            g_bias_k = Ad_list[k] @ g_bias_k + g_vec
            Phi[k*nx:(k+1)*nx, :] = A_prod
            g_bias[k*nx:(k+1)*nx] = g_bias_k

        for k in range(N):
            A_run = np.eye(nx)
            for j in range(k, N):
                Gamma[j*nx:(j+1)*nx, k*nu:(k+1)*nu] = A_run @ Bd_list[k]
                if j < N - 1:
                    A_run = Ad_list[j + 1] @ A_run

        # Build block-diagonal Q and R
        Q_bar = np.kron(np.eye(N), self._Q)
        R_bar = np.kron(np.eye(N), self._R)

        x_ref_bar = np.tile(state_ref, N)
        e0 = Phi @ state + g_bias - x_ref_bar

        H = Gamma.T @ Q_bar @ Gamma + R_bar
        f_vec = Gamma.T @ Q_bar @ e0

        if u_prev is not None and self._slew_weight > 0.0:
            # + slew_weight·‖u₀ − u_prev‖² → quadratic term on the first
            # block, linear pull toward u_prev. Legs with no step-0 contact
            # are excluded: their u₀ is zeroed by projection anyway and a
            # pull toward stale force would bias the redistribution.
            up = np.asarray(u_prev, dtype=float).copy()
            for i, c in enumerate(contact_schedule[0]):
                if float(c) <= 1e-6:
                    up[3*i:3*i+3] = 0.0
            H[:nu, :nu] += self._slew_weight * np.eye(nu)
            f_vec[:nu] -= self._slew_weight * up

        # Make symmetric (numerical noise)
        H = (H + H.T) * 0.5

        # Unconstrained QP solution: u* = -H^{-1} f
        # Much faster than SLSQP with Python lambda constraints (~1ms vs ~1000ms).
        # Constraints are enforced afterwards by projection (per-leg clamp).
        try:
            u_opt = np.linalg.solve(H, -f_vec)
        except np.linalg.LinAlgError:
            u_opt = np.linalg.lstsq(H, -f_vec, rcond=None)[0]

        # Project onto constraint set per step per leg:
        #   swing legs → zero force
        #   stance legs → fz ∈ [f_min, f_max], |fx|/|fy| ≤ μ*fz
        # Fractional contact scales the force bounds with the contact share,
        # so the feasible set (and thus the projected solution) shrinks to
        # zero continuously as a foot leaves contact.
        for k in range(N):
            for i, in_contact in enumerate(contact_schedule[k]):
                frac = float(in_contact)
                base = k * nu + i * 3
                if frac <= 1e-6:
                    u_opt[base:base + 3] = 0.0
                    continue
                fz = float(np.clip(u_opt[base + 2], frac * self._f_min, frac * self._f_max))
                f_xy_max = self._mu * fz
                fx = float(np.clip(u_opt[base],     -f_xy_max, f_xy_max))
                fy = float(np.clip(u_opt[base + 1], -f_xy_max, f_xy_max))
                u_opt[base]     = fx
                u_opt[base + 1] = fy
                u_opt[base + 2] = fz

        return u_opt[:nu]  # GRF for first step only
