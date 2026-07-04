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
from scipy.optimize import minimize


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
    contact_mask: list[bool],
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Build discrete-time state matrix Ad (12×12) and input matrix Bd (12×12)."""
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
        if not in_contact:
            continue
        Bc[6:9, 3*i:3*i+3] = I_world_inv @ _skew(r_foot)
        Bc[9:12, 3*i:3*i+3] = np.eye(3) / mass

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
    ) -> None:
        self._mass = mass
        self._I_body = np.asarray(inertia_body, dtype=float)
        self._dt = dt
        self._N = horizon
        self._r = r_weight
        self._f_min = f_min
        self._f_max = f_max
        self._mu = mu

        if q_weights is None:
            # roll/pitch/yaw, x/y/z, wx/wy/wz, vx/vy/vz
            q_weights = [200, 200, 100,  0, 0, 200,  1, 1, 1,  5, 5, 10]
        self._Q = np.diag(q_weights)
        self._R = np.eye(12) * r_weight

    def solve(
        self,
        state: np.ndarray,
        state_ref: np.ndarray,
        foot_positions_world: np.ndarray,
        contact_schedule: list[list[bool]],
    ) -> np.ndarray:
        """Solve MPC and return optimal GRF for the first step.

        Args:
            state:                 current 12-dim state [rpy, pos, ang_vel, lin_vel]
            state_ref:             desired 12-dim state (held constant over horizon)
            foot_positions_world:  4×3 foot positions in world frame (relative to CoM)
            contact_schedule:      list of N lists of 4 booleans (per-step contact)

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

        A_prod = np.eye(nx)
        for k in range(N):
            A_prod = Ad_list[k] @ A_prod
            Phi[k*nx:(k+1)*nx, :] = A_prod
            # gravity accumulation
            g_acc = np.zeros(nx)
            A_run = np.eye(nx)
            for j in range(k + 1):
                g_acc += A_run @ g_vec
                if j < k:
                    A_run = Ad_list[k - j - 1] @ A_run
            g_bias[k*nx:(k+1)*nx] = g_acc

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

        # Make symmetric (numerical noise)
        H = (H + H.T) * 0.5

        # Constraints: friction cone + normal force bounds per contact
        ineq_list = []
        for k in range(N):
            for i, in_contact in enumerate(contact_schedule[k]):
                base = k * nu + i * 3
                if not in_contact:
                    # force must be zero for swing legs — handled via eq constraint
                    continue
                # fz ≥ f_min
                ineq_list.append((base + 2, 1.0, self._f_min, None))
                # fz ≤ f_max
                ineq_list.append((base + 2, -1.0, -self._f_max, None))
                # |fx| ≤ μ*fz  → fx - μ*fz ≤ 0  and  -fx - μ*fz ≤ 0
                ineq_list.append((base, 1.0, None, (base + 2, self._mu)))
                ineq_list.append((base, -1.0, None, (base + 2, self._mu)))
                ineq_list.append((base + 1, 1.0, None, (base + 2, self._mu)))
                ineq_list.append((base + 1, -1.0, None, (base + 2, self._mu)))

        def objective(u):
            return 0.5 * u @ H @ u + f_vec @ u

        def gradient(u):
            return H @ u + f_vec

        constraints = []
        # Zero force for swing legs
        for k in range(N):
            for i, in_contact in enumerate(contact_schedule[k]):
                if not in_contact:
                    for d in range(3):
                        idx = k * nu + i * 3 + d
                        constraints.append({
                            "type": "eq",
                            "fun": lambda u, idx=idx: u[idx],
                            "jac": lambda u, idx=idx: np.eye(N * nu)[idx],
                        })

        # Friction + normal force bounds as inequality constraints
        for (fi, sign, lb, friction) in ineq_list:
            if lb is not None:
                constraints.append({
                    "type": "ineq",
                    "fun": lambda u, fi=fi, sign=sign, lb=lb: sign * u[fi] - lb,
                })
            elif friction is not None:
                fz_idx, mu = friction
                constraints.append({
                    "type": "ineq",
                    "fun": lambda u, fi=fi, sign=sign, fz_idx=fz_idx, mu=mu:
                        mu * u[fz_idx] - sign * u[fi],
                })

        u0 = np.zeros(N * nu)
        # Warm-start: set stance forces to body weight / n_contacts
        n_contacts_0 = max(1, sum(contact_schedule[0]))
        fz0 = self._mass * _G / n_contacts_0
        for i, in_contact in enumerate(contact_schedule[0]):
            if in_contact:
                u0[i * 3 + 2] = fz0

        result = minimize(
            objective,
            u0,
            jac=gradient,
            method="SLSQP",
            constraints=constraints,
            options={"maxiter": 50, "ftol": 1e-4},
        )

        return result.x[:nu]  # GRF for first step only
