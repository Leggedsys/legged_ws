"""wbc — Whole Body Controller using Pinocchio floating-base inverse dynamics.

Given MPC ground reaction forces f* and desired joint targets, computes joint
torques via the joint-space dynamics:

    τ = M_jj q̈_des + h_j − J_cj^T f*_stance

Requires ros-humble-pinocchio:
  sudo apt install ros-humble-pinocchio
"""

from __future__ import annotations
import sys
import numpy as np

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
try:
    import pinocchio as pin
    _PIN_AVAILABLE = True
except ImportError:
    _PIN_AVAILABLE = False


# YAML order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
# PIN order:  FL(0-2), FR(3-5), RL(6-8), RR(9-11)
# _YAML_TO_PIN[i] = Pinocchio joint-space index for YAML joint i
_YAML_TO_PIN = [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8]
_PIN_TO_YAML = [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8]  # self-inverse

# Pinocchio leg order (tree traversal)
_PIN_LEGS = ["FL", "FR", "RL", "RR"]

# MPC leg order
_MPC_LEGS = ["FR", "FL", "RR", "RL"]

# MPC index for each Pinocchio-order leg
_PIN_LEG_TO_MPC_IDX = {"FL": 1, "FR": 0, "RL": 3, "RR": 2}

# YAML joint slice per MPC leg index
_MPC_IDX_TO_YAML_SLICE = {0: slice(0, 3), 1: slice(3, 6), 2: slice(6, 9), 3: slice(9, 12)}

assert all(_YAML_TO_PIN[_PIN_TO_YAML[i]] == i for i in range(12)), \
    "_YAML_TO_PIN and _PIN_TO_YAML are not mutual inverses"

# GO-M8010-6 joint-side peak ≈ 23.7 Nm; stay 1.7 Nm below to avoid overcurrent trips.
_TAU_MAX = 22.0


class WBC:
    """Whole Body Controller using Pinocchio floating-base inverse dynamics."""

    def __init__(self, urdf_path: str, kp_swing: float = 800.0, kd_swing: float = 40.0):
        if not _PIN_AVAILABLE:
            raise RuntimeError(
                "pinocchio not available — install: sudo apt install ros-humble-pinocchio"
            )
        self._model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self._data  = self._model.createData()
        self._nv    = self._model.nv   # 18
        self._kp_sw = kp_swing
        self._kd_sw = kd_swing
        self._foot_ids: dict[str, int] = {}
        for leg in _PIN_LEGS:
            fid = self._model.getFrameId(f"{leg}_foot")
            if fid >= self._model.nframes:
                raise ValueError(f"Frame '{leg}_foot' not found in URDF")
            self._foot_ids[leg] = fid

    def solve(
        self,
        q_yaml: np.ndarray,            # (12,) joint positions, YAML order
        dq_yaml: np.ndarray,           # (12,) joint velocities, YAML order
        rpy: np.ndarray,               # (3,) roll/pitch/yaw
        base_vel_body: np.ndarray,     # (3,) base linear velocity, body frame
        base_ang_vel_body: np.ndarray, # (3,) base angular velocity, body frame
        f_mpc: np.ndarray,             # (12,) GRFs world frame, MPC order [FR,FL,RR,RL]×xyz
        contact: list[bool],           # 4 bools [FR,FL,RR,RL]
        q_swing_des: np.ndarray,       # (12,) desired joint pos for swing, YAML order
        dq_swing_des: np.ndarray,      # (12,) desired joint vel for swing, YAML order
    ) -> np.ndarray:                   # (12,) joint torques, YAML order
        """Compute joint torques via floating-base inverse dynamics."""
        q_pin, dq_pin = self._build_pin_state(q_yaml, dq_yaml, rpy, base_vel_body, base_ang_vel_body)

        pin.computeAllTerms(self._model, self._data, q_pin, dq_pin)
        M   = self._data.M        # 18×18
        h   = self._data.nle      # 18
        M_jj = M[6:, 6:]          # 12×12
        h_j  = h[6:]              # 12

        J_cj, f_stance = self._build_contact_jacobian(contact, f_mpc)
        q_ddot_des = self._build_q_ddot(contact, q_yaml, dq_yaml, q_swing_des, dq_swing_des)

        tau_pin = M_jj @ q_ddot_des + h_j
        if J_cj.shape[0] > 0:
            tau_pin = tau_pin - J_cj.T @ f_stance

        tau_yaml = np.zeros(12)
        for i_pin in range(12):
            tau_yaml[_PIN_TO_YAML[i_pin]] = tau_pin[i_pin]
        return np.clip(tau_yaml, -_TAU_MAX, _TAU_MAX)

    def _build_pin_state(self, q_yaml, dq_yaml, rpy, base_vel_body, base_ang_vel_body):
        r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
        cr, sr = np.cos(r/2), np.sin(r/2)
        cp, sp = np.cos(p/2), np.sin(p/2)
        cy, sy = np.cos(y/2), np.sin(y/2)
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        qw = cr*cp*cy + sr*sp*sy

        q_pin = pin.neutral(self._model)
        q_pin[3:7] = [qx, qy, qz, qw]
        for i_yaml, i_pin in enumerate(_YAML_TO_PIN):
            q_pin[7 + i_pin] = q_yaml[i_yaml]

        dq_pin = np.zeros(self._nv)
        # Pinocchio free-flyer generalized velocity dq[0:3] is in the LOCAL (body)
        # frame, verified experimentally: with 90° yaw, dq[0]=1 yields
        # getVelocity(LOCAL)=[1,0,0] not [0,1,0].  body-frame input is correct.
        dq_pin[0:3] = base_vel_body
        dq_pin[3:6] = base_ang_vel_body
        for i_yaml, i_pin in enumerate(_YAML_TO_PIN):
            dq_pin[6 + i_pin] = dq_yaml[i_yaml]

        return q_pin, dq_pin

    def _build_contact_jacobian(self, contact, f_mpc):
        J_rows, f_rows = [], []
        for leg in _PIN_LEGS:
            mpc_idx = _PIN_LEG_TO_MPC_IDX[leg]
            if not contact[mpc_idx]:
                continue
            frame_id = self._foot_ids[leg]
            J_full = pin.getFrameJacobian(
                self._model, self._data, frame_id, pin.LOCAL_WORLD_ALIGNED
            )
            J_rows.append(J_full[:3, 6:])   # 3×12, joint columns only
            f_rows.append(f_mpc[mpc_idx*3 : mpc_idx*3+3])
        if not J_rows:
            return np.zeros((0, 12)), np.zeros(0)
        return np.vstack(J_rows), np.concatenate(f_rows)

    def _build_q_ddot(self, contact, q_yaml, dq_yaml, q_sw_des, dq_sw_des):
        q_ddot_pin = np.zeros(12)
        for leg in _PIN_LEGS:
            mpc_idx = _PIN_LEG_TO_MPC_IDX[leg]
            if contact[mpc_idx]:
                continue  # stance: q̈ = 0
            yaml_sl = _MPC_IDX_TO_YAML_SLICE[mpc_idx]
            for k in range(3):
                i_yaml = yaml_sl.start + k
                i_pin  = _YAML_TO_PIN[i_yaml]
                q_ddot_pin[i_pin] = (
                    self._kp_sw * (q_sw_des[i_yaml] - q_yaml[i_yaml])
                    + self._kd_sw * (dq_sw_des[i_yaml] - dq_yaml[i_yaml])
                )
        return q_ddot_pin
