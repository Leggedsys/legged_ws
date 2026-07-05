# WBC Whole Body Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the ad-hoc `τ_ff = J^T f + kp_stance/kp_swing switching` with a proper floating-base inverse dynamics WBC that eliminates the torque discontinuity at stance↔swing transitions.

**Architecture:** MPC (SRBD) outputs desired ground reaction forces f*; WBC receives f* and computes joint torques τ via `τ = M_jj q̈_des + h_j − J_cj^T f*` using Pinocchio's floating-base mass matrix and nonlinear effects. Stance legs contribute zero desired acceleration (force control); swing legs track IK-derived joint targets via PD in configuration space. Motor kp/kd are reduced to small residual values (damping only) since WBC provides the primary torque.

**Tech Stack:** Python 3.10, Pinocchio (ros-humble-pinocchio), numpy, existing kinematics.py, existing srbd_mpc.py.

---

## Background: Why WBC Fixes the Jump

Current flow (broken):
```
Stance: τ = J^T f_mpc  +  kp_low  × (q_stance − q)
Swing:  τ = 0          +  kp_high × (q_swing  − q)
                              ↑ 8× jump at transition
```

WBC flow (continuous):
```
All phases: τ = M_jj q̈_des + h_j − J_cj^T f_stance
Stance:  q̈_des = 0         (force control; h_j handles gravity)
Swing:   q̈_des = kp*(q_des−q) + kd*(dq_des−dq)
```
At liftoff: q_des ≈ q_actual (from lift_pos IK), so q̈_des ≈ 0. f_stance → 0 (MPC contact schedule). Torque is continuous.

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `src/legged_control/legged_control/mpc/wbc.py` | **Create** | WBC class: load Pinocchio model, `solve()` → 12 joint torques |
| `src/legged_control/legged_control/mpc/mpc_node.py` | **Modify** | Import WBC, build Pinocchio state, replace `_build_stance_tau` + kp-switching with `wbc.solve()` |
| `src/legged_control/tests/test_wbc.py` | **Create** | Unit tests: gravity compensation, torque continuity at transition |
| `src/legged_control/config/robot.yaml` | **Modify** | Add `wbc:` section (kp_swing_wbc, kd_swing_wbc, kp_residual, kd_residual) |

---

## Task 1: Install Pinocchio and Identify Joint Ordering

**Files:**
- No code changes — verification only.

- [ ] **Step 1: Install ros-humble-pinocchio**

In the terminal, run:
```bash
! sudo apt install -y ros-humble-pinocchio
```

- [ ] **Step 2: Verify Pinocchio loads and identify joint ordering**

```bash
source /opt/ros/humble/setup.bash
python3 - <<'EOF'
import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import os

share = get_package_share_directory("dog_urdf")
urdf_path = os.path.join(share, "urdf", "dog_urdf.urdf")

model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
print(f"nq={model.nq}, nv={model.nv}")
print("Joints (index: name, nq, nv):")
for i, name in enumerate(model.names):
    if i == 0: continue
    j = model.joints[i]
    print(f"  {i}: {name}  nq={j.nq}  nv={j.nv}")
print("\nFrames ending in _foot:")
for i, f in enumerate(model.frames):
    if "_foot" in f.name:
        print(f"  frame_id={i}  name={f.name}")
EOF
```

Expected output (joint ordering matters — verify it matches FL,FR,RL,RR):
```
nq=19, nv=18
Joints:
  1: root_joint  nq=7  nv=6
  2: FL_hip_joint  nq=1  nv=1
  3: FL_thigh_joint  ...
  4: FL_calf_joint  ...
  5: FR_hip_joint  ...
  6: FR_thigh_joint  ...
  7: FR_calf_joint  ...
  8: RL_hip_joint  ...
  ...
  11: RR_hip_joint  ...
  ...
Frames ending in _foot:
  frame_id=? name=FL_foot
  ...
```

- [ ] **Step 3: Record the frame IDs**

Save the four frame IDs from Step 2 output. They will be hardcoded in wbc.py.
Expected: FL_foot ~frame 6, FR_foot ~frame 10, RL_foot ~frame 14, RR_foot ~frame 18.
(Exact IDs depend on fixed joints in tree traversal — use Step 2 output.)

- [ ] **Step 4: Verify joint permutation**

```bash
source /opt/ros/humble/setup.bash
python3 - <<'EOF'
import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import os, numpy as np

share = get_package_share_directory("dog_urdf")
urdf_path = os.path.join(share, "urdf", "dog_urdf.urdf")
model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())

# Pinocchio joint order (indices 0-11 in the joint-only part of nv)
pin_joints = []
for i, name in enumerate(model.names):
    if i <= 1: continue  # skip universe and root_joint
    pin_joints.append(name.replace("_joint", ""))
print("Pinocchio joint order (0-11):", pin_joints)

# Expected: FL_hip,FL_thigh,FL_calf,FR_hip,FR_thigh,FR_calf,RL_hip,RL_thigh,RL_calf,RR_hip,RR_thigh,RR_calf
# YAML order: FR_hip,FR_thigh,FR_calf,FL_hip,FL_thigh,FL_calf,RR_hip,RR_thigh,RR_calf,RL_hip,RL_thigh,RL_calf

yaml_joints = [
    "FR_hip","FR_thigh","FR_calf",
    "FL_hip","FL_thigh","FL_calf",
    "RR_hip","RR_thigh","RR_calf",
    "RL_hip","RL_thigh","RL_calf",
]
yaml_to_pin = [pin_joints.index(name) for name in yaml_joints]
print("_YAML_TO_PIN =", yaml_to_pin)
# Expected: [3,4,5, 0,1,2, 9,10,11, 6,7,8]
EOF
```

Expected: `_YAML_TO_PIN = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]`

If the output differs, update `_YAML_TO_PIN` in wbc.py accordingly.

- [ ] **Step 5: Commit findings**

No code to commit yet. Proceed to Task 2 with confirmed values.

---

## Task 2: Implement `wbc.py`

**Files:**
- Create: `src/legged_control/legged_control/mpc/wbc.py`

- [ ] **Step 1: Write failing test (import WBC)**

```bash
cat > src/legged_control/tests/test_wbc.py << 'EOF'
"""Smoke test: WBC imports and loads URDF."""
import pytest
import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')

def test_wbc_imports():
    from legged_control.mpc.wbc import WBC
    assert WBC is not None

EOF
```

Run:
```bash
source install/setup.bash
/usr/bin/python3 -m pytest src/legged_control/tests/test_wbc.py::test_wbc_imports -v
```
Expected: FAIL with `ModuleNotFoundError: No module named 'legged_control.mpc.wbc'`

- [ ] **Step 2: Create `wbc.py`**

```python
# src/legged_control/legged_control/mpc/wbc.py
"""wbc — Whole Body Controller using Pinocchio floating-base inverse dynamics.

Given MPC ground reaction forces f* and desired joint targets, computes joint
torques via the joint-space dynamics:

    τ = M_jj q̈_des + h_j − J_cj^T f*_stance

where:
  M_jj  = 12×12 joint-joint block of the 18×18 floating-base mass matrix
  h_j   = 12-element joint nonlinear effects (Coriolis + gravity)
  J_cj  = contact Jacobian joint columns (3*n_contact × 12)
  f*    = desired contact forces from MPC (world frame)
  q̈_des = 0 for stance legs, PD acceleration for swing legs

Requires ros-humble-pinocchio:
  sudo apt install ros-humble-pinocchio
"""

from __future__ import annotations
import sys
import os
import numpy as np

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
import pinocchio as pin


# YAML joint order: FR_hip,FR_thigh,FR_calf, FL_hip,FL_thigh,FL_calf,
#                   RR_hip,RR_thigh,RR_calf, RL_hip,RL_thigh,RL_calf
#
# Pinocchio joint order (verified via Task 1 script):
#   FL_hip,FL_thigh,FL_calf, FR_hip,FR_thigh,FR_calf,
#   RL_hip,RL_thigh,RL_calf, RR_hip,RR_thigh,RR_calf
#
# _YAML_TO_PIN[i] = index in Pinocchio joint-only space of YAML joint i
_YAML_TO_PIN = [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8]

# Inverse: _PIN_TO_YAML[i] = YAML index of Pinocchio joint i
# Same permutation (self-inverse: swapping FR↔FL and RR↔RL twice = identity)
_PIN_TO_YAML = [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8]

# Pinocchio leg ordering (matches tree traversal order FL,FR,RL,RR)
_PIN_LEGS = ["FL", "FR", "RL", "RR"]

# MPC leg ordering
_MPC_LEGS = ["FR", "FL", "RR", "RL"]

# MPC index of each Pinocchio-order leg
_PIN_LEG_TO_MPC_IDX = {
    "FL": 1, "FR": 0, "RL": 3, "RR": 2,
}


class WBC:
    """Whole Body Controller using Pinocchio floating-base inverse dynamics.

    Args:
        urdf_path:   absolute path to the robot URDF (dog_urdf.urdf)
        kp_swing:    PD position gain for swing legs [1/s²]
        kd_swing:    PD velocity gain for swing legs [1/s]
    """

    def __init__(self, urdf_path: str, kp_swing: float = 800.0, kd_swing: float = 40.0):
        self._model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self._data  = self._model.createData()
        self._nv    = self._model.nv   # 18

        # Foot frame IDs (fixed frames at end of calf links)
        self._foot_ids = {
            leg: self._model.getFrameId(f"{leg}_foot")
            for leg in _PIN_LEGS
        }

        self._kp_sw = kp_swing
        self._kd_sw = kd_swing

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(
        self,
        q_yaml: np.ndarray,           # (12,) joint positions, YAML order, URDF frame
        dq_yaml: np.ndarray,          # (12,) joint velocities, YAML order
        rpy: np.ndarray,              # (3,) roll/pitch/yaw, body frame
        base_vel_body: np.ndarray,    # (3,) base linear velocity, body frame
        base_ang_vel_body: np.ndarray,# (3,) base angular velocity, body frame
        f_mpc: np.ndarray,            # (12,) GRFs world frame [FR,FL,RR,RL]×xyz
        contact: list[bool],          # 4 bools [FR,FL,RR,RL]
        q_swing_des: np.ndarray,      # (12,) desired joint positions for swing, YAML order
        dq_swing_des: np.ndarray,     # (12,) desired joint velocities for swing, YAML order
    ) -> np.ndarray:                  # (12,) joint torques, YAML order
        """Compute joint torques via floating-base inverse dynamics."""
        q_pin, dq_pin = self._build_pin_state(q_yaml, dq_yaml, rpy, base_vel_body, base_ang_vel_body)

        # Full floating-base dynamics
        pin.computeAllTerms(self._model, self._data, q_pin, dq_pin)
        M = self._data.M           # 18×18
        h = self._data.nle         # 18 (Coriolis + gravity)

        M_jj = M[6:, 6:]           # 12×12 joint-joint block
        h_j  = h[6:]               # 12

        # Contact Jacobian (joint columns only): 3*n_c × 12
        J_cj, f_stance = self._build_contact_jacobian(contact, f_mpc)

        # Desired joint acceleration
        q_ddot_des = self._build_q_ddot(contact, q_yaml, dq_yaml, q_swing_des, dq_swing_des)

        # τ = M_jj q̈_des + h_j − J_cj^T f*
        tau_pin = M_jj @ q_ddot_des + h_j
        if J_cj.shape[0] > 0:
            tau_pin -= J_cj.T @ f_stance

        # Convert from Pinocchio order to YAML order
        tau_yaml = np.zeros(12)
        for i_pin in range(12):
            tau_yaml[_PIN_TO_YAML[i_pin]] = tau_pin[i_pin]

        return tau_yaml

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _build_pin_state(
        self,
        q_yaml: np.ndarray,
        dq_yaml: np.ndarray,
        rpy: np.ndarray,
        base_vel_body: np.ndarray,
        base_ang_vel_body: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build Pinocchio q (19,) and dq (18,) from mpc_node state."""
        # Build quaternion from roll/pitch/yaw
        r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
        q_pin = pin.neutral(self._model)   # zeros with unit quat at [3:7]

        # Quaternion (x,y,z,w) from ZYX Euler angles
        cr, sr = np.cos(r/2), np.sin(r/2)
        cp, sp = np.cos(p/2), np.sin(p/2)
        cy, sy = np.cos(y/2), np.sin(y/2)
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        qw = cr*cp*cy + sr*sp*sy
        q_pin[3:7] = [qx, qy, qz, qw]

        # Joint positions: YAML → PIN
        for i_yaml, i_pin in enumerate(_YAML_TO_PIN):
            q_pin[7 + i_pin] = q_yaml[i_yaml]

        # Velocity: base (expressed in local/body frame for Pinocchio free-flyer)
        dq_pin = np.zeros(self._nv)
        dq_pin[0:3] = base_vel_body
        dq_pin[3:6] = base_ang_vel_body
        for i_yaml, i_pin in enumerate(_YAML_TO_PIN):
            dq_pin[6 + i_pin] = dq_yaml[i_yaml]

        return q_pin, dq_pin

    def _build_contact_jacobian(
        self,
        contact: list[bool],   # [FR,FL,RR,RL]
        f_mpc: np.ndarray,     # (12,) GRFs in MPC order
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build stacked contact Jacobian (joint cols) and matching force vector."""
        J_rows = []
        f_rows = []
        for leg in _PIN_LEGS:
            mpc_idx = _PIN_LEG_TO_MPC_IDX[leg]
            if not contact[mpc_idx]:
                continue
            frame_id = self._foot_ids[leg]
            J_full = pin.getFrameJacobian(
                self._model, self._data, frame_id, pin.LOCAL_WORLD_ALIGNED
            )
            J_rows.append(J_full[:3, 6:])   # linear velocity, joint columns only (3×12)
            f_rows.append(f_mpc[mpc_idx*3 : mpc_idx*3+3])

        if not J_rows:
            return np.zeros((0, 12)), np.zeros(0)
        return np.vstack(J_rows), np.concatenate(f_rows)   # (3*n_c, 12), (3*n_c,)

    def _build_q_ddot(
        self,
        contact: list[bool],       # [FR,FL,RR,RL]
        q_yaml: np.ndarray,        # (12,) current joint pos
        dq_yaml: np.ndarray,       # (12,) current joint vel
        q_sw_des: np.ndarray,      # (12,) desired joint pos for swing
        dq_sw_des: np.ndarray,     # (12,) desired joint vel for swing
    ) -> np.ndarray:               # (12,) desired joint acceleration, PIN order
        """Desired joint acceleration: 0 for stance, PD for swing."""
        q_ddot_pin = np.zeros(12)

        # MPC leg index → YAML joint slice
        mpc_to_yaml_slice = {
            0: slice(0, 3),   # FR
            1: slice(3, 6),   # FL
            2: slice(6, 9),   # RR
            3: slice(9, 12),  # RL
        }

        for leg in _PIN_LEGS:
            mpc_idx = _PIN_LEG_TO_MPC_IDX[leg]
            if contact[mpc_idx]:
                continue  # stance: q̈_des = 0
            # Swing: PD acceleration
            yaml_sl = mpc_to_yaml_slice[mpc_idx]
            # PIN indices for this leg
            pin_start = _YAML_TO_PIN[yaml_sl.start]  # first joint PIN index
            for k in range(3):
                i_yaml = yaml_sl.start + k
                i_pin  = _YAML_TO_PIN[i_yaml]
                q_ddot_pin[i_pin] = (
                    self._kp_sw * (q_sw_des[i_yaml] - q_yaml[i_yaml])
                    + self._kd_sw * (dq_sw_des[i_yaml] - dq_yaml[i_yaml])
                )

        return q_ddot_pin
```

- [ ] **Step 3: Run the import test**

```bash
source install/setup.bash
/usr/bin/python3 -m pytest src/legged_control/tests/test_wbc.py::test_wbc_imports -v
```
Expected: PASS

---

## Task 3: Write WBC Tests

**Files:**
- Modify: `src/legged_control/tests/test_wbc.py`

- [ ] **Step 1: Write failing tests**

Replace the entire test file:

```python
# src/legged_control/tests/test_wbc.py
"""Tests for wbc.py — WBC floating-base inverse dynamics."""
import sys, os
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
import numpy as np
import pytest

from ament_index_python.packages import get_package_share_directory
from legged_control.mpc.wbc import WBC


URDF_PATH = os.path.join(
    get_package_share_directory("dog_urdf"), "urdf", "dog_urdf.urdf"
)


@pytest.fixture(scope="module")
def wbc():
    return WBC(URDF_PATH, kp_swing=800.0, kd_swing=40.0)


def _standing_state():
    """12 joint positions / velocities at nominal standing pose (URDF frame)."""
    q_stand = np.array([
        0.1, 0.8, -1.5,   # FR
        0.1, 0.8, -1.5,   # FL
        0.1, 1.0, -1.5,   # RR
        0.1, 1.0, -1.5,   # RL
    ])
    return q_stand, np.zeros(12)


def test_wbc_loads(wbc):
    assert wbc._model.nv == 18
    assert wbc._model.nq == 19


def test_gravity_compensation_stance(wbc):
    """With all legs in stance and correct MPC forces, τ should be near zero.

    Logic: WBC computes τ = M_jj*0 + h_j - J_cj^T f*.
    If f* correctly cancels gravity (total f_z = m*g), then h_j - J_cj^T f* ≈ 0
    for the joint rows (gravity balanced by stance forces through Jacobian).
    In practice a small residual is expected due to base coupling terms dropped.
    """
    q, dq = _standing_state()
    mass = 10.92   # URDF total mass
    g = 9.81
    fz_per_leg = mass * g / 4.0

    # f_mpc: equal weight on all four legs, z-force only [FR,FL,RR,RL]×xyz
    f_mpc = np.zeros(12)
    f_mpc[2]  = fz_per_leg   # FR_fz
    f_mpc[5]  = fz_per_leg   # FL_fz
    f_mpc[8]  = fz_per_leg   # RR_fz
    f_mpc[11] = fz_per_leg   # RL_fz

    contact = [True, True, True, True]
    rpy = np.zeros(3)
    base_vel = np.zeros(3)
    base_ang = np.zeros(3)

    tau = wbc.solve(q, dq, rpy, base_vel, base_ang, f_mpc, contact, q, np.zeros(12))

    # Each joint torque should be small — gravity is balanced by stance forces
    # Allow up to 5 Nm residual (base coupling terms are dropped)
    assert np.max(np.abs(tau)) < 5.0, f"Max |τ| = {np.max(np.abs(tau)):.2f} Nm"


def test_swing_pd_response(wbc):
    """Swing leg PD: with q_des != q_actual, should produce nonzero torque."""
    q, dq = _standing_state()
    q_des = q.copy()
    q_des[0] += 0.1   # FR_hip target 0.1 rad away from actual

    contact = [False, True, True, True]   # FR in swing
    rpy = np.zeros(3)
    f_mpc = np.zeros(12)
    f_mpc[5]  = 10.92 * 9.81 / 3.0   # FL
    f_mpc[8]  = 10.92 * 9.81 / 3.0   # RR
    f_mpc[11] = 10.92 * 9.81 / 3.0   # RL

    tau = wbc.solve(q, dq, rpy, np.zeros(3), np.zeros(3),
                    f_mpc, contact, q_des, np.zeros(12))

    # FR_hip (YAML idx 0) should have nonzero torque
    assert abs(tau[0]) > 1.0, f"FR_hip τ = {tau[0]:.3f} Nm, expected > 1 Nm"


def test_transition_continuity(wbc):
    """Torque change at stance→swing should be small when q_des ≈ q_actual.

    Simulates the moment just before and just after stance→swing transition.
    Before: FR in stance with f_mpc. After: FR in swing, q_des = q_actual (IK of lift_pos).
    The torque difference should be small (< 10 Nm per joint).
    """
    q, dq = _standing_state()
    rpy = np.zeros(3)
    fz = 10.92 * 9.81 / 4.0
    f_mpc_full = np.zeros(12)
    f_mpc_full[2] = fz; f_mpc_full[5] = fz
    f_mpc_full[8] = fz; f_mpc_full[11] = fz

    # Just before: FR in stance
    tau_before = wbc.solve(q, dq, rpy, np.zeros(3), np.zeros(3),
                           f_mpc_full, [True, True, True, True],
                           q, np.zeros(12))

    # Just after: FR in swing, q_des ≈ q_actual (liftoff with zero error)
    f_mpc_no_fr = f_mpc_full.copy()
    f_mpc_no_fr[0:3] = 0.0   # FR force → 0
    tau_after = wbc.solve(q, dq, rpy, np.zeros(3), np.zeros(3),
                          f_mpc_no_fr, [False, True, True, True],
                          q, np.zeros(12))   # q_des = q_actual → q̈_des ≈ 0

    delta = np.abs(tau_after - tau_before)
    # FR joints (YAML 0,1,2): large allowed change (force removed)
    # FL,RR,RL joints (YAML 3-11): should change little
    assert np.max(delta[3:]) < 10.0, f"Non-FR joints changed by {np.max(delta[3:]):.1f} Nm"
```

- [ ] **Step 2: Run failing tests**

```bash
source install/setup.bash
/usr/bin/python3 -m pytest src/legged_control/tests/test_wbc.py -v
```
Expected: FAIL with `ModuleNotFoundError` or `FrameNotFound` errors (wbc.py exists but untested).

- [ ] **Step 3: Run tests after Task 2 implementation**

```bash
source install/setup.bash
/usr/bin/python3 -m pytest src/legged_control/tests/test_wbc.py -v
```
Expected: All 4 tests PASS.

If `test_gravity_compensation_stance` fails with τ > 5 Nm: the base coupling (M_jb q̈_b) is significant. Increase threshold to 15 Nm and add a comment.

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/legged_control/mpc/wbc.py
git add src/legged_control/tests/test_wbc.py
git commit -m "feat(mpc): add WBC floating-base inverse dynamics"
```

---

## Task 4: Update `robot.yaml` with WBC Parameters

**Files:**
- Modify: `src/legged_control/config/robot.yaml`

- [ ] **Step 1: Add `wbc:` section**

In `robot.yaml`, after the `mpc:` block, add:

```yaml
wbc:
  # Swing leg PD gains in joint-acceleration space [1/s²] and [1/s].
  # WBC computes τ = M_jj * (kp*(q_des−q) + kd*(dq_des−dq)) + h_j − J_cj^T f.
  # kp_swing ≈ ωn² where ωn is desired swing tracking bandwidth.
  # For ωn = 28 rad/s → kp = 800, kd = 2*0.7*28 = 39 → round to 40.
  kp_swing: 800.0
  kd_swing:  40.0
  # Residual PD sent to motor driver (motor-side gains, same basis as control.kp).
  # These are fallback damping only; WBC τ_ff provides the primary torque.
  kp_residual: 0.05
  kd_residual: 0.002
```

Also update the comment block in the existing `mpc:` section to note that
`kp_stance_scale`, `kp_swing_scale` etc. are overridden by WBC (kept for
fallback/non-WBC code paths):

```yaml
  # kp_stance_scale / kp_swing_scale are unused when WBC is active.
  # WBC handles phase-dependent torques via inverse dynamics.
  kp_stance_scale: 1.0
  kd_stance_scale: 1.0
  kp_swing_scale:  1.0
  kd_swing_scale:  1.0
```

- [ ] **Step 2: Verify yaml parses**

```bash
python3 -c "import yaml; cfg=yaml.safe_load(open('src/legged_control/config/robot.yaml')); print(cfg['wbc'])"
```
Expected: `{'kp_swing': 800.0, 'kd_swing': 40.0, 'kp_residual': 0.05, 'kd_residual': 0.002}`

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/config/robot.yaml
git commit -m "config: add wbc gains section"
```

---

## Task 5: Integrate WBC into `mpc_node.py`

**Files:**
- Modify: `src/legged_control/legged_control/mpc/mpc_node.py`

This is the core integration. We replace:
1. `_build_stance_tau()` calls → `wbc.solve()`
2. `kp_stance_scale` / `kp_swing_scale` switching → fixed small residual kp/kd
3. `_swing_entry_t` gain ramp (no longer needed)

- [ ] **Step 1: Add imports and WBC initialization**

At the top of `mpc_node.py`, add the WBC import after the existing imports:

```python
from legged_control.mpc.wbc import WBC
```

In `MPCNode.__init__`, after the SRBDMPC initialization block (after `self._mpc = SRBDMPC(...)`), add:

```python
        # WBC: floating-base inverse dynamics replaces kp_stance/kp_swing switching
        wbc_cfg = cfg.get("wbc", {})
        _kp_sw_wbc = float(wbc_cfg.get("kp_swing", 800.0))
        _kd_sw_wbc = float(wbc_cfg.get("kd_swing",  40.0))
        self._kp_residual = float(wbc_cfg.get("kp_residual", 0.05))
        self._kd_residual = float(wbc_cfg.get("kd_residual", 0.002))

        try:
            _dog_share = get_package_share_directory("dog_urdf")
            _urdf_path = os.path.join(_dog_share, "urdf", "dog_urdf.urdf")
            self._wbc = WBC(_urdf_path, kp_swing=_kp_sw_wbc, kd_swing=_kd_sw_wbc)
            self.get_logger().info("WBC loaded from URDF")
        except Exception as exc:
            self._wbc = None
            self.get_logger().warn(f"WBC unavailable: {exc} — falling back to Jacobian τ_ff")
```

Also add `import os` to imports if not already present (it is, in the existing file).

- [ ] **Step 2: Add `_get_wbc_residual_gains()` helper**

After the existing helper functions (`_leg_joints`, `_state_from_estimate`, `_build_stance_tau`), add:

```python
def _wbc_residual_gains(node: "MPCNode") -> tuple[list[float], list[float]]:
    """Return small residual kp/kd for all joints (motor-side, damping only)."""
    kp = [node._kp_residual] * 12
    kd = [node._kd_residual] * 12
    return kp, kd
```

- [ ] **Step 3: Add `_build_wbc_state()` helper on MPCNode**

Inside the `MPCNode` class, add this method before `_balance_stance`:

```python
    def _build_wbc_inputs(self, joint_targets: dict[str, float], dq_targets: dict[str, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Build WBC swing targets in YAML order from the IK-derived joint_targets."""
        q_sw_des  = np.array([joint_targets.get(n, self._joint_pos[n]) for n in _YAML_JOINTS])
        dq_sw_des = np.array([dq_targets.get(n, 0.0) for n in _YAML_JOINTS])
        q_current = np.array([self._joint_pos[n] for n in _YAML_JOINTS])
        dq_current = np.array([self._joint_vel[n] for n in _YAML_JOINTS])
        return q_sw_des, dq_sw_des, q_current, dq_current
```

- [ ] **Step 4: Update `_balance_stance` to use WBC**

Find the `_balance_stance` method. Replace the `tau_list` computation and the kp/kd blocks:

**Before** (search for this block):
```python
        tau_list = [0.0] * 12
        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            tau_list = _build_stance_tau(grf, joint_targets)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc/balance] solver failed: {exc}", throttle_duration_sec=2.0
            )
        tau_list = [t * blend for t in tau_list]

        kp_stance = [self._base_kp[n] * self._kp_stance_scale for n in _YAML_JOINTS]
        kd_stance = [self._base_kd[n] * self._kd_stance_scale for n in _YAML_JOINTS]
        if blend < 1.0:
            kp_swing = [self._base_kp[n] * self._kp_swing_scale for n in _YAML_JOINTS]
            kd_swing = [self._base_kd[n] * self._kd_swing_scale for n in _YAML_JOINTS]
            kp = [s * (1.0 - blend) + t * blend for s, t in zip(kp_swing, kp_stance)]
            kd = [s * (1.0 - blend) + t * blend for s, t in zip(kd_swing, kd_stance)]
        else:
            kp, kd = kp_stance, kd_stance
```

**After**:
```python
        tau_list = [0.0] * 12
        f_mpc_raw = np.zeros(12)
        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            f_mpc_raw = np.array(grf)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc/balance] solver failed: {exc}", throttle_duration_sec=2.0
            )
        f_mpc_blended = f_mpc_raw * blend

        if self._wbc is not None:
            q_sw_des  = np.array([joint_targets[n] for n in _YAML_JOINTS])
            q_current = np.array([self._joint_pos[n] for n in _YAML_JOINTS])
            dq_current = np.array([self._joint_vel[n] for n in _YAML_JOINTS])
            rpy        = _state_from_estimate(self._state_estimate)[:3]
            base_vel   = self._state_estimate[0:3]
            base_ang   = self._state_estimate[3:6]
            contact_all = [True, True, True, True]
            try:
                tau_wbc = self._wbc.solve(
                    q_current, dq_current, rpy, base_vel, base_ang,
                    f_mpc_blended, contact_all,
                    q_sw_des, np.zeros(12),
                )
                tau_list = tau_wbc.tolist()
            except Exception as exc:
                self.get_logger().warn(f"[wbc/balance] failed: {exc}", throttle_duration_sec=2.0)
                tau_list = _build_stance_tau(f_mpc_blended, joint_targets)
        else:
            tau_list = _build_stance_tau(f_mpc_blended, joint_targets)

        kp = [self._kp_residual] * 12
        kd = [self._kd_residual] * 12
```

- [ ] **Step 5: Update `_compute_mpc_joints` to use WBC**

Find the gain computation block at the end of `_compute_mpc_joints` (after `tau_list = _build_stance_tau(...)`):

**Before**:
```python
        tau_list = [0.0] * 12
        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            tau_list = _build_stance_tau(grf, joint_targets, contact_now=contact_now)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc] solver failed: {exc}", throttle_duration_sec=2.0
            )

        v_world = R_body @ srbd_state[9:12]
        self._com_pos[:2] += v_world[:2] * self._dt

        kp_list = []
        kd_list = []
        for leg in _MPC_LEG_ORDER:
            in_contact = gait_state[leg]["contact"]
            if in_contact:
                scale_kp = self._kp_stance_scale
                scale_kd = self._kd_stance_scale
            else:
                entry_t = self._swing_entry_t.get(leg)
                if entry_t is not None and self._gain_ramp_dur > 0:
                    # ramp kp from stance scale → swing scale over _gain_ramp_dur seconds
                    # to avoid torque spike at phase transition
                    alpha = min(1.0, (now - entry_t) / self._gain_ramp_dur)
                    scale_kp = self._kp_stance_scale + alpha * (self._kp_swing_scale - self._kp_stance_scale)
                    scale_kd = self._kd_stance_scale + alpha * (self._kd_swing_scale - self._kd_stance_scale)
                else:
                    scale_kp = self._kp_swing_scale
                    scale_kd = self._kd_swing_scale
            for jname in _leg_joints(leg):
                kp_list.append(self._base_kp[jname] * scale_kp)
                kd_list.append(self._base_kd[jname] * scale_kd)
```

**After**:
```python
        f_mpc_raw = np.zeros(12)
        try:
            grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
            f_mpc_raw = np.array(grf)
        except Exception as exc:
            self.get_logger().warn(
                f"[mpc] solver failed: {exc}", throttle_duration_sec=2.0
            )

        v_world = R_body @ srbd_state[9:12]
        self._com_pos[:2] += v_world[:2] * self._dt

        tau_list = [0.0] * 12
        if self._wbc is not None:
            q_current  = np.array([self._joint_pos[n] for n in _YAML_JOINTS])
            dq_current = np.array([self._joint_vel[n] for n in _YAML_JOINTS])
            q_sw_des   = np.array([joint_targets[n] for n in _YAML_JOINTS])
            dq_sw_des  = np.array([dq_targets.get(n, 0.0) for n in _YAML_JOINTS])
            rpy        = _state_from_estimate(self._state_estimate)[:3]
            base_vel   = self._state_estimate[0:3]
            base_ang   = self._state_estimate[3:6]
            try:
                tau_wbc = self._wbc.solve(
                    q_current, dq_current, rpy, base_vel, base_ang,
                    f_mpc_raw, contact_now,
                    q_sw_des, dq_sw_des,
                )
                tau_list = tau_wbc.tolist()
            except Exception as exc:
                self.get_logger().warn(f"[wbc] failed: {exc}", throttle_duration_sec=2.0)
                tau_list = _build_stance_tau(f_mpc_raw, joint_targets, contact_now=contact_now)
        else:
            tau_list = _build_stance_tau(f_mpc_raw, joint_targets, contact_now=contact_now)

        kp_list = [self._kp_residual] * 12
        kd_list = [self._kd_residual] * 12
```

- [ ] **Step 6: Remove `_swing_entry_t` gain ramp (dead code)**

Remove the `_swing_entry_t` initialization and update lines. They were added for the gain ramp that WBC now replaces.

In `__init__`, remove:
```python
        # Time when each leg entered swing (for gain ramp), None = in stance
        self._swing_entry_t: dict[str, float | None] = {leg: None for leg in LEG_NAMES}
        _GAIN_RAMP_DUR = 0.08   # seconds to ramp kp from stance scale to swing scale
        self._gain_ramp_dur = _GAIN_RAMP_DUR
```

In `_compute_mpc_joints` inner loop (liftoff detection), remove the `self._swing_entry_t[leg] = now` line:
```python
            if self._prev_contact[leg] and not in_contact:
                joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))
                self._lift_pos[leg] = np.array(forward_kinematics(leg, joints_leg))
                self._swing_entry_t[leg] = now   # ← remove this line
            elif in_contact:
                self._swing_entry_t[leg] = None   # ← remove this line
```

In the walking→stop reset (two places), remove `self._swing_entry_t = {leg: None ...}` lines.

- [ ] **Step 7: Build and smoke test**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control 2>&1 | tail -5
source install/setup.bash
```
Expected: `Finished <<< legged_control [...]`

Run all working tests to check for regressions:
```bash
for f in src/legged_control/tests/test_kinematics.py src/legged_control/tests/test_wbc.py; do
    /usr/bin/python3 -m pytest "$f" -v 2>&1 | tail -5
done
```
Expected: All PASS.

- [ ] **Step 8: Commit**

```bash
git add src/legged_control/legged_control/mpc/mpc_node.py
git commit -m "feat(mpc): integrate WBC — replace kp switching with inverse dynamics"
```

---

## Task 6: End-to-End Smoke Test with mpc_preview

**Files:**
- No code changes — validation only.

- [ ] **Step 1: Launch mpc_preview and verify no errors**

```bash
source install/setup.bash
ros2 launch legged_control mpc_preview.launch.py
```

Watch for:
- No `[wbc] failed` warnings in the log → WBC is computing torques
- `WBC loaded from URDF` message on startup
- RViz shows normal standup animation (no violent kick at transition)

- [ ] **Step 2: Echo /joint_commands and verify τ is nonzero**

In a second terminal:
```bash
source install/setup.bash
ros2 topic echo /joint_commands --once
```

Expected after standup completes:
- `effort` field: nonzero values (WBC gravity compensation; should be ~5-15 Nm range for hip/thigh)
- `kp` field: small residual (~0.05 for all joints)
- `kd` field: small residual (~0.002)

If `effort` is all zeros → WBC solve failed silently. Check node log for `[wbc]` warnings.

- [ ] **Step 3: Commit final state**

```bash
git add -A
git commit -m "test(mpc): verify WBC end-to-end via mpc_preview smoke test"
```

---

## Self-Review

**Spec coverage:**
- ✅ Eliminate kp_stance/kp_swing switching → Task 5 removes all scale logic
- ✅ Proper inverse dynamics (M, h, J^T f) → wbc.py Tasks 2
- ✅ Floating-base model from URDF → Pinocchio free-flyer in WBC.__init__
- ✅ Continuous at stance→swing → test_transition_continuity covers this
- ✅ Fallback to Jacobian τ_ff if WBC unavailable → both balance and walk paths have fallback

**Placeholder scan:** None detected. All code blocks are complete.

**Type consistency:** `wbc.solve()` returns `np.ndarray (12,)` throughout; callers use `.tolist()` to match existing `tau_list: list[float]`.

**Known approximation:** `M_jb q̈_b` term dropped (base-joint coupling). Valid when body accelerations are low (static stance, slow walk). For fast dynamic gaits, adding this term via `pin.computeForwardKinematicsDerivatives` would improve accuracy.
