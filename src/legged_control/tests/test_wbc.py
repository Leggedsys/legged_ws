"""Tests for wbc.py."""
import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
import numpy as np
import pytest
import os
from legged_control.mpc.wbc import WBC

# Resolve URDF path without relying on ament_index (conftest stubs it to /nonexistent).
# Walk up from this file to the workspace root, then look in install and src.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, "..", "..", ".."))
_URDF_CANDIDATES = [
    os.path.join(_WS_ROOT, "install", "dog_urdf", "share", "dog_urdf", "urdf", "dog_urdf.urdf"),
    os.path.join(_WS_ROOT, "src", "dog_urdf", "urdf", "dog_urdf.urdf"),
]
URDF_PATH = next((p for p in _URDF_CANDIDATES if os.path.isfile(p)), None)

if URDF_PATH is None:
    pytest.skip("dog_urdf URDF not found; build the workspace first", allow_module_level=True)

@pytest.fixture(scope="module")
def wbc():
    return WBC(URDF_PATH, kp_swing=800.0, kd_swing=40.0)

def _standing_state():
    q = np.array([0.1, 0.8, -1.5, 0.1, 0.8, -1.5, 0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
    return q, np.zeros(12)

def test_wbc_loads(wbc):
    assert wbc._model.nv == 18
    assert wbc._model.nq == 19

def test_gravity_compensation_stance(wbc):
    q, dq = _standing_state()
    mass = 10.92
    fz = mass * 9.81 / 4.0
    f_mpc = np.zeros(12)
    f_mpc[2] = fz; f_mpc[5] = fz; f_mpc[8] = fz; f_mpc[11] = fz
    tau = wbc.solve(q, dq, np.zeros(3), np.zeros(3), np.zeros(3),
                    f_mpc, [True, True, True, True], q, np.zeros(12))
    assert np.max(np.abs(tau)) < 15.0, f"Max |τ| = {np.max(np.abs(tau)):.2f} Nm"

def test_swing_pd_response(wbc):
    q, dq = _standing_state()
    q_des = q.copy()
    q_des[0] += 0.1   # FR_hip off by 0.1 rad
    f_mpc = np.zeros(12)
    f_mpc[5] = 10.92 * 9.81 / 3.0
    f_mpc[8] = 10.92 * 9.81 / 3.0
    f_mpc[11] = 10.92 * 9.81 / 3.0
    tau = wbc.solve(q, dq, np.zeros(3), np.zeros(3), np.zeros(3),
                    f_mpc, [False, True, True, True], q_des, np.zeros(12))
    assert abs(tau[0]) > 1.0, f"FR_hip τ = {tau[0]:.3f} Nm"

def test_transition_continuity(wbc):
    q, dq = _standing_state()
    fz = 10.92 * 9.81 / 4.0
    f_full = np.array([0,0,fz, 0,0,fz, 0,0,fz, 0,0,fz], dtype=float)
    tau_before = wbc.solve(q, dq, np.zeros(3), np.zeros(3), np.zeros(3),
                           f_full, [True, True, True, True], q, np.zeros(12))
    f_no_fr = f_full.copy(); f_no_fr[0:3] = 0.0
    tau_after = wbc.solve(q, dq, np.zeros(3), np.zeros(3), np.zeros(3),
                          f_no_fr, [False, True, True, True], q, np.zeros(12))
    delta = np.abs(tau_after - tau_before)
    assert np.max(delta[3:]) < 10.0, f"Non-FR joints changed {np.max(delta[3:]):.1f} Nm"
