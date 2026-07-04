"""Tests for MPC modules — runs offline, no ROS, no hardware required."""

import time
import numpy as np
import pytest

from legged_control.mpc.gait_scheduler import GaitScheduler, LEG_NAMES
from legged_control.mpc.swing_trajectory import (
    swing_foot_position,
    nominal_foot_position,
    landing_target,
)
from legged_control.mpc.srbd_mpc import SRBDMPC


# ── GaitScheduler ─────────────────────────────────────────────────────────────

def test_gait_all_legs_present():
    g = GaitScheduler(period=0.6, swing_ratio=0.4)
    state = g.query(t=0.0)
    assert set(state.keys()) == set(LEG_NAMES)


def test_gait_trot_diagonal_pairs():
    """Diagonal pairs (FL+RR, FR+RL) must always be in sync; both pairs must swing."""
    g = GaitScheduler(period=0.6, swing_ratio=0.4)
    t0 = g._t0
    fl_swung = fr_swung = False
    # sample 100 points across one full period — avoids floating-point boundary cases
    for dt in np.linspace(0.0, 0.6, 100, endpoint=False):
        s = g.query(t=t0 + dt)
        assert s["FL"]["contact"] == s["RR"]["contact"], f"FL/RR mismatch at dt={dt:.4f}"
        assert s["FR"]["contact"] == s["RL"]["contact"], f"FR/RL mismatch at dt={dt:.4f}"
        if not s["FL"]["contact"]:
            fl_swung = True
        if not s["FR"]["contact"]:
            fr_swung = True
    assert fl_swung, "FL/RR pair never entered swing phase"
    assert fr_swung, "FR/RL pair never entered swing phase"


def test_gait_phase_in_range():
    g = GaitScheduler(period=0.6, swing_ratio=0.4)
    t0 = g._t0
    for dt in np.linspace(0, 1.2, 50):
        for leg in LEG_NAMES:
            p = g.query(t=t0 + dt)[leg]["phase"]
            assert 0.0 <= p < 1.0, f"phase out of range: {p}"


def test_gait_swing_phase_zero_during_stance():
    g = GaitScheduler(period=0.6, swing_ratio=0.4)
    t0 = g._t0
    for dt in np.linspace(0, 0.6, 30):
        t = t0 + dt
        s = g.query(t=t)
        sp = g.swing_phase("FL", t=t)
        if s["FL"]["contact"]:
            assert sp == 0.0
        else:
            assert 0.0 <= sp <= 1.0


def test_gait_contact_mask_length():
    g = GaitScheduler()
    mask = g.contact_mask()
    assert len(mask) == 4
    assert all(isinstance(v, bool) for v in mask)


# ── SwingTrajectory ────────────────────────────────────────────────────────────

def test_swing_endpoints():
    p_lift = np.array([0.0, 0.0, -0.27])
    p_land = np.array([0.05, 0.0, -0.27])
    p0 = swing_foot_position(0.0, p_lift, p_land)
    p1 = swing_foot_position(1.0, p_lift, p_land)
    np.testing.assert_allclose(p0[:2], p_lift[:2], atol=1e-6)
    np.testing.assert_allclose(p1[:2], p_land[:2], atol=1e-6)


def test_swing_peak_clearance():
    p_lift = np.array([0.0, 0.0, -0.27])
    p_land = np.array([0.0, 0.0, -0.27])
    step_h = 0.06
    p_mid = swing_foot_position(0.5, p_lift, p_land, step_height=step_h)
    assert p_mid[2] > -0.27, "foot should be above ground at mid-swing"
    assert abs(p_mid[2] - (-0.27 + step_h)) < 1e-3


def test_swing_clamps_s():
    p_lift = np.array([0.0, 0.0, -0.27])
    p_land = np.array([0.1, 0.0, -0.27])
    # s outside [0,1] should clamp
    p_neg = swing_foot_position(-0.5, p_lift, p_land)
    p_over = swing_foot_position(1.5, p_lift, p_land)
    np.testing.assert_allclose(p_neg[:2], p_lift[:2], atol=1e-6)
    np.testing.assert_allclose(p_over[:2], p_land[:2], atol=1e-6)


def test_landing_target_zero_vel():
    p = landing_target("FR", np.array([0.0, 0.0]), gait_period=0.6, swing_ratio=0.4)
    assert p[0] == 0.0 and p[1] == 0.0, "zero velocity → foot directly below hip"


def test_landing_target_clamped():
    p = landing_target("FR", np.array([10.0, 10.0]), gait_period=0.6, swing_ratio=0.4)
    assert abs(p[0]) <= 0.12 and abs(p[1]) <= 0.06, "landing target should be clamped"


# ── SRBD MPC ──────────────────────────────────────────────────────────────────

@pytest.fixture
def mpc():
    inertia = np.diag([0.0196, 0.0228, 0.0169])
    return SRBDMPC(mass=14.55, inertia_body=inertia, dt=0.02, horizon=6)


def test_mpc_output_shape(mpc):
    state = np.zeros(12)
    state[5] = 0.27  # standing height
    state_ref = state.copy()
    foot_pos = np.array([
        [0.18, -0.13, -0.27],
        [0.18,  0.13, -0.27],
        [-0.18, -0.13, -0.27],
        [-0.18,  0.13, -0.27],
    ])
    contact = [[True, False, False, True]] * 6  # trot: FR+RL in contact
    grf = mpc.solve(state, state_ref, foot_pos, contact)
    assert grf.shape == (12,), f"Expected (12,), got {grf.shape}"


def test_mpc_swing_forces_near_zero(mpc):
    """Swing legs should have near-zero GRF (equality constraint)."""
    state = np.zeros(12)
    state[5] = 0.27
    state_ref = state.copy()
    foot_pos = np.zeros((4, 3))
    foot_pos[:, 2] = -0.27
    contact = [[True, False, False, True]] * 6  # FL, RR are swing
    grf = mpc.solve(state, state_ref, foot_pos, contact)
    # FL=index 1, RR=index 2 should be ~0
    fl_force = grf[3:6]
    rr_force = grf[6:9]
    np.testing.assert_allclose(fl_force, 0.0, atol=0.5)
    np.testing.assert_allclose(rr_force, 0.0, atol=0.5)


def test_mpc_stance_fz_positive(mpc):
    """Stance legs should push up (fz > 0)."""
    state = np.zeros(12)
    state[5] = 0.27
    state_ref = state.copy()
    foot_pos = np.zeros((4, 3))
    foot_pos[:, 2] = -0.27
    contact = [[True, False, False, True]] * 10
    grf = mpc.solve(state, state_ref, foot_pos, contact)
    assert grf[2] > 0, f"FR fz should be positive, got {grf[2]:.2f}"
    assert grf[11] > 0, f"RL fz should be positive, got {grf[11]:.2f}"


def test_mpc_weight_support(mpc):
    """Total fz from stance legs should roughly equal body weight."""
    state = np.zeros(12)
    state[5] = 0.27
    state_ref = state.copy()
    foot_pos = np.zeros((4, 3))
    foot_pos[:, 2] = -0.27
    # All four legs in contact
    contact = [[True, True, True, True]] * 6
    grf = mpc.solve(state, state_ref, foot_pos, contact)
    total_fz = grf[2] + grf[5] + grf[8] + grf[11]
    weight = 14.55 * 9.81
    assert abs(total_fz - weight) < weight * 0.3, (
        f"Total fz={total_fz:.1f} N, weight={weight:.1f} N — too far off"
    )


# ── Solver timing benchmark ────────────────────────────────────────────────────

def test_mpc_solver_timing(mpc):
    """Solver must fit in < 20ms to leave headroom in the 50Hz loop."""
    state = np.zeros(12)
    state[5] = 0.27
    state_ref = state.copy()
    foot_pos = np.zeros((4, 3))
    foot_pos[:, 2] = -0.27
    contact = [[True, False, False, True]] * 6

    n_runs = 20
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        mpc.solve(state, state_ref, foot_pos, contact)
        times.append((time.perf_counter() - t0) * 1000)

    mean_ms = np.mean(times)
    max_ms  = np.max(times)
    print(f"\n[timing] solver: mean={mean_ms:.1f}ms  max={max_ms:.1f}ms  (budget=20ms)")
    assert mean_ms < 20.0, (
        f"Solver too slow: mean={mean_ms:.1f}ms > 20ms. "
        f"Consider reducing horizon or switching to OSQP."
    )
