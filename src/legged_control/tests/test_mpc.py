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
    expected = nominal_foot_position("FR")
    np.testing.assert_allclose(p, expected, atol=1e-9,
                               err_msg="zero velocity → foot at nominal stance position")


def test_landing_target_clamped():
    p = landing_target("FR", np.array([10.0, 10.0]), gait_period=0.6, swing_ratio=0.4)
    nom = nominal_foot_position("FR")
    assert abs(p[0] - nom[0]) <= 0.12 and abs(p[1] - nom[1]) <= 0.06, (
        "Raibert offset should be clamped to ±0.12m x, ±0.06m y"
    )


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


# ── MIT Cheetah τ_ff ──────────────────────────────────────────────────────────

def test_tau_ff_direction_and_magnitude():
    """J^T · f_contact should produce plausible stance joint torques."""
    import sys
    sys.path.insert(0, "src/legged_control")
    from legged_control.kinematics import _numerical_jacobian
    import numpy as np

    joints_fr = (0.1, 0.8, -1.5)
    J = _numerical_jacobian("FR", joints_fr)

    # 四腿均分 14.55 kg 体重的竖直支撑力
    fz = 14.55 * 9.81 / 4.0
    f = np.array([0.0, 0.0, fz])
    tau = J.T @ f

    # 所有关节力矩绝对值 < 电机额定 23 Nm
    assert np.all(np.abs(tau) < 23.0), f"τ exceeds motor limit: {tau}"
    # 大腿（index 1）应为正力矩（支撑体重）
    assert tau[1] > 0.0, f"Thigh τ should be positive, got {tau[1]:.3f}"
    # 小腿（index 2）应为负力矩（膝关节弯曲对抗重力）
    assert tau[2] < 0.0, f"Calf τ should be negative, got {tau[2]:.3f}"


def test_kp_scale_gives_correct_per_joint_value():
    """Stance scale 0.25 applied to base_kp=1.5 should give 0.375."""
    base_kp = {"FR_hip": 1.5, "FR_thigh": 1.5, "FR_calf": 0.5}
    stance_scale = 0.25
    swing_scale  = 2.0

    stance_kp = {n: v * stance_scale for n, v in base_kp.items()}
    swing_kp  = {n: v * swing_scale  for n, v in base_kp.items()}

    assert stance_kp["FR_hip"]   == pytest.approx(0.375)
    assert stance_kp["FR_calf"]  == pytest.approx(0.125)
    assert swing_kp["FR_hip"]    == pytest.approx(3.0)
    assert swing_kp["FR_calf"]   == pytest.approx(1.0)


def test_balance_stance_returns_joint_command_with_tau():
    """_build_stance_tau should return a 12-element list with non-zero torques for stance legs."""
    import sys
    sys.path.insert(0, "src/legged_control")
    import numpy as np
    from legged_control.mpc.mpc_node import _build_stance_tau, _YAML_JOINTS

    # 给定已知 GRF（每腿 35.7 N 竖直）
    grf = np.zeros(12)
    for i in range(4):
        grf[i * 3 + 2] = 35.7  # fz

    q_targets = {n: 0.1 if "hip" in n else (0.8 if "thigh" in n else -1.5)
                 for n in _YAML_JOINTS}
    tau = _build_stance_tau(grf, q_targets)

    assert len(tau) == 12
    # 至少一个关节有非零力矩
    assert any(abs(t) > 0.01 for t in tau), "All torques are zero — J^T·f not applied"
    # 无关节超过电机额定
    assert all(abs(t) < 23.0 for t in tau), f"Torque out of range: {tau}"


def test_build_stance_tau_swing_legs_zero():
    """Swing legs must have zero torque regardless of GRF."""
    import sys
    sys.path.insert(0, "src/legged_control")
    import numpy as np
    from legged_control.mpc.mpc_node import _build_stance_tau, _YAML_JOINTS

    grf = np.ones(12) * 50.0  # non-zero GRF
    q_targets = {n: 0.1 if "hip" in n else (0.8 if "thigh" in n else -1.5)
                 for n in _YAML_JOINTS}

    # FR(0) stance, FL(1) swing, RR(2) swing, RL(3) stance
    contact = [True, False, False, True]
    tau = _build_stance_tau(grf, q_targets, contact_now=contact)

    # FL joints (index 3,4,5) and RR joints (index 6,7,8) should be 0
    fl_tau = tau[3:6]
    rr_tau = tau[6:9]
    assert all(t == 0.0 for t in fl_tau), f"FL swing should have zero tau: {fl_tau}"
    assert all(t == 0.0 for t in rr_tau), f"RR swing should have zero tau: {rr_tau}"
    # FR and RL should be non-zero
    assert any(abs(t) > 0.0 for t in tau[0:3]), "FR stance should have non-zero tau"
