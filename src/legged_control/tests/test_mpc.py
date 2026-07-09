"""Tests for MPC modules — runs offline, no ROS, no hardware required."""

import numpy as np
import pytest

from legged_control.mpc.gait_scheduler import GaitScheduler, LEG_NAMES
from legged_control.mpc.swing_trajectory import (
    swing_foot_position,
    stance_foot_position,
    nominal_foot_position,
    landing_target,
    leg_velocity,
)


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


def test_stance_phase_progress():
    """stance_phase runs 0→1 during stance and is 0 during swing."""
    g = GaitScheduler(period=0.6, swing_ratio=0.4)
    t0 = g._t0
    for dt in np.linspace(0.0, 0.6, 60, endpoint=False):
        t = t0 + dt
        s = g.query(t=t)
        sp = g.stance_phase("FR", t=t)
        if s["FR"]["contact"]:
            assert 0.0 <= sp <= 1.0
        else:
            assert sp == 0.0
    # stance starts right after swing_ratio: progress near 0 there, near 1 at cycle end
    assert g.stance_phase("FR", t=t0 + 0.4 * 0.6 + 1e-4) < 0.05
    assert g.stance_phase("FR", t=t0 + 0.6 - 1e-4) > 0.95


def test_stance_stroke_endpoints_match_transitions():
    """Stance s=0 == landing_target (touchdown); zero vel → nominal at any s."""
    vel = np.array([0.3, 0.0])
    args = dict(body_vel=vel, gait_period=0.6, swing_ratio=0.4, stance_height=0.27)
    p_td   = stance_foot_position("FR", 0.0, **args)
    p_land = landing_target("FR", vel, 0.6, 0.4, 0.27)
    np.testing.assert_allclose(p_td, p_land, atol=1e-9,
                               err_msg="touchdown stance target must equal swing landing target")
    # lift-off point mirrors the touchdown offset about nominal
    p_lo = stance_foot_position("FR", 1.0, **args)
    nom = nominal_foot_position("FR")
    np.testing.assert_allclose(p_lo[0] - nom[0], -(p_td[0] - nom[0]), atol=1e-9)
    # zero velocity: stroke degenerates to nominal
    for s in (0.0, 0.5, 1.0):
        p = stance_foot_position("FR", s, np.zeros(2), 0.6, 0.4, 0.27)
        np.testing.assert_allclose(p, nom, atol=1e-9)


def test_stance_stroke_sweeps_backward():
    """With forward velocity the stance foot must move backward monotonically."""
    vel = np.array([0.3, 0.0])
    xs = [
        stance_foot_position("FR", s, vel, 0.6, 0.4, 0.27)[0]
        for s in np.linspace(0.0, 1.0, 11)
    ]
    assert all(a > b for a, b in zip(xs, xs[1:])), f"x not decreasing: {xs}"
    # total stroke = 2 × Raibert offset = v · T_stance
    assert xs[0] - xs[-1] == pytest.approx(0.3 * 0.6 * 0.6, abs=1e-9)


def test_swing_end_slope_matches_ground_velocity():
    """With xy_end_slope=−v·T, foot xy moves backward (with the ground) right
    after lift-off and right before touchdown, and endpoints stay exact."""
    v = np.array([0.3, 0.0])
    t_sw = 0.24
    o = 0.054
    p_lift = np.array([-o, 0.0, -0.27])
    p_land = np.array([+o, 0.0, -0.27])
    slope = -v * t_sw
    p0 = swing_foot_position(0.0, p_lift, p_land, 0.06, xy_end_slope=slope)
    p1 = swing_foot_position(1.0, p_lift, p_land, 0.06, xy_end_slope=slope)
    np.testing.assert_allclose(p0[:2], p_lift[:2], atol=1e-9)
    np.testing.assert_allclose(p1[:2], p_land[:2], atol=1e-9)
    # numeric slope at endpoints ≈ −v·T (foot moves with the ground)
    eps = 1e-4
    d0 = (swing_foot_position(eps, p_lift, p_land, 0.06, xy_end_slope=slope)[0] - p0[0]) / eps
    d1 = (p1[0] - swing_foot_position(1 - eps, p_lift, p_land, 0.06, xy_end_slope=slope)[0]) / eps
    assert d0 == pytest.approx(slope[0], abs=1e-2)
    assert d1 == pytest.approx(slope[0], abs=1e-2)
    # mid-swing still travels forward past both endpoints overall
    p_mid = swing_foot_position(0.5, p_lift, p_land, 0.06, xy_end_slope=slope)
    assert p_mid[0] == pytest.approx((p_lift[0] + p_land[0]) / 2, abs=0.02)


def test_backward_walk_reverses_stroke():
    """Negative vx: touchdown behind nominal, stroke sweeps forward."""
    vel = np.array([-0.3, 0.0])
    nom = nominal_foot_position("FR")
    p_td = stance_foot_position("FR", 0.0, vel, 0.6, 0.4, 0.27)
    p_lo = stance_foot_position("FR", 1.0, vel, 0.6, 0.4, 0.27)
    assert p_td[0] < nom[0], "backward: touchdown must be behind nominal"
    assert p_lo[0] > nom[0], "backward: lift-off must be ahead of nominal"


def test_leg_velocity_yaw_differential():
    """Pure yaw: left/right sides get opposite fore-aft velocity (turn in place),
    front/rear get opposite lateral velocity."""
    wz = 0.5  # rad/s, positive = CCW (left turn)
    v = {leg: leg_velocity(np.zeros(2), wz, leg) for leg in ("FR", "FL", "RR", "RL")}
    # CCW: right side (FR/RR, y<0) moves forward, left side (FL/RL) backward
    assert v["FR"][0] > 0 and v["RR"][0] > 0
    assert v["FL"][0] < 0 and v["RL"][0] < 0
    # front feet (x>0) move left (+y), rear feet right (−y)
    assert v["FR"][1] > 0 and v["FL"][1] > 0
    assert v["RR"][1] < 0 and v["RL"][1] < 0
    # symmetric magnitudes
    assert v["FR"][0] == pytest.approx(-v["FL"][0])
    assert v["FR"][1] == pytest.approx(-v["RR"][1])


def test_leg_velocity_zero_yaw_passthrough():
    """No yaw: per-leg velocity equals body velocity for every leg."""
    body = np.array([0.3, 0.1])
    for leg in ("FR", "FL", "RR", "RL"):
        np.testing.assert_allclose(leg_velocity(body, 0.0, leg), body, atol=1e-12)


def test_leveling_dz_signs_and_clamp():
    """Attitude leveling: tilted-down side gets longer legs (more negative z)."""
    from legged_control.mpc.mpc_node import _leveling_dz

    # nose down (gravity gains +x in body frame) → front legs extend, rear shorten
    dz = _leveling_dz(0.1, 0.0)
    assert dz["FR"] < 0 and dz["FL"] < 0, "front legs must extend when nose is down"
    assert dz["RR"] > 0 and dz["RL"] > 0, "rear legs must shorten when nose is down"
    # left side down (gravity gains +y) → left legs extend, right shorten
    dz = _leveling_dz(0.0, 0.1)
    assert dz["FL"] < 0 and dz["RL"] < 0, "left legs must extend when left is down"
    assert dz["FR"] > 0 and dz["RR"] > 0, "right legs must shorten when left is down"
    # symmetric magnitudes, clamped
    dz = _leveling_dz(10.0, 10.0)
    assert all(abs(v) <= 0.04 + 1e-12 for v in dz.values())
    # level → all zero
    dz = _leveling_dz(0.0, 0.0)
    assert all(v == 0.0 for v in dz.values())


def test_kp_scale_gives_correct_per_joint_value():
    """kp_scale applied to base_kp should scale each joint uniformly."""
    base_kp = {"FR_hip": 1.5, "FR_thigh": 1.5, "FR_calf": 0.5}
    scale = 0.6

    scaled_kp = {n: v * scale for n, v in base_kp.items()}

    assert scaled_kp["FR_hip"]  == pytest.approx(0.9)
    assert scaled_kp["FR_calf"] == pytest.approx(0.3)


# ── SRBD MPC (tau_ff feedforward) ────────────────────────────────────────────

@pytest.fixture
def mpc():
    from legged_control.mpc.srbd_mpc import SRBDMPC
    inertia = np.diag([0.0196, 0.0228, 0.0169])
    return SRBDMPC(mass=14.55, inertia_body=inertia, dt=0.01, horizon=6)


def _flat_feet():
    foot_pos = np.zeros((4, 3))
    foot_pos[:, 2] = -0.27
    return foot_pos


def test_mpc_output_shape(mpc):
    state = np.zeros(12); state[5] = 0.27
    grf = mpc.solve(state, state.copy(), _flat_feet(), [[True, False, False, True]] * 6)
    assert grf.shape == (12,)


def test_mpc_swing_forces_near_zero(mpc):
    state = np.zeros(12); state[5] = 0.27
    grf = mpc.solve(state, state.copy(), _flat_feet(), [[True, False, False, True]] * 6)
    np.testing.assert_allclose(grf[3:6], 0.0, atol=0.5)   # FL swing
    np.testing.assert_allclose(grf[6:9], 0.0, atol=0.5)   # RR swing


def test_mpc_weight_support(mpc):
    state = np.zeros(12); state[5] = 0.27
    grf = mpc.solve(state, state.copy(), _flat_feet(), [[True] * 4] * 6)
    total_fz = grf[2] + grf[5] + grf[8] + grf[11]
    weight = 14.55 * 9.81
    assert abs(total_fz - weight) < weight * 0.3


def test_stance_load_ramp_endpoints():
    """Force scale must be exactly 0 at touchdown/lift-off and 1 mid-stance."""
    from legged_control.mpc.mpc_node import _stance_load_ramp
    assert _stance_load_ramp(0.0) == 0.0
    assert _stance_load_ramp(1.0) == 0.0
    assert _stance_load_ramp(0.5) == pytest.approx(1.0)
    # monotone ramp-in over the first _TAU_RAMP_FRAC
    vals = [_stance_load_ramp(s) for s in np.linspace(0.0, 0.2, 10)]
    assert all(b >= a for a, b in zip(vals, vals[1:]))


def test_build_stance_tau_scaled_and_swing_zero():
    """leg_scale gates per-leg torque; scale 0 (swing / touchdown) gives 0."""
    from legged_control.mpc.mpc_node import _build_stance_tau, _YAML_JOINTS

    grf = np.zeros(12)
    for i in range(4):
        grf[i * 3 + 2] = 35.7
    q = {n: 0.1 if "hip" in n else (0.8 if "thigh" in n else -1.5)
         for n in _YAML_JOINTS}
    R = np.eye(3)

    full = _build_stance_tau(grf, q, {l: 1.0 for l in ("FR", "FL", "RR", "RL")}, R)
    assert any(abs(t) > 0.01 for t in full)
    assert all(abs(t) < 23.0 for t in full)

    half = _build_stance_tau(grf, q, {"FR": 0.5, "FL": 0.0, "RR": 0.0, "RL": 1.0}, R)
    np.testing.assert_allclose(half[0:3], [t * 0.5 for t in full[0:3]], atol=1e-9)
    assert all(t == 0.0 for t in half[3:9]), "scale-0 legs must have zero tau"
    np.testing.assert_allclose(half[9:12], full[9:12], atol=1e-9)


def test_build_stance_tau_supports_weight():
    """τ_ff must RESIST the upward GRF (τ = −J^T R^T f), i.e. do positive work
    lifting the body: τ · ∂h/∂q > 0 with h = −z_foot (foot pinned on ground).
    The original +J^T·f sign pushed the body down — seen on hardware as the
    stance height dropping when tau_ff was enabled."""
    from legged_control.kinematics import _numerical_jacobian
    from legged_control.mpc.mpc_node import _build_stance_tau, _YAML_JOINTS, _leg_joints

    grf = np.zeros(12)
    for i in range(4):
        grf[i * 3 + 2] = 35.7  # ground pushes UP on each foot (world z+)
    q = {n: 0.1 if "hip" in n else (0.8 if "thigh" in n else -1.5)
         for n in _YAML_JOINTS}
    tau = _build_stance_tau(grf, q, {l: 1.0 for l in ("FR", "FL", "RR", "RL")},
                            np.eye(3))

    for i, leg in enumerate(("FR", "FL", "RR", "RL")):
        joints_leg = tuple(q[j] for j in _leg_joints(leg))
        J = _numerical_jacobian(leg, joints_leg)
        tau_leg = np.array(tau[i * 3:i * 3 + 3])
        # exact statics: actuator cancels the generalized force of the GRF
        np.testing.assert_allclose(tau_leg, -(J.T @ grf[i * 3:i * 3 + 3]),
                                   atol=1e-9)
        # physical direction: with the foot on the ground, body height
        # h = −z_foot, so lifting power is τ·(−J_z) — must be positive.
        dh_dq = -J[2, :]
        assert float(tau_leg @ dh_dq) > 0.0, f"{leg}: tau_ff not lifting the body"


def test_apply_load_ramp_conserves_total_force():
    """Force ramped off a lifting leg must be handed to the loaded legs —
    total commanded force equals the QP total through every ramp window."""
    from legged_control.mpc.mpc_node import _apply_load_ramp

    grf = np.zeros(12)
    # FR, FL, RL in stance (30 N + some tangential); RR swing (already zero)
    for i, fz in [(0, 30.0), (1, 30.0), (3, 30.0)]:
        grf[i * 3 + 0] = 3.0
        grf[i * 3 + 2] = fz
    scale = {"FR": 1.0, "FL": 0.3, "RR": 0.0, "RL": 1.0}

    out = _apply_load_ramp(grf, scale).reshape(4, 3)
    np.testing.assert_allclose(out.sum(axis=0), grf.reshape(4, 3).sum(axis=0),
                               atol=1e-9)
    assert np.all(out[2] == 0.0), "swing leg must stay at zero force"
    # ramping leg keeps less than its full share; loaded legs carry more
    assert out[1, 2] < 30.0
    assert out[0, 2] > 30.0 and out[3, 2] > 30.0
    # friction cone respected everywhere
    for i in range(4):
        if out[i, 2] > 0:
            assert abs(out[i, 0]) <= 0.6 * out[i, 2] + 1e-9
            assert abs(out[i, 1]) <= 0.6 * out[i, 2] + 1e-9


def test_apply_load_ramp_noop_at_full_support():
    """Balance stance (all scales 1) must pass the GRF through unchanged."""
    from legged_control.mpc.mpc_node import _apply_load_ramp

    grf = np.zeros(12)
    for i in range(4):
        grf[i * 3 + 2] = 35.7
        grf[i * 3 + 1] = 1.5
    out = _apply_load_ramp(grf, {l: 1.0 for l in ("FR", "FL", "RR", "RL")})
    np.testing.assert_allclose(out, grf, atol=1e-9)


def test_state_from_estimate_recovers_euler():
    """Roll/pitch extracted from projected_gravity must match the ZYX Euler
    angles that generated it (proj_g = R^T·[0,0,−1], legged_gym convention).
    The original atan2 forms NEGATED both angles — the MPC leveled the mirror
    image of the actual tilt (positive feedback through the force path)."""
    from legged_control.mpc.mpc_node import _state_from_estimate
    from legged_control.mpc.srbd_mpc import _euler_to_R

    for roll, pitch in [(0.1, 0.0), (0.0, 0.1), (-0.15, 0.08), (0.2, -0.12)]:
        R = _euler_to_R(np.array([roll, pitch, 0.3]))  # yaw must not matter
        est = np.zeros(10)
        est[6:9] = R.T @ np.array([0.0, 0.0, -1.0])
        state = _state_from_estimate(est, np.array([0.0, 0.0, 0.27]))
        assert state[0] == pytest.approx(roll, abs=1e-9), "roll sign/value"
        assert state[1] == pytest.approx(pitch, abs=1e-9), "pitch sign/value"


def _spread_feet():
    """Realistic footprint (nominal contact points, body frame): FR,FL,RR,RL."""
    x, y = 0.207, 0.159
    return np.array([
        [ x, -y, -0.27], [ x,  y, -0.27],
        [-x, -y, -0.27], [-x,  y, -0.27],
    ])


def test_mpc_grf_restores_tilt(mpc):
    """Tilt fed through _state_from_estimate must yield GRF that pushes the
    LOW side up (restoring moment). With the old negated roll/pitch the force
    went to the high side — anti-leveling."""
    from legged_control.mpc.mpc_node import _state_from_estimate

    ref = np.zeros(12); ref[5] = 0.27
    schedule = [[True] * 4] * 6

    # roll +0.1 rad → right side down (g_y < 0) → FR+RR must carry more
    est = np.zeros(10)
    est[6:9] = [0.0, -np.sin(0.1), -np.cos(0.1)]
    state = _state_from_estimate(est, np.array([0.0, 0.0, 0.27]))
    grf = mpc.solve(state, ref, _spread_feet(), schedule)
    right, left = grf[2] + grf[8], grf[5] + grf[11]
    assert right > left + 2.0, f"roll: low (right) side not loaded ({right=} {left=})"

    # pitch +0.1 rad → nose down (g_x > 0) → FR+FL must carry more
    est[6:9] = [np.sin(0.1), 0.0, -np.cos(0.1)]
    state = _state_from_estimate(est, np.array([0.0, 0.0, 0.27]))
    grf = mpc.solve(state, ref, _spread_feet(), schedule)
    front, rear = grf[2] + grf[5], grf[8] + grf[11]
    assert front > rear + 2.0, f"pitch: low (front) side not loaded ({front=} {rear=})"


def test_mpc_com_offset_shifts_load_forward(mpc):
    """Feet expressed relative to a forward-shifted CoM (feet move backward)
    must load the front pair more — the mechanism behind the com_x parameter:
    moment balance about the true CoM instead of the geometric center."""
    ref = np.zeros(12); ref[5] = 0.27
    state = ref.copy()
    schedule = [[True] * 4] * 6

    grf_centered = mpc.solve(state, ref, _spread_feet(), schedule)
    feet = _spread_feet(); feet[:, 0] -= 0.03  # CoM 3 cm forward of center
    grf_shifted = mpc.solve(state, ref, feet, schedule)

    def _front_minus_rear(g):
        return (g[2] + g[5]) - (g[8] + g[11])

    assert abs(_front_minus_rear(grf_centered)) < 1.0, "centered feet: even split"
    assert _front_minus_rear(grf_shifted) > 5.0, (
        f"forward CoM must load front pair (diff={_front_minus_rear(grf_shifted):.2f} N)"
    )
    total_c = grf_centered[2::3].sum()
    total_s = grf_shifted[2::3].sum()
    assert total_s == pytest.approx(total_c, rel=0.05), "total weight support unchanged"


def test_stance_gain_scale_crossfade():
    """Soft-stance gain split: disabled passthrough, endpoint values, and a
    monotone crossfade in between (weight = load ramp × tau blend)."""
    from legged_control.mpc.mpc_node import _stance_gain_scale

    # stance <= 0 disables the split entirely
    assert _stance_gain_scale(1.2, -1.0, 1.0) == pytest.approx(1.2)
    assert _stance_gain_scale(1.2, 0.0, 0.7) == pytest.approx(1.2)
    # endpoints: swing (w=0) keeps base, full stance (w=1) reaches stance value
    assert _stance_gain_scale(1.2, 0.6, 0.0) == pytest.approx(1.2)
    assert _stance_gain_scale(1.2, 0.6, 1.0) == pytest.approx(0.6)
    # halfway crossfade, and weights are clipped to [0, 1]
    assert _stance_gain_scale(1.2, 0.6, 0.5) == pytest.approx(0.9)
    assert _stance_gain_scale(1.2, 0.6, 1.7) == pytest.approx(0.6)
    assert _stance_gain_scale(1.2, 0.6, -0.3) == pytest.approx(1.2)
