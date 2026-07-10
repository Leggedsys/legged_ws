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


def test_remove_mount_bias_round_trip():
    """IMU mounting-bias removal: corrected proj_g of a body at (roll, pitch)
    with offsets (r_off, p_off) must equal proj_g at (roll−r_off, pitch−p_off),
    zero offsets pass through untouched, and the norm is preserved."""
    from legged_control.mpc.mpc_node import _remove_mount_bias, _state_from_estimate
    from legged_control.mpc.srbd_mpc import _euler_to_R

    def proj_g(roll, pitch):
        R = _euler_to_R(np.array([roll, pitch, 0.0]))
        return R.T @ np.array([0.0, 0.0, -1.0])

    g = proj_g(0.05, -0.08)
    assert np.allclose(_remove_mount_bias(g, 0.0, 0.0), g), "zero offsets: passthrough"

    for roll, pitch, r_off, p_off in [
        (0.0, 0.0, 0.01, -0.02), (0.06, -0.04, -0.03, 0.05), (-0.1, 0.12, 0.02, 0.02),
    ]:
        corrected = _remove_mount_bias(proj_g(roll, pitch), r_off, p_off)
        expected = proj_g(roll - r_off, pitch - p_off)
        assert np.allclose(corrected, expected, atol=1e-9), (roll, pitch, r_off, p_off)
        assert np.linalg.norm(corrected) == pytest.approx(1.0, abs=1e-9)

    # end-to-end: a body held exactly at the bias attitude reads as level
    est = np.zeros(10)
    est[6:9] = proj_g(0.03, -0.05)
    est[6:9] = _remove_mount_bias(est[6:9], 0.03, -0.05)
    state = _state_from_estimate(est, np.zeros(3))
    assert abs(state[0]) < 1e-9 and abs(state[1]) < 1e-9


def test_project_vertical_grf_removes_net_push(mpc):
    """With a forward CoM offset the raw QP balances part of the pitch moment
    with a net horizontal push (x cost weight is 0) — the body creeps forward
    as tau_ff blends in. The projection must zero net fx/fy while preserving
    total weight support and the roll/pitch moments via the fz split."""
    from legged_control.mpc.mpc_node import _project_vertical_grf

    ref = np.zeros(12); ref[5] = 0.27
    state = ref.copy()
    schedule = [[True] * 4] * 6
    feet = _spread_feet(); feet[:, 0] -= 0.05   # CoM 5 cm forward of center

    grf = mpc.solve(state, ref, feet, schedule)
    f_raw = grf.reshape(4, 3)
    assert abs(f_raw[:, 0].sum()) > 5.0, "precondition: raw QP does push horizontally"

    proj = _project_vertical_grf(grf, feet).reshape(4, 3)
    assert np.allclose(proj[:, 0], 0.0) and np.allclose(proj[:, 1], 0.0)
    assert proj[:, 2].sum() == pytest.approx(f_raw[:, 2].sum(), rel=1e-6), "weight kept"
    M_raw = np.cross(feet, f_raw).sum(axis=0)
    M_proj = np.cross(feet, proj).sum(axis=0)
    assert M_proj[0] == pytest.approx(M_raw[0], abs=1e-6), "roll moment kept"
    assert M_proj[1] == pytest.approx(M_raw[1], abs=1e-6), "pitch moment kept"
    # moment balance now through the fz split: front pair carries more
    assert proj[0, 2] + proj[1, 2] > proj[2, 2] + proj[3, 2] + 5.0

    # two-leg (trot) stance degrades gracefully: weight preserved, no NaN
    grf2 = grf.copy().reshape(4, 3)
    grf2[1] = 0.0; grf2[2] = 0.0            # only FR + RL loaded
    proj2 = _project_vertical_grf(grf2.reshape(-1), feet).reshape(4, 3)
    assert np.all(np.isfinite(proj2))
    assert proj2[:, 2].sum() == pytest.approx(grf2[:, 2].sum(), rel=1e-6)
    assert proj2[1, 2] == 0.0 and proj2[2, 2] == 0.0, "swing legs stay zero"


def test_stance_load_ramp_frac_param():
    """Runtime tau_ramp_frac: smaller frac reaches full load sooner."""
    from legged_control.mpc.mpc_node import _stance_load_ramp
    assert _stance_load_ramp(0.1, 0.1) == pytest.approx(1.0)
    assert _stance_load_ramp(0.1, 0.2) < 1.0
    assert _stance_load_ramp(0.95, 0.1) < 1.0   # lift-off ramp still applies
    assert _stance_load_ramp(0.0, 0.1) == 0.0
    assert _stance_load_ramp(1.0, 0.1) == 0.0


def test_measured_body_z_matches_fk():
    """Load-weighted stance FK height: matches plain FK, ignores unloaded
    legs, returns None with no load anywhere."""
    from legged_control.kinematics import forward_kinematics
    from legged_control.mpc.mpc_node import _measured_body_z

    q_leg = (0.0, 0.8, -1.6)
    joint_pos = {f"{leg}_{j}": v for leg in ["FR", "FL", "RR", "RL"]
                 for j, v in zip(["hip", "thigh", "calf"], q_leg)}
    joint_vel = {n: 0.0 for n in joint_pos}
    R = np.eye(3)

    h_fk = -forward_kinematics("FR", q_leg)[2]
    out = _measured_body_z(joint_pos, joint_vel, {l: 1.0 for l in ["FR", "FL", "RR", "RL"]}, R)
    assert out is not None
    assert out[0] == pytest.approx(h_fk, abs=1e-9)
    assert out[1] == pytest.approx(0.0, abs=1e-9)

    # a zero-weight leg is excluded: garbage joints there must not matter
    joint_pos["RL_calf"] = 2.5
    out2 = _measured_body_z(
        joint_pos, joint_vel, {"FR": 1.0, "FL": 1.0, "RR": 1.0, "RL": 0.0}, R
    )
    assert out2[0] == pytest.approx(h_fk, abs=1e-9)

    assert _measured_body_z(joint_pos, joint_vel, {}, R) is None


def test_measured_body_z_vz_finite_diff():
    """Vertical rate must equal the finite-difference of the FK height."""
    from legged_control.kinematics import forward_kinematics
    from legged_control.mpc.mpc_node import _measured_body_z

    q = (0.05, 0.9, -1.7)
    dq = (0.1, -0.4, 0.6)
    dt = 1e-6
    joint_pos = {f"FR_{j}": v for j, v in zip(["hip", "thigh", "calf"], q)}
    joint_vel = {f"FR_{j}": v for j, v in zip(["hip", "thigh", "calf"], dq)}
    # other legs unloaded
    for leg in ["FL", "RR", "RL"]:
        for j, v in zip(["hip", "thigh", "calf"], q):
            joint_pos[f"{leg}_{j}"] = v
            joint_vel[f"{leg}_{j}"] = 0.0
    out = _measured_body_z(
        joint_pos, joint_vel,
        {"FR": 1.0, "FL": 0.0, "RR": 0.0, "RL": 0.0}, np.eye(3),
    )
    q2 = tuple(qi + di * dt for qi, di in zip(q, dq))
    h1 = -forward_kinematics("FR", q)[2]
    h2 = -forward_kinematics("FR", q2)[2]
    assert out[1] == pytest.approx((h2 - h1) / dt, rel=1e-2)


def test_mpc_z_feedback_lifts_sagging_body(mpc):
    """A measured height below reference must raise total commanded lift —
    the closed height loop that replaces inflating the mass parameter."""
    mpc._Q[5, 5] = 2000.0                  # node default z_fb_weight
    ref = np.zeros(12); ref[5] = 0.27
    schedule = [[True] * 4] * 6
    feet = _spread_feet()

    fz_level = mpc.solve(ref.copy(), ref, feet, schedule).reshape(4, 3)[:, 2].sum()

    sag = ref.copy(); sag[5] = 0.23        # body 4 cm low
    fz_sag = mpc.solve(sag, ref, feet, schedule).reshape(4, 3)[:, 2].sum()
    assert fz_sag > fz_level + 80.0, "sag must command extra lift"

    fall = ref.copy(); fall[11] = -0.3     # body moving down
    fz_fall = mpc.solve(fall, ref, feet, schedule).reshape(4, 3)[:, 2].sum()
    assert fz_fall > fz_level + 50.0, "downward rate must command extra lift"

    high = ref.copy(); high[5] = 0.31      # body 4 cm high
    fz_high = mpc.solve(high, ref, feet, schedule).reshape(4, 3)[:, 2].sum()
    assert fz_high < fz_level - 20.0, "riding high must shed force"


def test_mpc_rate_feedback_damps_roll(mpc):
    """A measured body rate in the QP state must yield a counteracting moment
    (force-level attitude damping) that survives the vertical projection:
    +wx rolls the right (−y) side down → right pair pushed up harder, Mx < 0."""
    from legged_control.mpc.mpc_node import _project_vertical_grf

    ref = np.zeros(12); ref[5] = 0.27
    schedule = [[True] * 4] * 6
    feet = _spread_feet()

    still = _project_vertical_grf(
        mpc.solve(ref.copy(), ref, feet, schedule), feet
    ).reshape(4, 3)
    state = ref.copy(); state[6] = 1.0     # pure roll rate, attitude still level
    rolling = _project_vertical_grf(
        mpc.solve(state, ref, feet, schedule), feet
    ).reshape(4, 3)

    Mx_still = float(np.cross(feet, still).sum(axis=0)[0])
    Mx_roll = float(np.cross(feet, rolling).sum(axis=0)[0])
    assert abs(Mx_still) < 0.5, "no rate error → no damping moment"
    assert Mx_roll < -1.0, "damping moment must oppose the roll rate"
    right = rolling[0, 2] + rolling[2, 2]
    left = rolling[1, 2] + rolling[3, 2]
    assert right > left + 2.0, "falling side must be pushed up harder"
    # weight support must not be traded away for the damping moment
    assert rolling[:, 2].sum() == pytest.approx(still[:, 2].sum(), rel=0.05)


def test_q_weights_stay_float_under_runtime_pokes():
    """The node pokes _Q[i,i] with runtime param floats. An integer-dtype Q
    would silently truncate them — rate_fb_weight 0.15 became 0 on hardware,
    turning the damping channel fully off instead of merely weaker."""
    from legged_control.mpc.srbd_mpc import SRBDMPC
    m = SRBDMPC(mass=14.55, inertia_body=np.diag([0.02, 0.02, 0.02]))
    assert np.issubdtype(m._Q.dtype, np.floating)
    m._Q[6, 6] = 0.15
    assert m._Q[6, 6] == pytest.approx(0.15)


def _long_mpc(wz=800.0, wvz=5.0, wrate=1.0):
    """MPC at the deployed long-lookahead config (robot.yaml mpc_dt/horizon)."""
    from legged_control.mpc.srbd_mpc import SRBDMPC
    inertia = np.diag([0.0196, 0.0228, 0.0169])
    m = SRBDMPC(mass=13.6, inertia_body=inertia, dt=0.025, horizon=8)
    m._Q[5, 5] = wz
    m._Q[11, 11] = wvz
    m._Q[6, 6] = m._Q[7, 7] = wrate
    return m


def test_long_lookahead_gain_equivalence():
    """The robot.yaml weights for the 200 ms window must reproduce the
    hardware-validated effective z stiffness of the old 60 ms window
    (z_fb_weight 2000 @ dt 0.01 × N 6 ≈ 3200 N/m). Guards the calibration:
    anyone changing mpc_dt/horizon/weights must keep these gains matched."""
    m = _long_mpc()
    ref = np.zeros(12); ref[5] = 0.27
    schedule = [[True] * 4] * 8
    feet = _spread_feet()

    def sum_fz(state):
        return m.solve(state, ref, feet, schedule).reshape(4, 3)[:, 2].sum()

    base = sum_fz(ref.copy())
    sag = ref.copy(); sag[5] -= 0.02
    kz = (sum_fz(sag) - base) / 0.02
    assert 2700.0 < kz < 3700.0, f"z stiffness {kz:.0f} N/m drifted from ~3200"

    fall = ref.copy(); fall[11] = -0.2
    kvz = (sum_fz(fall) - base) / 0.2
    assert 300.0 < kvz < 600.0, f"vz damping {kvz:.0f} N/(m/s) drifted from ~440"


def test_long_lookahead_preloads_before_contact_flip():
    """The point of the long window: a lift-off scheduled mid-horizon must
    change the force NOW. With FR+RL leaving at step 2 (50 ms ahead) the QP
    should push harder in total — building upward momentum before support
    thins out — versus the same instant with no flip in sight."""
    m = _long_mpc()
    ref = np.zeros(12); ref[5] = 0.27
    feet = _spread_feet()

    steady = [[True] * 4] * 8
    flip = [[True] * 4] * 2 + [[True, False, False, True]] * 6  # FL+RR leave

    fz_steady = m.solve(ref.copy(), ref, feet, steady).reshape(4, 3)[:, 2]
    fz_flip = m.solve(ref.copy(), ref, feet, flip).reshape(4, 3)[:, 2]

    assert fz_flip.sum() > fz_steady.sum() + 5.0, (
        "QP must pre-load against the upcoming support loss"
    )
    # the legs that stay (FR idx 0, RL idx 3) should carry the increase
    staying = fz_flip[0] + fz_flip[3]
    steady_pair = fz_steady[0] + fz_steady[3]
    assert staying > steady_pair, "extra force must go to the legs that remain"


def test_fractional_contact_scales_continuously():
    """Contact shares (0..1) must shrink a leg's force bounds with the share
    and vanish at 0 — the schedule quantization fix: a flip time sliding
    across a prediction-step boundary may not step the solution."""
    m = _long_mpc()
    ref = np.zeros(12); ref[5] = 0.27
    feet = _spread_feet()

    def solve_fr(share):
        sched = [[share, 1.0, 1.0, 1.0]] + [[True] * 4] * 7
        return m.solve(ref.copy(), ref, feet, sched).reshape(4, 3)

    full = solve_fr(1.0)
    half = solve_fr(0.5)
    gone = solve_fr(0.0)
    assert gone[0].sum() == 0.0, "zero share must zero the force"
    assert half[0, 2] <= 0.5 * m._f_max + 1e-9, "bounds must scale with share"
    # continuity across the boolean end of the range
    near = solve_fr(0.999)
    assert abs(near[0, 2] - full[0, 2]) < 2.0, "share→1 must approach bool result"


def test_slew_anchor_pulls_solution_toward_previous():
    """With slew_weight on, solve(u_prev=…) must move part-way from u_prev
    toward the unanchored optimum and converge to it over repeated solves.
    Default slew_weight=0.0 keeps solve() history-free."""
    from legged_control.mpc.srbd_mpc import SRBDMPC
    inertia = np.diag([0.0196, 0.0228, 0.0169])
    ref = np.zeros(12); ref[5] = 0.27
    feet = _spread_feet()
    sched = [[True] * 4] * 8

    m0 = SRBDMPC(mass=13.6, inertia_body=inertia, dt=0.025, horizon=8)
    free = m0.solve(ref.copy(), ref, feet, sched)
    anchored_off = m0.solve(ref.copy(), ref, feet, sched, u_prev=free * 2.0)
    np.testing.assert_allclose(anchored_off, free, atol=1e-6)  # 0.0 → no-op

    m = SRBDMPC(mass=13.6, inertia_body=inertia, dt=0.025, horizon=8,
                slew_weight=1e-3)
    sag = ref.copy(); sag[5] -= 0.02
    target = m.solve(sag, ref, feet, sched).reshape(4, 3)[:, 2].sum()
    base_u = m.solve(ref.copy(), ref, feet, sched)
    base = base_u.reshape(4, 3)[:, 2].sum()

    u = base_u.copy()
    prev_fz = base
    for _ in range(10):
        u = m.solve(sag, ref, feet, sched, u_prev=u)
        fz = u.reshape(4, 3)[:, 2].sum()
        assert fz >= prev_fz - 1e-6, "anchored response must move monotonically"
        prev_fz = fz
    assert base + 0.6 * (target - base) < prev_fz <= target + 1.0, (
        "must converge toward the unanchored optimum"
    )


# ── TerrainEstimator ─────────────────────────────────────────────────────────

def _body_xy(leg, stance_h=0.27):
    """Nominal foot xy in the BODY frame (hip-frame nominal + hip mount),
    matching what mpc_node feeds the estimator."""
    from legged_control.mpc.swing_trajectory import _HIP_MOUNT_X, _HIP_MOUNT_Y
    from legged_control.kinematics import _leg_signs
    _, lat_sign, x_sign = _leg_signs(leg)
    nom = nominal_foot_position(leg, stance_h)
    return float(nom[0]) + x_sign * _HIP_MOUNT_X, float(nom[1]) + lat_sign * _HIP_MOUNT_Y


def _make_terrain(stance_h=0.27, lp_tau=0.4):
    from legged_control.mpc.terrain_estimator import TerrainEstimator
    foot_xy = {leg: _body_xy(leg, stance_h) for leg in LEG_NAMES}
    return TerrainEstimator(foot_xy, stance_height=stance_h, lp_tau=lp_tau)


def _feed(te, foot_body, R, seconds=8.0, dt=0.01):
    w = {leg: 1.0 for leg in LEG_NAMES}
    for _ in range(int(seconds / dt)):
        te.update(foot_body, w, R, dt)


def test_terrain_flat_is_noop():
    """Flat ground must reproduce the hardware-validated baseline exactly:
    zero slope, zero foot offsets, level attitude reference."""
    te = _make_terrain()
    flat = {
        leg: np.array([*_body_xy(leg), -0.27]) for leg in LEG_NAMES
    }
    _feed(te, flat, np.eye(3))
    assert np.allclose(te.world_slope, 0.0, atol=1e-9)
    assert te.dz(0.19, 0.12, np.eye(3)) == pytest.approx(0.0, abs=1e-9)
    r, p = te.ref_attitude()
    assert abs(r) < 1e-9 and abs(p) < 1e-9


def test_terrain_recovers_15deg_slope():
    """Body level, feet on a 15° x-slope → fit reports tan(15°) along x."""
    te = _make_terrain()
    a_true = np.tan(np.radians(15.0))
    feet = {}
    for leg in LEG_NAMES:
        x, y = _body_xy(leg)
        feet[leg] = np.array([x, y, -0.27 + a_true * x])
    _feed(te, feet, np.eye(3))
    assert te.world_slope[0] == pytest.approx(a_true, rel=0.02)
    assert te.world_slope[1] == pytest.approx(0.0, abs=1e-6)


def test_terrain_ref_attitude_aligns_body_z_with_normal():
    """R(ref_attitude)·ẑ must equal the terrain normal — the geometric
    definition of 'body parallel to slope', valid for any slope direction."""
    from legged_control.mpc.srbd_mpc import _euler_to_R
    te = _make_terrain()
    a, b = 0.2, -0.15  # oblique slope
    feet = {}
    for leg in LEG_NAMES:
        x, y = _body_xy(leg)
        feet[leg] = np.array([x, y, -0.27 + a * x + b * y])
    _feed(te, feet, np.eye(3))
    roll, pitch = te.ref_attitude()
    n = np.array([-a, -b, 1.0]); n /= np.linalg.norm(n)
    body_z = _euler_to_R(np.array([roll, pitch, 0.0])) @ np.array([0.0, 0.0, 1.0])
    assert np.allclose(body_z, n, atol=1e-6)
    # uphill along +x must reference nose-up, which is pitch < 0 in this
    # Euler convention (body x-axis world-z component = −sin(pitch))
    assert pitch < 0.0


def test_terrain_dz_vanishes_when_body_parallel():
    """THE steady-state invariant: body already riding parallel to the
    slope → feet are coplanar in the body frame → dz ≈ 0 (flat-ground
    behavior recovered on the slope itself)."""
    from legged_control.mpc.srbd_mpc import _euler_to_R
    te = _make_terrain()
    pitch = -np.radians(15.0)  # nose-up on an uphill
    R = _euler_to_R(np.array([0.0, pitch, 0.0]))
    # feet at constant extension in the BODY frame (equal leg lengths)
    feet = {
        leg: np.array([*_body_xy(leg), -0.27]) for leg in LEG_NAMES
    }
    _feed(te, feet, R)
    # world fit sees the 15° slope…
    assert te.world_slope[0] == pytest.approx(np.tan(np.radians(15.0)), rel=0.05)
    # …but body-frame offsets are ~zero: nothing changes for the gait
    a_b, b_b = te.body_plane(R)
    assert abs(a_b) < 1e-5 and abs(b_b) < 1e-5
    assert abs(te.dz(0.19, 0.12, R)) < 1e-5


def test_terrain_transition_raises_uphill_feet():
    """Body still level but front feet already on the incline (walking onto
    a ramp): front targets must rise, rear must drop — before touchdown."""
    te = _make_terrain()
    a_true = np.tan(np.radians(15.0))
    feet = {}
    for leg in LEG_NAMES:
        x, y = _body_xy(leg)
        z = -0.27 + (a_true * x if x > 0 else 0.0)  # only fronts on the ramp
        feet[leg] = np.array([x, y, z])
    _feed(te, feet, np.eye(3))
    front_x, _ = _body_xy("FR")
    assert te.dz(front_x, 0.0, np.eye(3)) > 0.01     # front foot raised
    assert te.dz(-front_x, 0.0, np.eye(3)) < -0.01   # rear foot lowered


def test_terrain_slope_and_dz_clamped():
    """A pathological fit (kinematic error, slipping foot) must saturate at
    the 20° slope clamp and the ±7 cm dz clamp instead of steering feet."""
    te = _make_terrain()
    feet = {}
    for leg in LEG_NAMES:
        x, y = _body_xy(leg)
        feet[leg] = np.array([x, y, -0.27 + 2.0 * x])  # "63° slope"
    _feed(te, feet, np.eye(3), seconds=10.0)
    assert abs(te.world_slope[0]) <= 0.364 + 1e-9
    assert abs(te.dz(1.0, 0.0, np.eye(3))) <= 0.07 + 1e-12


def test_terrain_lp_no_step():
    """Anchors jumping a full slope in one tick must not step the output:
    the LP bounds the per-tick change."""
    te = _make_terrain()
    a_true = np.tan(np.radians(15.0))
    feet = {}
    for leg in LEG_NAMES:
        x, y = _body_xy(leg)
        feet[leg] = np.array([x, y, -0.27 + a_true * x])
    w = {leg: 1.0 for leg in LEG_NAMES}
    prev = 0.0
    for _ in range(300):
        te.update(feet, w, np.eye(3), 0.01)
        cur = float(te.world_slope[0])
        assert abs(cur - prev) < a_true * (0.01 / 0.4) * 1.1
        prev = cur
    assert prev == pytest.approx(a_true, rel=0.05)


def test_terrain_untrusted_feet_do_not_move_anchors():
    """Legs below the load-trust threshold (swinging / barely touching)
    must not write anchors — a swing foot is not ground."""
    te = _make_terrain()
    bogus = {
        leg: np.array([*_body_xy(leg), 0.5]) for leg in LEG_NAMES
    }
    w = {leg: 0.3 for leg in LEG_NAMES}  # below trust threshold
    for _ in range(200):
        te.update(bogus, w, np.eye(3), 0.01)
    assert np.allclose(te.world_slope, 0.0, atol=1e-9)


def test_terrain_reset_returns_to_flat():
    te = _make_terrain()
    feet = {}
    for leg in LEG_NAMES:
        x, y = _body_xy(leg)
        feet[leg] = np.array([x, y, -0.27 + 0.2 * x])
    _feed(te, feet, np.eye(3))
    assert abs(te.world_slope[0]) > 0.1
    te.reset(0.27)
    # reset zeros the coefficients outright — carrying them across a
    # standup made the first WALK tick step into the previous lean
    assert np.allclose(te.world_slope, 0.0, atol=1e-12)


# ── Low-posture lateral spread ───────────────────────────────────────────────

def _knee_clearance(leg, h, spread):
    """Knee (thigh-calf joint) height above ground for a foot planted at
    (nominal x, D_LAT + spread, −h)."""
    from legged_control.kinematics import (
        inverse_kinematics, _leg_signs, D_LAT, L_HIP_X, L2,
    )
    hip_sign, lat_sign, x_sign = _leg_signs(leg)
    q = inverse_kinematics(leg, (x_sign * L_HIP_X, lat_sign * (D_LAT + spread), -h))
    assert q is not None
    hip = hip_sign * q[0]
    z_knee = lat_sign * D_LAT * np.sin(hip) - L2 * np.cos(q[1]) * np.cos(hip)
    return h + z_knee, q


def test_lateral_spread_schedule():
    from legged_control.mpc.mpc_node import _lateral_spread
    assert _lateral_spread(0.27, 0.22, 0.06) == 0.0   # normal height: no-op
    assert _lateral_spread(0.22, 0.22, 0.06) == 0.0
    assert _lateral_spread(0.20, 0.22, 0.06) == pytest.approx(0.02)
    assert _lateral_spread(0.16, 0.22, 0.06) == pytest.approx(0.06)  # capped
    # hip-limit taper at crouch heights: full 6 cm would need q1 > 0.4
    assert _lateral_spread(0.12, 0.22, 0.06) == pytest.approx(0.05)
    assert _lateral_spread(0.10, 0.22, 0.06) == pytest.approx(0.04)


def test_lateral_spread_raises_knee_clearance():
    """The point of the feature: at crouch heights the spread posture must
    measurably lift the knee off the ground, on front and rear legs."""
    for leg in ("FR", "RL"):
        c0, _ = _knee_clearance(leg, 0.17, 0.0)
        c6, _ = _knee_clearance(leg, 0.17, 0.06)
        assert c6 > c0 + 0.012, f"{leg}: {c0:.3f} → {c6:.3f}"


def test_lateral_spread_respects_hip_limit():
    """The SCHEDULED spread at every allowed height (down to _HEIGHT_MIN
    0.11) must stay inside the ±0.4 rad hip limit with margin for attitude
    corrections — otherwise the bridge clips and feet land inboard."""
    from legged_control.mpc.mpc_node import _lateral_spread
    for leg in LEG_NAMES:
        for h in (0.11, 0.12, 0.14, 0.16, 0.18, 0.20):
            s = _lateral_spread(h, 0.22, 0.06)
            _, q = _knee_clearance(leg, h, s)
            assert abs(q[0]) < 0.38, f"{leg} h={h} s={s}: q1={q[0]:.3f}"


def test_lateral_spread_relaxes_calf_fold():
    """Spread must also back the calf away from its −2.65 rad limit — the
    swing-phase fold margin that keeps low-height stepping feasible."""
    _, q0 = _knee_clearance("FR", 0.16, 0.0)
    _, q6 = _knee_clearance("FR", 0.16, 0.06)
    assert q6[2] > q0[2] + 0.2  # calf angle retreats ≥ 0.2 rad from the limit


def test_gait_swing_ratio_runtime_sync():
    """set_swing_ratio must move the contact boundary (runtime param sync)
    and clamp at 0.49 — a trot needs stance overlap of the diagonal pairs."""
    g = GaitScheduler(period=0.8, swing_ratio=0.4)
    t0 = g._t0
    # phase 0.45 for FR (trot offset 0): boundary moves with the ratio
    t = t0 + 0.45 * 0.8
    assert g.query(t)["FR"]["contact"]           # 0.45 >= 0.40 → stance
    g.set_swing_ratio(0.47)
    assert not g.query(t)["FR"]["contact"]       # 0.45 < 0.47 → swing
    assert g.swing_ratio == pytest.approx(0.47)
    g.set_swing_ratio(0.60)                      # out of range → clamp
    assert g.swing_ratio == pytest.approx(0.49)


# ── QP moment arms (hip mounts) & standing terrain freeze ────────────────────

def test_foot_positions_world_uses_body_frame_arms():
    """The QP's fore-aft lever arms must be ~±0.21 m (hip mounts included),
    not the ±0.065 m hip-frame values — the 3× understated arm made the QP
    load the front pair 113/24 N on flat level ground (hardware 2026-07-10)."""
    from legged_control.mpc.mpc_node import (
        _foot_positions_world, _DEFAULT_Q, _MPC_LEG_ORDER,
    )
    fp = _foot_positions_world(_DEFAULT_Q, np.eye(3), np.zeros(3))
    x = dict(zip(_MPC_LEG_ORDER, fp[:, 0]))
    assert x["FR"] > 0.14 and x["FL"] > 0.14, x
    assert x["RR"] < -0.14 and x["RL"] < -0.14, x
    # lateral arms too: D_LAT + hip mount ≈ ±0.21
    y = dict(zip(_MPC_LEG_ORDER, fp[:, 1]))
    assert y["FL"] > 0.15 and y["FR"] < -0.15, y


def test_standing_fz_split_with_correct_arms(mpc):
    """Static QP split on flat ground with com_x forward must be moderately
    front-biased (arm ratio), nowhere near the 5:1 the wrong arms produced."""
    from legged_control.mpc.mpc_node import _project_vertical_grf
    state = np.zeros(12); state[5] = 0.27
    ref = state.copy()
    feet = _spread_feet() - np.array([0.05, 0.0, 0.0])  # com_x 0.05
    grf = mpc.solve(state, ref, feet, [[True] * 4] * 6)
    grf = _project_vertical_grf(grf, feet)
    fz = grf.reshape(4, 3)[:, 2]
    front, rear = fz[0] + fz[1], fz[2] + fz[3]
    # arms 0.157 vs 0.257 → front/rear ≈ 1.64; allow slack for Q weighting
    assert 1.2 < front / rear < 2.3, (front, rear)


def test_terrain_hold_without_updates_decays_after_reset():
    """update() with no trusted feet must keep converging toward the stored
    anchors' fit — a standing robot after reset() decays to flat instead of
    holding a stale slope."""
    te = _make_terrain()
    feet = {leg: np.array([*_body_xy(leg), -0.27 + 0.2 * _body_xy(leg)[0]])
            for leg in LEG_NAMES}
    _feed(te, feet, np.eye(3))
    assert abs(te.world_slope[0]) > 0.15
    te.reset(0.27)
    for _ in range(800):           # standing: no trusted feet, LP still runs
        te.update({}, {}, np.eye(3), 0.01)
    assert abs(te.world_slope[0]) < 2e-3


def test_body_plane_conversion_conforms_to_tilt():
    """Documents the disturbance-transparency mechanism the node must
    filter: converting a world-flat plane through a rolled attitude yields
    dz that CONFORMS the feet to the roll (kills PD leveling stiffness if
    fed the instant attitude — that's why _terrain_tick uses a 0.5 s slow
    attitude for this conversion, while anchors keep the instant one)."""
    from legged_control.mpc.srbd_mpc import _euler_to_R
    te = _make_terrain()  # world plane stays flat (reset state)
    R_rolled = _euler_to_R(np.array([np.radians(5.0), 0.0, 0.0]))
    a_b, b_b = te.body_plane(R_rolled)
    y = _body_xy("FL")[1]
    # flat world plane + 5° roll → body-frame dz ≈ ±1.4 cm at the foot span
    assert abs(b_b * y) > 0.012
    # level attitude → no offsets
    a0, b0 = te.body_plane(np.eye(3))
    assert abs(a0) < 1e-9 and abs(b0) < 1e-9


# ── Touchdown detection (stairs) ─────────────────────────────────────────────

def test_leg_contact_threshold():
    from legged_control.mpc.mpc_node import _leg_contact
    eff = {"FR_calf": -5.2, "FL_calf": 0.8, "RR_calf": 3.4}
    assert _leg_contact(eff, "FR", 3.0)          # loaded, sign-agnostic
    assert not _leg_contact(eff, "FL", 3.0)      # swing-level torque
    assert _leg_contact(eff, "RR", 3.0)
    assert not _leg_contact(eff, "RL", 3.0)      # missing joint → no contact


def test_aggregator_forwards_joint_frame_effort():
    """Rotor tau must come out as direction × gear_ratio × tau — the
    contact-detection signal's units contract."""
    import importlib
    m = importlib.import_module("legged_control.real.joint_aggregator")
    # pure math check on the conversion used in _publish
    direction, gear_ratio, tau_rotor = -1.0, 12.66, 0.5
    assert direction * gear_ratio * tau_rotor == pytest.approx(-6.33)


# ── Crawl gait (stair mode) ──────────────────────────────────────────────────

def test_crawl_one_leg_at_a_time():
    """Statically stable invariant: at every instant of a crawl cycle at
    most ONE leg is in swing (three feet always planted)."""
    g = GaitScheduler(period=1.2, swing_ratio=0.4)  # 0.4 → clamped on switch
    g.set_mode("crawl")
    assert g.swing_ratio <= 0.24
    t0 = g._t0
    for dt in np.linspace(0.0, 1.2, 400, endpoint=False):
        st = g.query(t=t0 + dt)
        in_air = [leg for leg in LEG_NAMES if not st[leg]["contact"]]
        assert len(in_air) <= 1, f"dt={dt:.3f}: {in_air}"


def test_crawl_creep_sequence():
    """Swing order must be the classic creep RL → FL → RR → FR."""
    g = GaitScheduler(period=1.2, swing_ratio=0.2)
    g.set_mode("crawl")
    t0 = g._t0
    order = []
    for dt in np.linspace(0.0, 1.2, 1200, endpoint=False):
        st = g.query(t=t0 + dt)
        for leg in LEG_NAMES:
            if not st[leg]["contact"] and (not order or order[-1] != leg):
                order.append(leg)
    assert order == ["RL", "FL", "RR", "FR"], order


def test_sway_points_away_from_swing_corner():
    from legged_control.mpc.mpc_node import _sway_for
    # _sway_for returns the FEET-target offset. Swinging RL (rear-left):
    # body must move forward-right, so feet targets move rear-left (−x, +y).
    s_rl = _sway_for("RL")
    assert s_rl[0] < 0 and s_rl[1] > 0
    # Swinging FR (front-right): body rear-left → feet targets (+x, −y).
    s_fr = _sway_for("FR")
    assert s_fr[0] > 0 and s_fr[1] < 0
    # opposite corners get opposite sway
    assert np.allclose(_sway_for("RL"), -_sway_for("FR"))


# ── Operator step-up levels ──────────────────────────────────────────────────

def test_renorm_levels_common_offset_drains():
    """All four feet on the new step → the common offset slews to zero
    (the body climbs); rate-limited, never a step."""
    from legged_control.mpc.mpc_node import _renorm_levels, _LEVEL_RENORM_RATE
    dz = {leg: 0.10 for leg in LEG_NAMES}
    _renorm_levels(dz, 0.01)
    assert all(v == pytest.approx(0.10 - _LEVEL_RENORM_RATE * 0.01) for v in dz.values())
    for _ in range(1000):
        _renorm_levels(dz, 0.01)
    assert all(abs(v) < 1e-6 for v in dz.values())


def test_renorm_levels_straddle_holds():
    """Fronts up, rears not yet: offsets must NOT move — the straddle is
    real geometry, only the common part is bookkeeping."""
    from legged_control.mpc.mpc_node import _renorm_levels
    dz = {"FR": 0.10, "FL": 0.10, "RR": 0.0, "RL": 0.0}
    for _ in range(200):
        _renorm_levels(dz, 0.01)
    assert dz["FR"] == pytest.approx(0.10) and dz["RR"] == pytest.approx(0.0)


def test_renorm_levels_clear_drains_everything():
    from legged_control.mpc.mpc_node import _renorm_levels
    dz = {"FR": 0.08, "FL": -0.05, "RR": 0.0, "RL": 0.02}
    for _ in range(500):
        _renorm_levels(dz, 0.01, clear=True)
    assert all(abs(v) < 1e-6 for v in dz.values())

# ── Hurdle mode (cross a thin ~150 mm board) ─────────────────────────────────

def test_hurdle_flat_top_holds_full_height():
    """Trapezoid profile: full height across the whole centered flat window,
    ground-level endpoints — the board is cleared anywhere in the window."""
    from legged_control.mpc.swing_trajectory import swing_foot_position
    p0 = np.array([0.0645, 0.1127, -0.29])
    p1 = np.array([0.0645, 0.1127, -0.29])
    H = 0.17
    for s in (0.36, 0.5, 0.64):
        z = swing_foot_position(s, p0, p1, H, flat_top=0.3)[2]
        assert z == pytest.approx(-0.29 + H, abs=1e-9), s
    for s in (0.0, 1.0):
        z = swing_foot_position(s, p0, p1, H, flat_top=0.3)[2]
        assert z == pytest.approx(-0.29, abs=1e-9), s
    # >150 mm window must span a wide fraction of the swing (cosine arc
    # with the same 0.17 peak only manages ~23%)
    ss = np.linspace(0.0, 1.0, 1001)
    zs = np.array([swing_foot_position(s, p0, p1, H, flat_top=0.3)[2] for s in ss])
    assert float(np.mean(zs > -0.29 + 0.15)) > 0.40


def test_hurdle_vertical_speed_in_validated_band():
    """Peak vertical foot speed at the hurdle period floor must stay near the
    stair-v2 hardware-validated level (~0.8 m/s), despite the taller lift."""
    from legged_control.mpc.swing_trajectory import swing_foot_position
    from legged_control.mpc.mpc_node import (
        _HURDLE_MIN_PERIOD, _HURDLE_FLAT_TOP, _STAIR_SWING_RATIO,
    )
    t_sw = _HURDLE_MIN_PERIOD * _STAIR_SWING_RATIO
    p0 = np.array([0.0645, 0.1127, -0.29])
    ss = np.linspace(0.0, 1.0, 2001)
    zs = np.array([
        swing_foot_position(s, p0, p0, 0.17, flat_top=_HURDLE_FLAT_TOP)[2]
        for s in ss
    ])
    vz = np.abs(np.diff(zs) / (np.diff(ss) * t_sw))
    assert float(vz.max()) < 1.0


def test_hurdle_swing_ik_feasible_within_limits():
    """The whole hurdle swing (0.17 m lift, hurdle body height 0.29, stair
    stride at 0.08 m/s, period 3.5) must be IK-reachable AND inside the
    robot.yaml joint limits for a front and a rear leg."""
    from legged_control.mpc.swing_trajectory import (
        swing_foot_position, landing_target, stance_foot_position,
    )
    from legged_control.kinematics import inverse_kinematics
    period, ratio, h, lift = 3.5, 0.24, 0.29, 0.17
    v = np.array([0.08, 0.0])
    t_sw = period * ratio
    thigh_max = {"FR": 1.8, "RL": 2.0}
    for leg in ("FR", "RL"):
        p_lift = np.asarray(stance_foot_position(leg, 1.0, v, period, ratio, h))
        p_land = np.asarray(landing_target(leg, v, period, ratio, h))
        for s in np.linspace(0.0, 1.0, 41):
            p = swing_foot_position(
                s, p_lift, p_land, lift,
                xy_end_slope=-v * t_sw, flat_top=0.3,
            )
            q = inverse_kinematics(leg, tuple(p))
            assert q is not None, f"{leg} s={s:.3f} unreachable at {p}"
            q1, q2, q3 = q
            assert abs(q1) <= 0.4 + 1e-6, f"{leg} s={s:.3f} hip {q1:.3f}"
            assert -0.5 - 1e-6 <= q2 <= thigh_max[leg] + 1e-6, \
                f"{leg} s={s:.3f} thigh {q2:.3f}"
            assert -2.60 <= q3 <= -0.9 + 1e-6, \
                f"{leg} s={s:.3f} calf {q3:.3f} (fold margin 0.05)"


def test_hurdle_plain_arc_unchanged():
    """flat_top=0 (and the default) must reproduce the original raised-cosine
    arc bit-for-bit — hurdle mode off is the hardware-validated baseline."""
    from legged_control.mpc.swing_trajectory import swing_foot_position
    import math as _m
    p0 = np.array([0.05, 0.11, -0.235])
    p1 = np.array([0.09, 0.11, -0.235])
    for s in np.linspace(0.0, 1.0, 21):
        z_default = swing_foot_position(s, p0, p1, 0.04)[2]
        z_flat0 = swing_foot_position(s, p0, p1, 0.04, flat_top=0.0)[2]
        z_ref = -0.235 + 0.04 * 0.5 * (1.0 - _m.cos(2.0 * _m.pi * s))
        assert z_default == pytest.approx(z_ref, abs=1e-12)
        assert z_flat0 == pytest.approx(z_ref, abs=1e-12)

# ── Shin mode (断桥: kneel and crawl on the shins) ───────────────────────────

def test_shin_geometry_all_legs():
    """Shin pose FK: foot on the deck at shin body height, foot forward of
    the knee by ~L3, knee riding L3·sin(pitch) above the deck — identical
    planar geometry for all four legs (joint-space contract)."""
    from legged_control.mpc.shin_gait import (
        shin_pose_joints, shin_body_height, SHIN_PITCH,
    )
    from legged_control.kinematics import forward_kinematics, L2, L3, L_HIP_X
    q = shin_pose_joints()
    h = shin_body_height()
    assert h == pytest.approx(L2 + L3 * np.sin(SHIN_PITCH), abs=1e-12)
    for leg in LEG_NAMES:
        x, y, z = forward_kinematics(leg, q)
        assert z == pytest.approx(-h, abs=1e-9), leg           # foot on deck
        x_sign = 1.0 if leg in ("FR", "FL") else -1.0
        # foot ≈ L3 ahead of the knee (knee under hip at q2=0)
        assert x - x_sign * L_HIP_X == pytest.approx(
            L3 * np.cos(SHIN_PITCH), abs=1e-9
        ), leg
    # knee clearance above deck = L3·sin(pitch) ≈ 9 mm
    knee_z = -L2  # q2 = 0
    assert (-h) - knee_z == pytest.approx(-L3 * np.sin(SHIN_PITCH), abs=1e-12)


def test_shin_joint_limits_and_flat_contract():
    """Across the full stride and swing: hip 0, thigh far inside ±limits,
    calf inside [−2.65, −0.9] with ≥ 0.3 rad fold margin; stance keeps the
    shin-tilt identity q2 + q3 = −π/2 + pitch exactly."""
    from legged_control.mpc.shin_gait import (
        shin_stance_joints, shin_swing_joints, KNEE_OFFSET_MAX, SHIN_PITCH,
    )
    for off in (-KNEE_OFFSET_MAX, 0.0, KNEE_OFFSET_MAX):
        for s in np.linspace(0.0, 1.0, 41):
            q1, q2, q3 = shin_stance_joints(s, off)
            assert q1 == 0.0
            assert q2 + q3 == pytest.approx(-np.pi / 2 + SHIN_PITCH, abs=1e-9)
            assert abs(q2) <= 0.26
            assert -2.35 <= q3 <= -0.9
            q1, q2, q3 = shin_swing_joints(s, off)
            assert q1 == 0.0
            assert abs(q2) <= 0.26
            assert -2.35 <= q3 <= -0.9


def test_shin_body_height_bob_small():
    """Body height variation over the stance sweep at max stride ≤ 6 mm —
    the knee-under-hip sweet spot (cos flat near q2=0)."""
    from legged_control.mpc.shin_gait import (
        shin_stance_joints, shin_body_height, KNEE_OFFSET_MAX,
    )
    from legged_control.kinematics import forward_kinematics
    zs = [
        forward_kinematics("FR", shin_stance_joints(s, KNEE_OFFSET_MAX))[2]
        for s in np.linspace(0.0, 1.0, 41)
    ]
    assert max(zs) - min(zs) < 0.006
    # deepest foot target (= tallest body) is exactly mid-sweep, q2 = 0
    assert min(zs) == pytest.approx(-shin_body_height(), abs=1e-9)


def test_shin_swing_lifts_foot_and_is_continuous():
    """Mid-swing foot clearance ≥ 4 cm above the deck; the stance↔swing
    hand-offs match bit-for-bit so phase flips never step the command."""
    from legged_control.mpc.shin_gait import (
        shin_stance_joints, shin_swing_joints, shin_body_height,
    )
    from legged_control.kinematics import forward_kinematics
    off = 0.03
    z_mid = forward_kinematics("FR", shin_swing_joints(0.5, off))[2]
    assert z_mid - (-shin_body_height()) > 0.04
    for a, b in (
        (shin_stance_joints(1.0, off), shin_swing_joints(0.0, off)),
        (shin_swing_joints(1.0, off), shin_stance_joints(0.0, off)),
    ):
        assert np.allclose(a, b, atol=1e-9)
