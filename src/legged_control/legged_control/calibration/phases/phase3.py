"""Phase 3: suspended PD gain tuning."""

from __future__ import annotations
import time

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.oscillation import OscillationDetector

KP_MAX = 40
KP_START = 5
KP_STEP = 5
KD_START = 0.5
KD_MAX = 2.0

# Use FR_thigh as the step-response test joint (significant gravity load when suspended)
_STEP_TEST_JOINT = 'FR_thigh'
_STEP_SIZE = 0.1   # rad


def run(joints_cfg: list[dict], config_path: str,
        ros_client, motor_manager, pause_ctrl) -> tuple[float, float]:
    """Tune kp then kd while robot is still suspended.

    Returns (kp, kd).
    """
    ui.phase_banner(3, 'PD Gain Tuning (suspended)')

    default_q = {j['name']: j['default_q'] for j in joints_cfg}

    kp = _tune_kp(default_q, ros_client, motor_manager, pause_ctrl)
    kd = _tune_kd(kp, default_q, ros_client, motor_manager, pause_ctrl)

    ConfigIO(config_path).patch({'control': {'kp': round(kp, 2), 'kd': round(kd, 3)}})
    ui.success(f'Gains written: kp={kp}, kd={kd}')
    return kp, kd


def _tune_kp(default_q: dict[str, float], ros_client,
             motor_manager, pause_ctrl) -> float:
    ui.info(f'\n--- kp sweep (kd fixed at {KD_START}) ---')
    ui.info('Motors will hold the standing pose with increasing stiffness.')
    ui.info('Watch for vibration/oscillation. The script will auto-detect and stop.')

    kp = KP_START
    last_stable_kp = 0  # 0 = no stable step confirmed yet; safe fallback is zero torque

    while kp <= KP_MAX:
        pause_ctrl.check()
        ui.info(f'\nApplying kp={kp}, kd={KD_START}...')
        motor_manager.launch(kp=kp, kd=KD_START, targets=default_q)
        time.sleep(0.5)

        if _monitor_oscillation(ros_client, pause_ctrl, duration=2.0):
            ui.warn(f'Oscillation detected at kp={kp}!')
            kp = last_stable_kp
            motor_manager.launch(kp=kp, kd=KD_START, targets=default_q)
            break

        last_stable_kp = kp
        ui.success(f'kp={kp} stable.')

        if kp >= KP_MAX:
            ui.info(f'Reached kp cap ({KP_MAX}).')
            break

        if not ui.confirm('Increase kp further?'):
            break

        kp = min(kp + KP_STEP, KP_MAX)

    recommended = round(last_stable_kp * 0.7, 1)
    ui.info(f'Recommended kp = 0.7 × {last_stable_kp} = {recommended}')
    raw = ui.prompt(f'Accept kp={recommended}? [Enter to accept or type value]:')
    if raw:
        try:
            recommended = min(float(raw), KP_MAX)
        except ValueError:
            ui.warn('Invalid input; using recommended value.')
    ui.success(f'kp set to {recommended}')
    return recommended


def _tune_kd(kp: float, default_q: dict[str, float], ros_client,
             motor_manager, pause_ctrl) -> float:
    ui.info(f'\n--- kd tuning (kp={kp} fixed) ---')
    ui.info(f'Sending a small step to {_STEP_TEST_JOINT} and measuring response.')

    kd = KD_START
    test_joint = _STEP_TEST_JOINT

    if test_joint not in default_q:
        ui.warn(f'{test_joint} not found in joints_cfg; skipping step test, keeping kd={kd}.')
        return kd

    motor_manager.launch(kp=kp, kd=kd, targets=default_q)
    time.sleep(0.5)

    # Step: perturb then revert
    pause_ctrl.check()
    step_target = dict(default_q)
    step_target[test_joint] = default_q[test_joint] + _STEP_SIZE
    ros_client.send_command(step_target)
    time.sleep(0.2)
    pause_ctrl.check()
    ros_client.send_command(default_q)

    # Record response for 1 second
    target_val = default_q[test_joint]
    peak_overshoot = _measure_peak_overshoot(
        ros_client, test_joint, target_val, duration=1.0)

    suggested_kd = suggest_kd_from_overshoot(kd, peak_overshoot, _STEP_SIZE)
    ui.info(f'Peak overshoot: {peak_overshoot:.4f} rad  (step={_STEP_SIZE} rad)')
    ui.info(f'Suggested kd: {suggested_kd:.3f}')

    raw = ui.prompt(f'Accept kd={suggested_kd:.3f}? [Enter to accept or type value]:')
    if raw:
        try:
            suggested_kd = float(raw)
        except ValueError:
            ui.warn('Invalid input; using suggested value.')
    suggested_kd = min(suggested_kd, KD_MAX)
    ui.success(f'kd set to {suggested_kd}')
    return suggested_kd


def _monitor_oscillation(ros_client, pause_ctrl,
                         duration: float, poll_hz: float = 50.0) -> bool:
    """Return True if oscillation detected in any joint during `duration` seconds."""
    detectors: dict[str, OscillationDetector] = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        pause_ctrl.check()
        t = time.time()
        for name, vel in ros_client.get_joint_velocities().items():
            d = detectors.setdefault(name, OscillationDetector())
            d.update(t, vel)
            if d.is_oscillating():
                return True
        time.sleep(1.0 / poll_hz)
    return False


def _measure_peak_overshoot(ros_client, joint_name: str,
                             target: float, duration: float) -> float:
    """Return maximum absolute deviation from target after a step response."""
    peak = 0.0
    deadline = time.time() + duration
    while time.time() < deadline:
        pos = ros_client.get_joint_positions().get(joint_name, target)
        peak = max(peak, abs(pos - target))
        time.sleep(0.02)
    return peak


# ------------------------------------------------- pure logic (testable)

def suggest_kd_from_overshoot(current_kd: float, peak_overshoot: float,
                               step: float) -> float:
    """Heuristic kd suggestion based on peak overshoot fraction of step size.

    overshoot_frac > 1.0 → increase kd by 50% (overshoot exceeds full step size)
    overshoot_frac < 0.20 → decrease kd by 30% (less than 20% overshoot → overdamped)
    else                  → keep current
    """
    frac = peak_overshoot / step if step > 0 else 0.0
    if frac > 1.0:
        return round(min(current_kd * 1.5, KD_MAX), 3)
    if frac < 0.20:
        return round(current_kd * 0.7, 3)
    return current_kd
