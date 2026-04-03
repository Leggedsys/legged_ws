"""Phase 4: ground full-stack verification and optional gain fine-tuning."""

from __future__ import annotations
import time
import numpy as np

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.oscillation import OscillationDetector

_KP_STEP = 2.0    # increment per kp adjustment step
_KD_STEP = 0.1    # increment per kd adjustment step
_KP_MAX  = 40.0   # hard cap (motor datasheet limit)
_KD_MAX  = 2.0    # hard cap


def run(joints_cfg: list[dict], config_path: str,
        ros_client, motor_manager, pause_ctrl) -> None:
    ui.phase_banner(4, 'Ground Verification')
    ui.warn('Place the robot on flat ground. Have a safety person nearby.')
    input('Ready? Press Enter.')

    cfg = ConfigIO(config_path).read()
    kp = cfg['control']['kp']
    kd = cfg['control']['kd']
    default_q = {j['name']: j['default_q'] for j in joints_cfg}

    motor_manager.launch(kp=kp, kd=kd, targets=default_q)
    ui.info('Control stack running. Live observation (Ctrl+C or ESTOP to pause):')

    _live_display(ros_client, pause_ctrl, duration=None)

    kp, kd = _fine_tune_loop(kp, kd, default_q, config_path,
                              ros_client, motor_manager, pause_ctrl)

    ConfigIO(config_path).patch({'control': {'kp': round(kp, 2), 'kd': round(kd, 3)}})
    ui.success(f'Final gains written: kp={kp}, kd={kd}')
    ui.success('Phase 4 complete. Calibration finished.')


def _live_display(ros_client, pause_ctrl, duration=None) -> None:
    """Print observation vector summary until user presses Enter or duration expires."""
    import threading
    stop = threading.Event()

    def _input_waiter():
        input('\n[Press Enter to stop live display]\n')
        stop.set()

    t = threading.Thread(target=_input_waiter, daemon=True)
    t.start()

    deadline = time.time() + duration if duration is not None else float('inf')
    while not stop.is_set() and time.time() < deadline:
        pause_ctrl.check()
        _print_obs(ros_client)
        time.sleep(1.0)


def _print_obs(ros_client) -> None:
    joy = ros_client.get_joy()
    imu = ros_client.get_imu()
    odom = ros_client.get_odom()
    pos = ros_client.get_joint_positions()
    vel = ros_client.get_joint_velocities()

    vx = vy = yaw = 0.0
    if joy:
        # Raw axes — user can verify mapping visually
        vx  = joy.axes[1] if len(joy.axes) > 1 else 0.0
        vy  = joy.axes[0] if len(joy.axes) > 0 else 0.0
        yaw = joy.axes[3] if len(joy.axes) > 3 else 0.0

    gx = gy = gz = 0.0
    if odom:
        q = odom.pose.pose.orientation
        gx, gy, gz = _gravity_in_body(q.x, q.y, q.z, q.w)

    wx = wy = wz = 0.0
    if imu:
        wx, wy, wz = imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z

    print(f'  cmd_vel   : vx={vx:+.2f} vy={vy:+.2f} yaw={yaw:+.2f}')
    print(f'  gravity   : x={gx:+.3f} y={gy:+.3f} z={gz:+.3f}  (flat→~0,0,-1)')
    print(f'  ang_vel   : x={wx:+.3f} y={wy:+.3f} z={wz:+.3f}')
    if pos:
        pos_str = '  '.join(f'{n}={v:+.2f}' for n, v in list(pos.items())[:4])
        print(f'  joint_pos : {pos_str}')
    if vel:
        vel_str = '  '.join(f'{n}={v:+.2f}' for n, v in list(vel.items())[:4])
        print(f'  joint_vel : {vel_str}')
    print()


def _gravity_in_body(qx, qy, qz, qw) -> tuple[float, float, float]:
    """Rotate world gravity vector [0,0,-1] into body frame.

    Uses passive rotation (q^-1 * v * q): v' = v(2w²-1) - 2w(q×v) + 2q(q·v)
    This matches policy_node._quat_rotate_inverse.
    """
    q_vec = np.array([qx, qy, qz])
    v = np.array([0.0, 0.0, -1.0])
    a = v * (2.0 * qw**2 - 1.0)
    b = np.cross(q_vec, v) * qw * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    g = a - b + c
    return float(g[0]), float(g[1]), float(g[2])


def _monitor_oscillation(ros_client, pause_ctrl,
                          duration: float, poll_hz: float = 50.0) -> bool:
    """Return True if oscillation detected in any joint during `duration` seconds."""
    detectors: dict[str, OscillationDetector] = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        pause_ctrl.check()
        t = time.time()
        for name, v in ros_client.get_joint_velocities().items():
            d = detectors.setdefault(name, OscillationDetector())
            d.update(t, v)
            if d.is_oscillating():
                return True
        time.sleep(1.0 / poll_hz)
    return False


def _fine_tune_loop(kp: float, kd: float, default_q: dict[str, float],
                    config_path: str, ros_client, motor_manager,
                    pause_ctrl) -> tuple[float, float]:
    """Offer ±kp/kd adjustments with oscillation guard. Returns final (kp, kd)."""
    while True:
        pause_ctrl.check()
        if not ui.confirm('\nAdjust gains?'):
            break

        ui.info(f'Current: kp={kp:.1f}, kd={kd:.3f}')
        raw_kp = ui.prompt(f'kp: [+{_KP_STEP:.0f} / -{_KP_STEP:.0f} / Enter=skip]:')
        raw_kd = ui.prompt(f'kd: [+{_KD_STEP:.1f} / -{_KD_STEP:.1f} / Enter=skip]:')

        new_kp = kp
        new_kd = kd
        if raw_kp == '+':
            new_kp = min(kp + _KP_STEP, _KP_MAX)
        elif raw_kp == '-':
            new_kp = max(kp - _KP_STEP, 0.0)
        if raw_kd == '+':
            new_kd = min(kd + _KD_STEP, _KD_MAX)
        elif raw_kd == '-':
            new_kd = max(kd - _KD_STEP, 0.0)

        if new_kp == kp and new_kd == kd:
            ui.info('No change.')
            continue

        ui.info(f'Applying kp={new_kp:.1f}, kd={new_kd:.3f}...')
        motor_manager.launch(kp=new_kp, kd=new_kd, targets=default_q)

        if _monitor_oscillation(ros_client, pause_ctrl, duration=2.0):
            ui.warn(f'Oscillation detected! Reverting to kp={kp:.1f}, kd={kd:.3f}.')
            motor_manager.launch(kp=kp, kd=kd, targets=default_q)
        else:
            kp, kd = new_kp, new_kd
            ui.success(f'kp={kp:.1f}, kd={kd:.3f} stable.')
            _live_display(ros_client, pause_ctrl, duration=10.0)

    return kp, kd
