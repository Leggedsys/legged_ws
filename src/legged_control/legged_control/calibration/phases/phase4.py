"""Phase 4: ground full-stack verification and optional gain fine-tuning."""

from __future__ import annotations
import subprocess
import time
import numpy as np

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.oscillation import OscillationDetector


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

    _fine_tune_loop(kp, kd, default_q, config_path,
                    ros_client, motor_manager, pause_ctrl)

    ui.success('Phase 4 complete. Calibration finished.')


def _live_display(ros_client, pause_ctrl, duration=10.0) -> None:
    """Print observation vector summary until user presses Enter or duration expires."""
    import threading
    stop = threading.Event()

    def _input_waiter():
        input('\n[Press Enter to stop live display]\n')
        stop.set()

    t = threading.Thread(target=_input_waiter, daemon=True)
    t.start()

    deadline = time.time() + duration if duration else float('inf')
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

    print(f'\r  cmd_vel : vx={vx:+.2f} vy={vy:+.2f} yaw={yaw:+.2f}')
    print(f'  gravity : x={gx:+.3f} y={gy:+.3f} z={gz:+.3f}  (flat→~0,0,-1)')
    print(f'  ang_vel : x={wx:+.3f} y={wy:+.3f} z={wz:+.3f}')
    if pos:
        pos_str = '  '.join(f'{n}={v:+.2f}' for n, v in list(pos.items())[:4])
        print(f'  joint_pos (first 4): {pos_str}')
    print()


def _gravity_in_body(qx, qy, qz, qw) -> tuple[float, float, float]:
    q_vec = np.array([qx, qy, qz])
    v = np.array([0.0, 0.0, -1.0])
    a = v * (2.0 * qw**2 - 1.0)
    b = np.cross(q_vec, v) * qw * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    g = a - b + c
    return float(g[0]), float(g[1]), float(g[2])


def _fine_tune_loop(kp: float, kd: float, default_q: dict[str, float],
                    config_path: str, ros_client, motor_manager, pause_ctrl) -> None:
    while True:
        pause_ctrl.check()
        if not ui.confirm('\nAdjust gains?'):
            break
        ui.info(f'Current: kp={kp}, kd={kd}')
        raw_kp = ui.prompt(f'New kp (Enter = keep {kp}):')
        raw_kd = ui.prompt(f'New kd (Enter = keep {kd}):')
        try:
            if raw_kp:
                kp = min(float(raw_kp), 40.0)
            if raw_kd:
                kd = min(float(raw_kd), 2.0)
        except ValueError:
            ui.warn('Invalid input; keeping current values.')
            continue

        motor_manager.launch(kp=kp, kd=kd, targets=default_q)
        _live_display(ros_client, pause_ctrl, duration=10.0)

    ConfigIO(config_path).patch({'control': {'kp': round(kp, 2), 'kd': round(kd, 3)}})
    ui.success(f'Final gains written: kp={kp}, kd={kd}')
