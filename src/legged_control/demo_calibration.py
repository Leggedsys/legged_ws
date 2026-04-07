#!/usr/bin/env python3
"""
demo_calibration.py — Run the calibration UI without ROS or real hardware.

All motor commands, ROS topics, and subprocess launches are stubbed out.
The terminal UI, prompts, config I/O, and phase logic run for real.

Usage:
    cd src/legged_control
    python3 demo_calibration.py [--phase 0|1|2|3|4]
"""

import argparse
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path


# ------------------------------------------------------------------ stubs

class _FakeJoy:
    """Simulates a Joy message with all axes and buttons zero."""
    axes    = [0.0] * 8
    buttons = [0]   * 12


class _FakeImu:
    class _Vec:
        x = 0.01; y = -0.02; z = 0.0
    angular_velocity = _Vec()


class _FakeOdom:
    """Simulates a level robot: identity quaternion → gravity (0,0,-1) in body."""
    class _Pose:
        class _Inner:
            class _Orientation:
                x = 0.0; y = 0.0; z = 0.0; w = 1.0
            orientation = _Orientation()
        pose = _Inner()
    pose = _Pose()


class FakeRosClient:
    """Stub for legged_control.calibration.ros_client.RosClient."""

    def __init__(self, joints_cfg):
        self._positions = {j['name']: j['default_q'] for j in joints_cfg}
        self._velocities = {j['name']: 0.0 for j in joints_cfg}
        self._joy = _FakeJoy()
        self._t0 = time.time()
        # Simulate gentle noise
        self._thread = threading.Thread(target=self._noise_loop, daemon=True)
        self._thread.start()

    def _noise_loop(self):
        while True:
            t = time.time() - self._t0
            for name in self._positions:
                self._positions[name] += 0.001 * math.sin(t * 2 + hash(name) % 10)
                self._velocities[name] = 0.001 * math.cos(t * 2 + hash(name) % 10)
            time.sleep(0.05)

    def get_joint_positions(self): return dict(self._positions)
    def get_joint_velocities(self): return dict(self._velocities)
    def get_joy(self):  return self._joy
    def get_imu(self):  return _FakeImu()
    def get_odom(self): return _FakeOdom()

    def send_command(self, targets):
        for name, val in targets.items():
            if name in self._positions:
                self._positions[name] = val


class FakeMotorManager:
    """Stub for legged_control.calibration.motor_manager.MotorManager."""

    def launch(self, kp=0.0, kd=0.0, targets=None):
        from legged_control.calibration import ui
        ui.info(f'  [stub] MotorManager.launch(kp={kp}, kd={kd})')
        time.sleep(0.1)   # skip the real 1-second wait

    def zero_torque(self):
        from legged_control.calibration import ui
        ui.info('  [stub] MotorManager.zero_torque()')

    def shutdown(self):
        from legged_control.calibration import ui
        ui.info('  [stub] MotorManager.shutdown()')


class FakePauseController:
    """Never pauses — ESTOP is keyboard-only in demo."""
    def check(self): pass
    def trigger(self): pass


# ------------------------------------------------------------------ main

DEMO_YAML = """\
# Demo robot.yaml (temporary copy used by demo)
joints:
  - name: FR_hip
    motor_id: 0
    default_q: 0.0
    q_min: -1.047
    q_max:  1.047
  - name: FR_thigh
    motor_id: 1
    default_q: 0.8
    q_min: -1.571
    q_max:  3.927
  - name: FR_calf
    motor_id: 2
    default_q: -1.5
    q_min: -2.723
    q_max: -0.837
  - name: FL_hip
    motor_id: 3
    default_q: 0.0
    q_min: -1.047
    q_max:  1.047
  - name: FL_thigh
    motor_id: 4
    default_q: 0.8
    q_min: -1.571
    q_max:  3.927
  - name: FL_calf
    motor_id: 5
    default_q: -1.5
    q_min: -2.723
    q_max: -0.837
  - name: RR_hip
    motor_id: 6
    default_q: 0.0
    q_min: -1.047
    q_max:  1.047
  - name: RR_thigh
    motor_id: 7
    default_q: 0.8
    q_min: -1.571
    q_max:  3.927
  - name: RR_calf
    motor_id: 8
    default_q: -1.5
    q_min: -2.723
    q_max: -0.837
  - name: RL_hip
    motor_id: 9
    default_q: 0.0
    q_min: -1.047
    q_max:  1.047
  - name: RL_thigh
    motor_id: 10
    default_q: 0.8
    q_min: -1.571
    q_max:  3.927
  - name: RL_calf
    motor_id: 11
    default_q: -1.5
    q_min: -2.723
    q_max: -0.837

control:
  serial_port: /dev/ttyUSB0
  motor_hz: 1000.0
  policy_hz: 50.0
  kp: 20.0
  kd: 0.5
  action_scale: 0.25

teleop:
  max_vx: 1.0
  max_vy: 0.5
  max_yaw: 1.0
  deadzone: 0.05
  axis_vx: 1
  axis_vy: 0
  axis_yaw: 3
  invert_vx: true
  invert_vy: false
  invert_yaw: true
  btn_emergency_stop: -1

policy:
  model_path: ''
  obs_mean: []
  obs_std: []
"""


def run_demo(phase_num: int, config_path: str) -> None:
    from legged_control.calibration import ui
    from legged_control.calibration.config_io import ConfigIO

    joints_cfg = ConfigIO(config_path).read()['joints']
    ros_client  = FakeRosClient(joints_cfg)
    motor_mgr   = FakeMotorManager()
    pause_ctrl  = FakePauseController()

    if phase_num == 0:
        # Phase 0 has env checks that will fail (no serial port, no ROS env)
        # So just demo the UI helpers instead
        ui.phase_banner(0, 'Environment Check + Gamepad Setup  [DEMO]')
        ui.success('Serial port /dev/ttyUSB0 found.')
        ui.success('ROS2 (humble) sourced.')
        ui.success('robot.yaml is writable.')
        ui.success('Python dependencies present.')
        ui.success('Environment checks passed.')
        ui.warn('No gamepad found at /dev/input/js0.')
        ui.info('  [demo: skipping gamepad binding]')

    elif phase_num == 2:
        from legged_control.calibration.phases import phase2
        phase2.run(joints_cfg, config_path, ros_client, motor_mgr, pause_ctrl)

    elif phase_num == 3:
        from legged_control.calibration.phases import phase3
        phase3.run(joints_cfg, config_path, ros_client, motor_mgr, pause_ctrl)

    elif phase_num == 4:
        from legged_control.calibration.phases import phase4
        phase4.run(joints_cfg, config_path, ros_client, motor_mgr, pause_ctrl)

    else:
        # Run ALL phases in sequence
        run_demo(0, config_path)
        run_demo(2, config_path)
        run_demo(3, config_path)
        run_demo(4, config_path)



def main():
    parser = argparse.ArgumentParser(description='Calibration UI demo (no ROS needed)')
    parser.add_argument('--phase', type=int, default=2,
                        choices=[0, 2, 3, 4],
                        help='Which phase to demo (default: 2 = default_q sampling)')
    args = parser.parse_args()

    # Write temp yaml
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                     delete=False, prefix='demo_robot_')
    tmp.write(DEMO_YAML)
    tmp.flush()
    config_path = tmp.name

    from legged_control.calibration import ui
    ui.info(f'\nDemo config: {config_path}')
    ui.info('(Ctrl+C to exit at any time)\n')

    try:
        run_demo(args.phase, config_path)
    except (KeyboardInterrupt, SystemExit):
        ui.warn('\nDemo exited.')
    finally:
        os.unlink(config_path)


if __name__ == '__main__':
    main()
