#!/usr/bin/env python3
"""
calibrate.py — Robot initialization and calibration script.

Usage:
    source /opt/ros/humble/setup.bash
    source ~/rc/legged_ws/install/setup.bash
    python3 src/legged_control/scripts/calibrate.py [--config PATH]
"""

import argparse
import signal
import sys
import threading

import rclpy

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.ros_client import RosClient
from legged_control.calibration.motor_manager import MotorManager
from legged_control.calibration.estop import EStopMonitor, PauseController
from legged_control.calibration.phases import phase0, phase1, phase2, phase3, phase4

from ament_index_python.packages import get_package_share_directory
import os


def _default_config_path() -> str:
    share = get_package_share_directory('legged_control')
    return os.path.join(share, 'config', 'robot.yaml')


def main() -> None:
    parser = argparse.ArgumentParser(description='Legged robot calibration script')
    parser.add_argument('--config', default=_default_config_path(),
                        help='Path to robot.yaml')
    args = parser.parse_args()
    config_path = args.config

    # Read config
    cfg = ConfigIO(config_path).read()
    joints_cfg  = cfg['joints']
    serial_port = cfg['control']['serial_port']

    # Init ROS
    rclpy.init()
    ros_client = RosClient()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_client,), daemon=True)
    spin_thread.start()

    motor_manager = MotorManager(joints_cfg, serial_port)
    joy_proc = None

    # Signal handler: zero torque + exit on Ctrl+C
    def _sigint(sig, frame):
        ui.warn('\nCtrl+C — zeroing all motors and exiting.')
        motor_manager.zero_torque()
        motor_manager.shutdown()
        if joy_proc is not None:
            joy_proc.terminate()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    pause_ctrl = PauseController(motor_manager, ui)

    try:
        # Phase 0: env check + gamepad
        btn_index, joy_proc = phase0.run(config_path, ros_client)

        if btn_index >= 0:
            estop = EStopMonitor(ros_client, btn_index, pause_ctrl.trigger)
            estop.start()

        # Phase 1: motor ID mapping (writes to robot.yaml internally)
        phase1.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)
        # Reload joints_cfg with updated motor_ids
        joints_cfg = ConfigIO(config_path).read()['joints']

        # Phase 2: default_q sampling
        phase2.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)
        # Reload default_q after write
        joints_cfg = ConfigIO(config_path).read()['joints']

        # Phase 3: PD gain tuning (still suspended)
        phase3.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)

        # Phase 4: ground verification
        phase4.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)

        ui.success('\n=== Calibration complete. robot.yaml updated. ===')

    except SystemExit as e:
        raise
    finally:
        motor_manager.shutdown()
        if joy_proc is not None:
            joy_proc.terminate()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
