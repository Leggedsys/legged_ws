"""Phase 2: suspended default pose sampling."""

from __future__ import annotations
import time

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO


def run(joints_cfg: list[dict], config_path: str,
        ros_client, motor_manager, pause_ctrl) -> dict[str, float]:
    """Sample standing pose angles and write to robot.yaml.

    Returns dict of {joint_name: default_q}.
    """
    ui.phase_banner(2, 'Default Pose Sampling (suspended)')
    ui.info('Motors stay at zero-torque. Manually position all legs into')
    ui.info('an approximate standing pose, then press Enter to sample.')

    motor_manager.zero_torque()

    while True:
        pause_ctrl.check()
        input('\nAdjust pose, then press Enter to sample...')
        pause_ctrl.check()

        default_q = _sample(ros_client, duration=0.5, rate_hz=50.0)
        _print_table(default_q)

        if ui.confirm('Write these as default_q to robot.yaml?'):
            _write(config_path, default_q)
            ui.success('default_q written.')
            return default_q
        ui.info('Resampling...')


def _sample(ros_client, duration: float, rate_hz: float) -> dict[str, float]:
    """Continuously sample joint positions and return per-joint mean."""
    samples: dict[str, list[float]] = {}
    n = int(duration * rate_hz)
    for _ in range(n):
        for name, val in ros_client.get_joint_positions().items():
            samples.setdefault(name, []).append(val)
        time.sleep(1.0 / rate_hz)
    return {name: sum(vals) / len(vals) for name, vals in samples.items()}


def _print_table(default_q: dict[str, float]) -> None:
    rows = [(name, val) for name, val in default_q.items()]
    ui.joint_table(rows)


def _write(config_path: str, default_q: dict[str, float]) -> None:
    io = ConfigIO(config_path)
    for name, val in default_q.items():
        io.patch_joint(name, 'default_q', round(float(val), 4))
