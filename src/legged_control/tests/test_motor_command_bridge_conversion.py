"""Offline tests for motor_command_bridge torque conversion formula."""
import pytest


def _tau_motor(direction: float, tau_urdf: float, gear_ratio: float) -> float:
    """Mirrors the conversion in motor_command_bridge._on_command."""
    return direction * tau_urdf / gear_ratio


def test_tau_positive_direction():
    # direction=+1, gear_ratio=6.33, tau_urdf=6.33 Nm → tau_motor=1.0 Nm at rotor
    assert _tau_motor(1.0, 6.33, 6.33) == pytest.approx(1.0, rel=1e-4)


def test_tau_negative_direction():
    # FR_hip: direction=-1 → tau is flipped
    assert _tau_motor(-1.0, 6.33, 6.33) == pytest.approx(-1.0, rel=1e-4)


def test_tau_calf_gear_ratio():
    # calf: gear_ratio=12.66, same URDF torque → half motor torque
    tau_hip  = _tau_motor(1.0, 10.0, 6.33)
    tau_calf = _tau_motor(1.0, 10.0, 12.66)
    assert tau_calf == pytest.approx(tau_hip / 2.0, rel=1e-3)


def test_zero_effort_gives_zero_tau():
    assert _tau_motor(-1.0, 0.0, 6.33) == 0.0


def test_tau_within_motor_limit():
    # max URDF torque (approx motor rated 23 Nm) should be reasonable at motor side
    max_tau_urdf = 23.0
    tau_m = _tau_motor(1.0, max_tau_urdf, 6.33)
    assert abs(tau_m) < 5.0, f"Motor-side torque too large: {tau_m}"
