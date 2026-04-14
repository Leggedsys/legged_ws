"""
teleop_node — reads /joy, publishes /cmd_vel for policy_node.

Pure functions (_apply_deadzone, _scale_axis) have no ROS2 dependency
and can be unit-tested directly.
"""

import os
import yaml


def _apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone and linearly remap to [-1, 1].

    Inside the deadzone (|value| < deadzone), returns 0.0.
    Outside, linearly remaps so that the deadzone edge maps to 0
    and ±1 maps to ±1 (no discontinuity at the boundary).
    """
    if deadzone >= 1.0:
        return 0.0
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _scale_axis(raw: float, deadzone: float, max_vel: float, invert: bool) -> float:
    """Apply deadzone, scale to physical units, and optionally invert.

    Returns velocity in the same units as max_vel (m/s or rad/s).
    """
    scaled = _apply_deadzone(raw, deadzone) * max_vel
    return -scaled if invert else scaled
