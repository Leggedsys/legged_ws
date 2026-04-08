"""
motor_bench_node

Single-leg joystick position test. Sweeps one of three joints through its
configured q_min/q_max range via left stick vertical. B/X buttons cycle
through joints; switching locks the departing joint at its last feedback
position.

Subscriptions:
  /joy                        sensor_msgs/Joy
  /<ns>/joint_states × 3      sensor_msgs/JointState

Publication:
  /joint_commands             sensor_msgs/JointState  (50 Hz)

Parameters:
  leg  string  default 'FR'   leg prefix, case-insensitive (FR/FL/RR/RL)
"""

_AXIS_POS = 1    # left stick vertical (inverted inside callback)
_BTN_NEXT = 1    # B button
_BTN_PREV = 2    # X button

_NS_MAP = {
    'FR': ['fr/hip', 'fr/thigh', 'fr/calf'],
    'FL': ['fl/hip', 'fl/thigh', 'fl/calf'],
    'RR': ['rr/hip', 'rr/thigh', 'rr/calf'],
    'RL': ['rl/hip', 'rl/thigh', 'rl/calf'],
}


def _apply_deadzone(v: float, dz: float = 0.05) -> float:
    """Return v rescaled so that |v| < dz maps to 0, full range stays ±1."""
    if abs(v) <= dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _map_to_range(v: float, q_min: float, q_max: float) -> float:
    """Map v ∈ [-1, 1] linearly to [q_min, q_max]."""
    return q_min + (v + 1.0) / 2.0 * (q_max - q_min)


def _select_joints(joints_cfg: list, leg: str) -> list:
    """Return the joints whose name starts with leg (case-insensitive)."""
    prefix = leg.upper()
    return [j for j in joints_cfg if j['name'].upper().startswith(prefix)]
