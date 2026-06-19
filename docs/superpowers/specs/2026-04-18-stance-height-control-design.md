# Stance Height Control Design

**Date:** 2026-04-18
**Branch:** dev/position-control

## Goal

Make `stance_height` the single authoritative source for standing height, and add real-time height control via gamepad triggers.

---

## Part 1: IK-Derived `_default_targets`

### Problem

Currently `default_q` (yaml motor-frame angles) and `stance_height` (gait param) are independent. `_default_targets` comes from `default_q` directly; `stance_height` only affects TROT foot-z. STANDUP always ramps to `default_q`, ignoring `stance_height`.

### Solution

At `GaitNode.__init__`, after `_nominal_feet` is computed, derive `_default_targets` from `stance_height` via IK:

```
for each leg in (FL, FR, RL, RR):
    foot_target = (nominal_feet[leg].x, nominal_feet[leg].y, -stance_height)
    q_urdf = inverse_kinematics(leg, foot_target, preferred=_leg_default_q_urdf[leg])
    if q_urdf is None:
        WARN + keep original default_q for this leg
        continue
    for each joint (hip, thigh, calf):
        q_motor = direction × (q_urdf[j] - zero_offset)
        _default_targets[joint_index] = q_motor
```

`_nominal_feet[leg]` z is also set to `-stance_height`.

`default_q` in `robot.yaml` is now only used as the IK preferred-joints hint (knee-up/down selection), not as a direct position target.

### Re-derivation on height change

Same procedure is called whenever `stance_height` changes during WAIT or TROT phases. Only `_default_targets` and `_nominal_feet` z are updated — the oscillator and phase are not reset.

---

## Part 2: Gamepad Height Control

### Data flow

```
Betop Kunpeng 20 triggers
  → joy_node /joy
  → teleop_node: dz = (rt - lt) × max_dz  →  Twist.linear.z
  → /cmd_vel
  → gait_node: stance_height += linear_z × dt  (WAIT/TROT only)
               → clamp [0.20, 0.35]
               → re-derive _default_targets via IK
```

### `teleop_node` changes

New keys in `robot.yaml` `teleop:` section:

```yaml
axis_lt: 2          # left trigger axis (joy_node: 0.0=released, 1.0=fully pressed)
axis_rt: 5          # right trigger axis
max_dz:  0.03       # m/s height rate limit (full range 0.15 m in ~5 s)
```

Trigger value convention (Betop Kunpeng 20 / Linux joy_node): 0.0 = not pressed, 1.0 = fully pressed. Apply same deadzone as other axes.

`dz = (_scale_axis(rt, deadzone, max_dz, invert=False) - _scale_axis(lt, deadzone, max_dz, invert=False))`

Published on `Twist.linear.z`.

### `gait_node` changes

New parameters (declared and readable from yaml `gait:` section):

```yaml
stance_height_min: 0.20   # m
stance_height_max: 0.35   # m
```

In `_on_cmd_vel`: store `msg.linear.z` as `_dz_rate`.

In `_tick` (WAIT and TROT phases only):

```python
if abs(self._dz_rate) > 0:
    new_h = clamp(stance_height + _dz_rate × dt, min, max)
    if abs(new_h - stance_height) > 0.0005:   # 0.5 mm threshold
        stance_height = new_h
        self.set_parameters([Parameter("stance_height", ..., new_h)])
        _rederive_defaults(new_h)
```

`_rederive_defaults(h)` is the same IK procedure from Part 1.

### Safety

- Height changes are **ignored** during STANDUP phase.
- IK failure on any leg during height change: that leg keeps previous `_default_targets`, logs WARN, height change still applies to other legs.
- `_dz_rate` is cleared (set to 0) when `_on_cmd_vel` is called with `linear.z ≈ 0`, i.e., when triggers are released.

---

## Files Modified

| File | Change |
|------|--------|
| `legged_control/gait_node.py` | IK-derived defaults at init; integrate dz; re-derive on change |
| `legged_control/teleop_node.py` | Read LT/RT axes; publish linear.z |
| `config/robot.yaml` | Add `teleop.axis_lt/rt/max_dz`; add `gait.stance_height_min/max` |
| `config/robot_sim.yaml` | Same additions |
| `tests/test_gait_node.py` | Tests for `_rederive_defaults`, height integration, clamping |
| `tests/test_teleop_node.py` | Tests for trigger axis reading and linear.z output |

---

## Non-Goals

- No height control during STANDUP.
- No per-leg independent height.
- No height feedback from IMU (open-loop).
