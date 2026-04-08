# Motor Bench Test — Design Spec

**Date:** 2026-04-08
**Branch:** dev/motor-bench
**Status:** Approved

## Goal

Joystick-driven bench test for one assembled leg (3 joints: hip, thigh, calf).
Lets the operator manually sweep each joint through its full range to verify
motion and confirm that the `q_min`/`q_max` limits in `robot.yaml` are correct.

## Scope

- One leg at a time (configurable via launch arg `leg:=FR|FL|RR|RL`)
- Position control only, using kp/kd from `robot.yaml`
- No changes to existing nodes (`go_m8010_6_node`, `joint_aggregator`, `teleop_node`, etc.)

## New Files

| File | Purpose |
|------|---------|
| `src/legged_control/legged_control/motor_bench_node.py` | Joystick → joint position commands |
| `src/legged_control/launch/motor_bench.launch.py` | Starts 3 motor nodes + joy_node + motor_bench_node |

## Architecture

```
joy_node ──── /joy ──────► motor_bench_node ──► /joint_commands
                                  ▲
                 /<ns>/joint_states  (×3, for lock-on-switch feedback)

go_m8010_6_node × 3 ◄── /joint_commands  (each filters by joint_name)
```

## `motor_bench_node`

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `leg` | string | `'FR'` | Leg prefix (case-insensitive: `FR`/`fr` both work); selects 3 joints from `robot.yaml` whose name starts with this prefix (uppercase match against joint names like `FR_hip`) |

### Internal State

| Field | Description |
|-------|-------------|
| `active_idx` | Index 0/1/2 into the 3-joint list (0 = hip). Starts at 0. |
| `locked_q[3]` | Target position for each joint. Non-active joints hold this value. Initialized from `default_q`. |
| `feedback_q[3]` | Latest position feedback from each motor's `joint_states` topic. |

### Subscriptions

| Topic | Type | Use |
|-------|------|-----|
| `/joy` | `sensor_msgs/Joy` | Stick + button input |
| `/<ns>/joint_states` × 3 | `sensor_msgs/JointState` | Position feedback for lock-on-switch |

### Publications

| Topic | Type | Rate |
|-------|------|------|
| `/joint_commands` | `sensor_msgs/JointState` | 50 Hz timer |

### Control Logic

**Left stick vertical (axis 1, inverted, deadzone 0.05):**

Maps the deadzone-rescaled value `v ∈ [-1, 1]` to the active joint's range:

```
target_q = q_min + (v + 1) / 2 * (q_max - q_min)
```

This becomes `locked_q[active_idx]` on every Joy callback.

**B button (buttons[1]) — next joint:**

1. Detect falling edge (pressed this frame, not last frame)
2. `locked_q[active_idx] = feedback_q[active_idx]`  ← lock at current position
3. `active_idx = (active_idx + 1) % 3`
4. Log: `[motor_bench] active joint → <joint_name>`

**X button (buttons[2]) — previous joint:**

Same as B but `active_idx = (active_idx - 1) % 3`.

**50 Hz timer — publish `/joint_commands`:**

Publishes a single `JointState` with all 3 joint names and `locked_q` values.
The active joint's entry uses the latest stick-derived position; others use their locked value.

### Deadzone Rescaling

Identical to `teleop_node`:

```python
def _apply_deadzone(v, dz=0.05):
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)
```

## `motor_bench.launch.py`

### Launch Arguments

| Arg | Default | Description |
|-----|---------|-------------|
| `leg` | `'FR'` | Which leg to test |
| `joy_device` | `/dev/input/js0` | Joystick device path |

### Nodes Started

- 3 × `go_m8010_6_node` (package `unitree_actuator_sdk`)
  - Namespace: `fr/hip`, `fr/thigh`, `fr/calf` (adjusted per `leg` arg)
  - Parameters: `motor_id`, `joint_name`, `default_q`, `kp`, `kd` read from `robot.yaml`
  - `serial_port` from `robot.yaml control.serial_port`
  - `loop_hz` from `robot.yaml control.motor_hz`
- `joy_node` (package `joy`) with `device` = `joy_device`
- `motor_bench_node` (package `legged_control`) with `leg` parameter

### Usage

```bash
ros2 launch legged_control motor_bench.launch.py            # default FR leg
ros2 launch legged_control motor_bench.launch.py leg:=RL
ros2 launch legged_control motor_bench.launch.py leg:=FL joy_device:=/dev/input/js1
```

## Gamepad Layout Reference

| Input | Axis/Button | Action |
|-------|-------------|--------|
| Left stick vertical | axis 1 (inverted) | Control active joint position |
| B | buttons[1] | Next joint (hip→thigh→calf→hip) |
| X | buttons[2] | Previous joint |

## Changes to Existing Files

| File | Change |
|------|--------|
| `src/legged_control/setup.py` | Add `motor_bench_node` to `console_scripts`; add `motor_bench.launch.py` to `data_files` |

## Out of Scope

- Velocity mode
- PID runtime tuning
- More than one leg at a time
- Emergency stop (add after hardware testing confirms it's needed)
