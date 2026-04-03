# Calibration Script Design

Date: 2026-04-03
Status: Approved

## Overview

A single interactive Python script (`src/legged_control/scripts/calibrate.py`) that
guides the user through full robot initialization from a safe suspended state to
verified ground operation, automatically writing all measured values to `robot.yaml`.

## Architecture

- **Runtime**: Python 3, runs on onboard computer after sourcing ROS2 + workspace
- **ROS integration**: embeds `rclpy` directly to subscribe to topics; launches motor
  nodes via `subprocess`
- **Config I/O**: reads/writes `robot.yaml` via `ruamel.yaml` to preserve comments
- **UI**: colorized terminal output (`colorama` or ANSI codes), press-Enter confirmation
  at each step, Ctrl+C signal handler for emergency zero-torque exit

```
python3 src/legged_control/scripts/calibrate.py
```

## Safety Mechanisms

Applied globally throughout all phases:

| Mechanism | Detail |
|-----------|--------|
| Joint limit clamp | Every outgoing command is clamped to `q_min`/`q_max` before sending |
| Emergency stop | Ctrl+C → signal handler sends `kp=0, kd=0, tau=0` to all motors, then exits |
| Oscillation detection | If a joint's velocity sign reverses >3 times/sec → kp halved automatically |
| kp hard cap | Script refuses to raise kp above 40 under any circumstances |
| kd hard cap | Script refuses to raise kd above 2.0 |

## Phase 0 — Environment Check

Runs automatically on startup. Checks:

1. `/dev/ttyUSB0` (or configured serial port) exists and is accessible
2. `ROS_DISTRO` env var is set (ROS2 sourced)
3. `robot.yaml` exists and is writable
4. Required Python packages available (`rclpy`, `ruamel.yaml`)

Any failure prints a specific fix instruction and exits immediately.

## Phase 1 — Suspended · Motor ID Mapping

**Precondition**: user confirms robot is suspended off the ground.

All 12 motors set to zero-torque (kp=0, kd=0, tau=0). Script records baseline
joint angles, then guides through each joint in order:

**Confirmation order** (easiest to isolate first):
```
FR_calf, FL_calf, RR_calf, RL_calf   → small legs, most independent
FR_thigh, FL_thigh, RR_thigh, RL_thigh
FR_hip, FL_hip, RR_hip, RL_hip       → last, movement propagates to other joints
```

**Per-joint loop**:
1. Prompt: "Please wiggle the joint you believe is `<name>` (large movement)"
2. Monitor all motor angles; when any delta exceeds 0.3 rad, identify `motor_id`
   with largest change
3. Display: "motor_id=X changed by Δ=Y rad. Is this `<name>`? [y/n]"
4. On `n`: retry (max 3 attempts), then mark as "pending manual resolution"
5. On `y`: record mapping, continue

**Conflict resolution**:
- If detected `motor_id` already assigned to another joint:
  prompt "motor_id=X was assigned to `<prev>`. Overwrite, or re-wiggle? [o/r]"
- After all joints processed, if duplicates or gaps remain: print conflict table,
  resolve each entry interactively
- Final gate: all 12 motor_ids unique, all 12 joints assigned → proceed

**Output**: `motor_id` mapping written to `robot.yaml` joints list.

## Phase 2 — Suspended · Default Pose Sampling

**Precondition**: motor ID mapping complete; robot still suspended.

All 12 motors remain at zero-torque.

1. Prompt: "Manually position all legs into an approximate standing pose, then press Enter"
2. On Enter: sample all 12 joint angles continuously for 0.5 seconds, compute mean
3. Display table of sampled angles per joint
4. Confirm: "Write these as default_q? [y / resample]"
5. On resample: return to step 1

**Notes**:
- Precision is not critical; `default_q` is a baseline that will be refined after
  first ground test
- 0.5-second mean filters out hand-tremor noise

**Output**: `default_q` written for each joint in `robot.yaml`.

## Phase 3 — Suspended · PD Gain Rough Tuning

**Precondition**: `default_q` written; robot still suspended.

Target for all joints = `default_q`. Tune `kp` first, then `kd`.

### kp sweep

Start: `kp=5, kd=0.5`

Loop:
1. Apply current `kp`, wait 2 seconds
2. Monitor joint velocities for oscillation (velocity sign reversal >3×/sec)
   - Oscillation detected → auto-reduce to previous `kp`, report limit, exit loop
3. If stable: "kp=X stable. Increase? [y / stop here]"
4. On `y`: `kp += 5`, repeat
5. Hard cap: refuse to exceed `kp=40`

Recommended `kp` = 0.7 × oscillation threshold (or last stable value if cap reached).

### kd tuning

With `kp` locked:
1. Send `default_q + 0.1 rad` to a single test joint, then immediately revert to `default_q`
2. Measure: overshoot amplitude, settling time
3. Script computes suggested `kd` from step response; display with rationale
4. "Accept suggested kd=X? [y / enter manually]"

**Output**: `kp` and `kd` written to `control` section of `robot.yaml`.

## Phase 4 — Ground · Full Stack Verification + Fine-Tuning

**Precondition**: user confirms robot placed on ground; safety person present.

Launch full control stack (stand-still placeholder policy):
- 12 motor nodes, joint_aggregator, policy_node (no model), teleop_node

Print live observation vector summary every second:
```
cmd_vel    : vx=0.00  vy=0.00  yaw=0.00
gravity    : x=+0.01  y=-0.02  z=-0.99   ← should be ~(0,0,-1) on flat ground
ang_vel    : x=+0.00  y=+0.01  z=+0.00
joint_pos  : [FR_hip=-0.01, FR_thigh=+0.79, ...]   ← should be near default_q
joint_vel  : [all near 0.00]
```

User inspects values, confirms robot is standing stably.

**Optional fine-tuning**: if robot is unstable, offer `kp ±2` and `kd ±0.1`
adjustments with the same oscillation guard as Phase 3.

On user confirmation: finalize all values.

## Output

All measured data patched into `robot.yaml` in-place (comments preserved):
- Motor ID mapping (joint order)
- `default_q` per joint
- `control.kp`, `control.kd`

Script prints a summary of all written values and exits cleanly.

## Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | Subscribe to joint_states, imu, odometry topics |
| `ruamel.yaml` | Read/write robot.yaml preserving comments |
| `colorama` | Cross-platform colored terminal output |

`ruamel.yaml` and `colorama` may need `pip install` if not present on onboard computer.
