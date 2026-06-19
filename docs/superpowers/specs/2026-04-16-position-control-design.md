# Position Control (步态位控) — Design Spec

**Date**: 2026-04-16  
**Branch**: `dev/position-control`  
**Status**: Approved, ready for implementation

---

## 1. Overview

Add a `position_control` operating mode to `legged_control` that lets the robot stand up and trot using analytical inverse kinematics driven by `/cmd_vel`. This replaces the RL policy with a classical foot-trajectory + IK controller that can be deployed immediately without a trained model.

**Primary gait**: Trot (diagonal pairs FL+RR, FR+RL alternate).  
**Extensibility**: Gait scheduler is abstracted so Walk can be added later without restructuring.

---

## 2. New Files

```
legged_control/
  legged_control/
    kinematics.py          # Pure math: single-leg FK / IK, no ROS dependency
    gait_node.py           # ROS2 node: state machine + gait + safety
  tests/
    test_kinematics.py     # Unit tests: FK(IK(p)) ≈ p for all 4 legs, error < 0.1 mm
```

**Modified files**:
- `launch/robot.launch.py` — add `mode:=position_control`
- `setup.py` — register `gait_node` entry point
- `config/robot.yaml` — add `gait:` config block

---

## 3. Kinematics (`kinematics.py`)

### 3.1 URDF-derived constants

All four legs share the same link lengths:

| Parameter | Value | Source |
|-----------|-------|--------|
| `D_lat`   | 0.11266 m | hip link y (0.02046) + thigh link y (0.09220) |
| `l_hip_x` | 0.06451 m | hip link x offset (thigh joint forward of hip joint) |
| `l2`      | 0.18000 m | thigh sagittal length (z-component of thigh→calf) |
| `l3`      | 0.18126 m | calf length (z-component of calf→foot) |

### 3.2 Analytical IK — three steps

IK is computed in the **URDF frame** of each leg. The canonical convention follows FL (thigh/calf rotate around +Y, hip around +X). FR/RR use −Y for thigh/calf; RL/RR use −X for hip. Handled by mirroring foot coordinates before calling IK and negating outputs afterward.

**Step 1 — Hip abduction (q1)**

The y-projection of the foot onto the plane perpendicular to Y is always `D_lat`, independent of q2/q3. This decouples q1 from the sagittal IK:

```
q1 = atan2(z_foot, y_foot) - acos(D_lat / sqrt(y_foot² + z_foot²))
```

The `−acos` branch gives the leg-splayed-outward solution.

**Step 2 — Sagittal plane coordinates**

Remove q1 and the hip x-offset to get the 2D IK input:

```
x' = x_foot - l_hip_x
A  = y_foot · sin(q1) - z_foot · cos(q1)
```

**Step 3 — Thigh (q2) and calf (q3)**

Standard two-link planar IK:

```
cos(q3) = (x'² + A² − l2² − l3²) / (2 · l2 · l3)
q3 = −acos(cos_q3)                          # negative = knee bent downward
q2 = atan2(−x', A) + atan2(l3·sin(q3), l2 + l3·cos(q3))
```

IK returns `None` if `cos(q3)` is outside [−1, 1] (workspace violation).

### 3.3 URDF → motor frame conversion

After IK produces URDF-frame angles, `gait_node` applies per-joint conversion from `robot.yaml`:

```python
q_motor = direction * (q_urdf - zero_offset)
```

### 3.4 Unit tests

`test_kinematics.py` verifies for each leg and a grid of valid foot positions:

```
p_target → IK → (q1, q2, q3) → FK → p_recovered
assert |p_target - p_recovered| < 1e-4 m
```

---

## 4. Gait Algorithm

### 4.1 Phase oscillator

A global phase φ ∈ [0, 2π) increments at `gait_freq` Hz (50 Hz node loop).  
Per-leg phase offset:

| Leg | Offset |
|-----|--------|
| FL  | 0 |
| RR  | 0 |
| FR  | π |
| RL  | π |

φ_leg = (φ + offset) mod 2π.  
- Stance: φ_leg ∈ [0, π)  
- Swing: φ_leg ∈ [π, 2π)

### 4.2 Nominal foot positions (body frame)

Foot rests directly below the hip joint. In body frame:

| Leg | x_nom   | y_nom   | z_nom          | Derivation |
|-----|---------|---------|----------------|------------|
| FL  | +0.1426 | +0.1592 | −stance_height | +0.04649 + 0.11266 |
| FR  | +0.1426 | −0.1592 | −stance_height | −0.04651 − 0.11266 |
| RL  | −0.1426 | +0.1592 | −stance_height | +0.04651 + 0.11266 |
| RR  | −0.1426 | −0.1592 | −stance_height | −0.04649 − 0.11266 |

y_nom = hip_y_body ± D_lat  (+ for left legs, − for right legs).

### 4.3 Foot trajectory

**Stride parameters** (derived from `/cmd_vel` each cycle):
```
stride_x = clamp(cmd_vx / (2 · gait_freq), ±max_stride_x)
stride_y = clamp(cmd_vy / (2 · gait_freq), ±max_stride_y)
```

**Stance phase** — foot slides backward at ground speed:
```
s = φ_leg / π   ∈ [0, 1)
x_foot = x_nom + (0.5 − s) · stride_x
z_foot = −stance_height
```

**Swing phase** — sinusoidal lift:
```
t = (φ_leg − π) / π   ∈ [0, 1)
x_foot = x_nom + (t − 0.5) · stride_x
z_foot = −stance_height + step_height · sin(π · t)
```

y_foot is fixed at y_nom for v1. Yaw is approximated by rotating each leg's nominal position around body Z by `cmd_yaw · dt`.

### 4.4 YAML configuration (with runtime override via ROS2 params)

```yaml
gait:
  stance_height:  0.28    # m
  step_height:    0.06    # m
  gait_freq:      2.0     # Hz
  max_stride_x:   0.08    # m per half-cycle
  max_stride_y:   0.04    # m per half-cycle
  idle_timeout:   1.0     # s before cmd_vel considered stale
  safety_limit_window: 0.5   # s window for sustained-limit detection
  safety_limit_ratio:  0.8   # fraction of frames hitting limit → FAULT
```

---

## 5. `gait_node.py` — State Machine

```
STANDUP ──done──► WAIT ──joint_states received──► TROT
                                                     │
                              cmd_vel=0, phase done  │
                              ◄────────────────────-─┘
                              (hold default_q)

Any state ──sustained joint limit──► FAULT
```

### STANDUP
Reuses the same three-phase logic as `standup_node` (hold at 0 → smoothstep ramp to `default_q` → hold). Implemented inline — no separate node. Transitions to WAIT when ramp completes.

### WAIT
Publishes `default_q` at 50 Hz. Waits for `/joint_states_aggregated` to confirm motors are online. Transitions to TROT on first valid message.

### TROT
50 Hz loop:
1. Read `cmd_vel` (zero if stale > `idle_timeout`)
2. Advance φ
3. Compute foot targets for all 4 legs
4. IK → URDF angles → motor angles
5. Clamp to `[q_min, q_max]` (Layer 1 safety)
6. Publish `/joint_commands`

### FAULT
Broadcasts `kp=0` to both bus nodes (kd retained for damped fall). Stops publishing `/joint_commands`. Logs warning. Requires node restart to recover.

---

## 6. Safety — Three Layers

| Layer | Trigger | Action |
|-------|---------|--------|
| 1 | Any joint outside `[q_min, q_max]` | Clamp silently, continue |
| 2 | `/cmd_vel` stale > `idle_timeout` | Zero stride, hold `default_q` |
| 3 | Clamp rate > `safety_limit_ratio` over `safety_limit_window` | kp→0, kd retained, enter FAULT |

Layer 3 deliberately allows a slow damped fall rather than fighting an unsolvable IK in a loop.

---

## 7. Launch Integration

```bash
# New mode — full pipeline, all 12 joints
ros2 launch legged_control robot.launch.py mode:=position_control

# Tuning at runtime
ros2 param set /gait_node stance_height 0.25
ros2 param set /gait_node step_height 0.05
ros2 param set /gait_node gait_freq 1.5
```

`mode:=position_control` enforces `legs:=all` (same as policy mode).

Nodes started by launch:
- 2× `motor_bus_node` (front/rear)
- `joint_aggregator`
- `gait_node`
- `joy_node` + `teleop_node` (reused from policy mode)

---

## 8. Coordinate Conventions Summary

| Frame | Usage |
|-------|-------|
| Body frame | Gait planner; foot positions relative to `base_link` |
| Hip frame | IK solver input; foot position relative to each hip joint |
| URDF joint frame | IK output (q_urdf) |
| Motor frame | q_motor = direction × (q_urdf − zero_offset); sent to motor_bus_node |

Conversion body→hip: subtract hip joint position from `robot.yaml` URDF joint origins.

---

## 9. Out of Scope (v1)

- Full lateral / yaw decoupling (approximated via nominal foot rotation)
- Walk gait (interface reserved; not implemented)
- Foot contact sensing / adaptive stance
- Body height regulation feedback
