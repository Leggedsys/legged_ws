# Safety Architecture

Motor safety protection layers, ordered from real-time hardware to policy-level.

---

## 1. Motor Bus Level (`real/motor_bus_node.py`)

| Protection | Mechanism | Response |
|-----------|-----------|----------|
| **Data validation** | `data.correct == True` + `data.motor_id` matches expected | Skip invalid frame, don't publish |
| **Spike filter** | Position delta > ±1.0 rad between ticks | Rejected, hold last known-good position |
| **Graceful estop** | `/joint_commands` stops arriving | 0.5s hold last target → 2s kp linear fade to 0 (kd stays active) |
| **Temp > 80°C** | `data.temp` check every tick | `ERROR` log — immediate attention required |
| **Temp > 65°C** | `data.temp` check every tick | `WARN` log — monitor but continue |
| **Motor fault** | `data.merror != 0` (1=overheat, 2=overcurrent, 3=overvoltage, 4=encoder) | `ERROR` log with fault type |

## 2. Command Bridge Level (`real/motor_command_bridge.py`)

| Protection | Mechanism | Response |
|-----------|-----------|----------|
| **Hardware angle limits** | URDF→motor conversion, clipped to `q_min`/`q_max` from `robot.yaml` | Silently capped |
| **Joint speed limit** | Per-joint delta capped to `max_joint_speed * dt` (configurable via `robot.yaml control.max_joint_speed`, default 3.0 rad/s) | Silently clamped |

## 3. Teleop Level (`processing/teleop_node.py`)

| Protection | Mechanism | Response |
|-----------|-----------|----------|
| **E-stop (B button)** | Hold B button | Publishes `posture_command=false` (trigger lie-down → PASSIVE zero torque) + zero `Twist` |
| **Deadzone** | Joystick axes < `deadzone` (default 0.05) → zero output | Prevents stick drift activating policy |

## 4. Policy Level (`policy_node.py`)

| Protection | Mechanism | Response |
|-----------|-----------|----------|
| **State machine gate** | `PASSIVE` → must receive `posture_command=true` → `STANDUP` → `WAIT` → must receive non-zero `cmd_vel` → `POLICY` | Multi-step human-confirmed transition |
| **STANDUP ramp** | `robot.yaml standup.ramp_duration` seconds (default 8s) smooth-step from current to `q_default_urdf` | Smooth transition, no jump |
| **STANDUP completion check** | Joints must be within `_STANDUP_TOL` (0.05 rad) of `q_default_urdf` AND joint velocities settled | Won't enter `WAIT` until physically stable |
| **LIEDOWN ramp** | `robot.yaml standup.lie_down_duration` seconds (default 2s) smooth-step to initial passive position | Smooth descent |
| **LIEDOWN completion check** | Joints must be near initial passive position AND settled | Won't re-enter `PASSIVE` until stable |
| **Progress timeout** | STANDUP/LIEDOWN completion + 5s hard timeout → force next phase | Prevents infinite hang |
| **Soft angle limits** | `q_target` clipped to `policy.yaml joint_soft_limits` | Joints stay within training-safe range |
| **Obs validation gate** | All 49 obs groups checked against training 2σ ranges before `WAIT→POLICY` | Refuse POLICY entry, log 🚨 |
| **Obs anomaly watch** | Same checks every inference tick during `POLICY` (3s throttle) | `WARN` log of out-of-range values |
| **Peak action check** | `raw_action` logged every tick during `POLICY` | Human oversight via terminal |

## 5. Configuration (`config/robot.yaml`)

```yaml
control:
  kp: 3.8
  kd: 0.26
  max_joint_speed: 3.0    # rad/s (URDF frame) — per-joint cmd delta limit

teleop:
  btn_emergency_stop: 1    # B button
  deadzone: 0.05
```

## 6. Emergency Response Hierarchy

```
User presses B (e-stop)
  │
  ▼
teleop_node: publish posture_command=false + zero Twist
  │
  ▼
policy_node: transition POLICY/WAIT/STANDUP → LIEDOWN
  │          smooth ramp to initial passive position
  ▼
policy_node: LIEDOWN complete → PASSIVE
  │          broadcast kp=kd=0 to motor_bus_node
  ▼
motor_bus_node: motors at zero torque (kp=kd=0)
  DOG IS SAFE
```

```
Ctrl+C (kill launch)
  │
  ▼
motor_bus_node: /joint_commands stops arriving
  │ 0.5s hold last target
  ▼
motor_bus_node: 2s kp linear fade to 0 (kd stays for damping)
  DOG DESCENDS GENTLY
```

## 7. Log Severity Levels

| Level | Meaning | Action |
|-------|---------|--------|
| `INFO` | Normal operation (phase transitions, model loaded) | Monitor |
| `WARN` | Anomaly detected (high temp, obs out of range, stale joints) | Investigate after run |
| `ERROR` | Fault detected (motor overheating, motor fault code, obs gate blocked) | Stop and investigate immediately |
| `🚨` | Pre-POLICY obs gate blocked | Human must acknowledge before retrying |

## 8. Quick Reference

```bash
# Emergency stop (hardware)
Hold B button on gamepad

# Emergency stop (software)
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: false}"

# Kill everything (hardware)
Ctrl+C in terminal

# Check motor temps
ros2 topic echo /fr/thigh/joint_states  # effort field is toruqe, temp is in motor_bus logs
```
