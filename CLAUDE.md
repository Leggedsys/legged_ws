# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Humble workspace for a quadruped legged robot. Four packages:
- `legged_control` (ament_python) — main control stack
- `unitree_actuator_sdk` (ament_python + C extension) — motor SDK
- `odin_ros_driver` (ament_cmake, C++17) — IMU/VIO driver
- `dog_urdf` (git submodule) — URDF model

## Build & Setup

```bash
source /opt/ros/humble/setup.bash
colcon build                                     # all packages
colcon build --packages-select legged_control    # single package rebuild
source install/setup.bash                        # must re-source after every build
```

**Config changes require a rebuild.** `robot.yaml` is at `src/legged_control/config/robot.yaml` but nodes read from `install/share/legged_control/config/robot.yaml`. After editing, rebuild and re-source.

## Tests

Run test files individually — the `launch-pytest` plugin breaks batch glob collection:

```bash
source install/setup.bash
/usr/bin/python3 -m pytest src/legged_control/tests/test_kinematics.py
# Loop all tests:
for f in src/legged_control/tests/test_*.py; do /usr/bin/python3 -m pytest "$f"; done
```

**Working tests:** `test_kinematics.py`, `test_motor_bus_node.py`, `test_joint_aggregator.py`, `test_state_estimator.py`, `test_teleop_node.py`, `test_height_scan.py`, `test_obs_assembler.py`, `test_policy_node_guards.py`.

**Known broken tests (do not fix without checking):**
- `test_robot_launch.py` — imports `_leg_group`/`_parse_legs` which moved from `robot.launch.py` to `real.launch.py`
- `test_policy_node.py` — `_decode_action` signature changed; `_reorder_yaml_to_policy` was removed
- `test_gazebo_control_bridge.py` — intentionally skipped with `pytest.skip`

## Launch

| Command | Purpose |
|---------|---------|
| `ros2 launch legged_control robot.launch.py mode:=passive` | Hardware: passive mode (zero torque, read-only) |
| `ros2 launch legged_control robot.launch.py mode:=policy` | Hardware: full policy deployment |
| `ros2 launch legged_control robot.launch.py mode:=passive legs:=FR` | Single-leg passive (calibration) |
| `ros2 launch legged_control real.launch.py` | Hardware only (no processing/policy) |
| `ros2 launch legged_control test.launch.py` | Real + processing + viz, kp=kd=0 (data-link check) |
| `ros2 launch legged_control gazebo_sim.launch.py` | Full Gazebo simulation |

Key launch args: `dry_run:=true` (no serial port), `model_path:=/path/to/model.pt`, `serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1`, `legs:=FR,FL`.

> **README is outdated.** `mode:=stand`, `mode:=position_control`, and `position_control_sim.launch.py` do not exist. Valid modes are `passive` and `policy` only.

## Architecture & Data Flow

```
legged_control/legged_control/
├── real/           hardware layer
│   ├── motor_bus_node.py         RS485 ↔ /<ns>/joint_states (per port, 1 kHz)
│   ├── joint_aggregator.py       12 motor topics → /joint_states_aggregated (motor→URDF)
│   ├── motor_command_bridge.py   /joint_commands (URDF) → motor frame → /joint_commands_motor
│   └── urdf_joint_state_bridge.py → /joint_states (TF tree)
├── processing/     shared obs (real + sim)
│   ├── state_estimator_node.py   IMU + VIO → /state_estimate
│   ├── obs_assembler.py          4 topics → /observation (46-dim single frame)
│   └── teleop_node.py            /joy → /cmd_vel + /posture_command
├── test/           visualization / calibration tools
│   ├── monitor_node.py           dashboard + per-frame CSV logger (~/.legged_logs/)
│   └── leg_track_node.py         sine/chirp trajectory for PD gain characterization
├── kinematics.py   pure math: FK, IK, Jacobian, gravity projection
└── policy_node.py  TorchScript policy runner at 50 Hz
```

**Data flow:**
```
REAL:  motors → motor_bus → joint_aggregator (motor→URDF) → /joint_states_aggregated
SIM:   Gazebo → gazebo_control_bridge → /joint_states_aggregated

/joint_states_aggregated → state_estimator → /state_estimate
/camera/depth/* → height_scan → /height_scan
/joy → teleop → /cmd_vel

policy_node reads obs topics → /joint_commands (URDF frame)
  REAL: → motor_command_bridge (URDF→motor) → motor_bus
  SIM:  → gazebo_control_bridge → Gazebo
```

## Policy Node FSM

`PASSIVE` → (posture_command=true) → `STANDUP` → (ramp done + joint convergence) → `WAIT` → (non-zero cmd_vel + obs validation) → `POLICY` → (posture_command=false) → `LIEDOWN` → `PASSIVE`. `FAULT` is terminal (kp=0.5, kd=0.1).

All transitions happen inside `_tick()`. B-button sends `posture_command=false` + zero Twist for immediate lie-down from any state.

## Policy Input (B+C, 3-frame stack × 46 dims = 138 dims)

Single frame: `[0:3]` base_ang_vel×0.25, `[3:6]` projected_gravity, `[6:9]` velocity_commands×(2,2,0.25), `[9]` height_command, `[10:22]` joint_pos_rel, `[22:34]` joint_vel×0.05, `[34:46]` last_action.

## Key Quirks

- **Two serial ports:** FR/FL → `/dev/ttyUSB0` (front), RR/RL → `/dev/ttyUSB1` (rear). One `motor_bus_node` per port.
- **Gear ratio:** `motor_bus_node` divides readings and multiplies commands by `gear_ratio`. PD gains are rotor-side: joint stiffness K = kp × gear_ratio².
- **Motor IDs are non-sequential:** FR(0,1,2), FL(3,4,5), RR(6,7,**11**), RL(**9**,10,**8**). RR_calf=11, RL_hip=9, RL_calf=8.
- **Joint ordering:** YAML order is FR→FL→RR→RL (hip,thigh,calf). Policy order is FL→FR→RL→RR (paired). `_POLICY_TO_YAML` and `_reorder_policy_to_yaml()` in `policy_node.py` handle conversion.
- **URDF is a submodule.** Clone with `git clone --recursive`. Launch files skip `robot_state_publisher`/`rviz2` if `dog_urdf` not found.
- **Odin driver needs LD_PRELOAD.** Launch files automatically preload `libusb-1.0.so.0`.
- **Direction/zero_offset only in `real/`.** Policy and processing work in pure URDF frame. Frame conversion is done at the bus boundary only.
- **Single-layer joint limits:** `robot.yaml` q_min/q_max are the only angle clip, applied in `motor_command_bridge` (URDF frame, before motor-frame conversion). The `policy.yaml` soft-limit clip in `policy_node.py` was removed in `aa900e2`.

## Runtime Gain Tuning

```bash
ros2 param set /motor_bus_front kp 1.5
ros2 param set /motor_bus_front kd 0.07
ros2 param set /motor_bus_front kp_calf 1.0   # calf gains ~1/4 of hip/thigh (gr² ratio)
ros2 param set /motor_bus_rear  kp 1.5
# Per-joint override (higher priority):
ros2 param set /motor_bus_front kp_FR_hip 1.2
```

Changes take effect immediately. Update `robot.yaml` when satisfied; then rebuild.

## Safety Layers (outermost → innermost)

| Layer | Location | Key protection |
|-------|---------|---------------|
| Motor bus | `real/motor_bus_node.py` | temp/error monitoring, soft-stop on command loss |
| Command bridge | `real/motor_command_bridge.py` | q_min/q_max clip (only angle clip layer) |
| Teleop | `processing/teleop_node.py` | B-button e-stop, joystick deadzone |
| Policy FSM | `policy_node.py` | state machine gate, obs validation before POLICY entry, raw_action divergence → FAULT trip, standup/liedown ramps |

See `docs/safety.md` for full detail.

## Leg Track Test (PD characterization)

```bash
source install/setup.bash
ros2 run legged_control leg_track_node --ros-args -p leg:=FR -p mode:=chirp -p freq:=0.5 -p freq_end:=8.0 -p duration:=15.0
# Output: ~/.legged_logs/leg_track_<ts>/leg_track.csv + leg_track.png
```

Or use the wrapper script: `bash scripts/leg_track_test.sh`
