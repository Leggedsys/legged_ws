# AGENTS.md

ROS2 workspace for a legged robot. Four packages: `legged_control` (ament_python), `unitree_actuator_sdk` (ament_python + C extension), `odin_ros_driver` (ament_cmake, C++17), `dog_urdf` (URDF submodule).

## Setup

```bash
git clone --recursive                                 # dog_urdf is a submodule
bash scripts/install_deps.sh                          # apt packages + udev rules + dialout group
sudo usermod -a -G dialout $USER && newgrp dialout    # serial port access
source /opt/ros/humble/setup.bash
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build                                          # all packages
colcon build --packages-select legged_control          # single package
source install/setup.bash                              # must re-source after build
```

**Config changes need a rebuild.** `robot.yaml` lives in `src/legged_control/config/` but gets copied to `install/share/legged_control/config/` via `setup.py data_files`. Nodes read from the install location at runtime. After editing `robot.yaml`, run `colcon build --packages-select legged_control` and re-source.

## Architecture

```
legged_control/legged_control/
├── real/           hardware data sources
│   ├── motor_bus_node.py         RS485 → /<ns>/joint_states
│   ├── joint_aggregator.py       12 topics → /joint_states_aggregated (motor→URDF)
│   ├── motor_command_bridge.py   /joint_commands (URDF) → motor frame → /joint_commands_motor
│   └── urdf_joint_state_bridge.py → /joint_states (TF tree)
├── sim/            simulation data sources
│   └── gazebo_control_bridge.py  Gazebo joints → /joint_states_aggregated (URDF, YAML order)
├── processing/     shared obs computation (real + sim)
│   ├── state_estimator_node.py   IMU + VIO odom → /state_estimate
│   ├── height_scan_node.py       depth → /height_scan (325 floats)
│   ├── teleop_node.py            /joy → /cmd_vel + /posture_command
│   └── obs_assembler.py          4 topics → /observation (373 floats)
├── test/           visualization
│   ├── obs_monitor_node.py       terminal 2Hz obs display
│   ├── monitor_node.py           dashboard + per-frame CSV logger (~/.legged_logs/)
│   └── vel_viz_node.py           velocity arrows MarkerArray
├── kinematics.py   pure math: FK, IK, Jacobian, gravity projection
└── policy_node.py  policy: reads 4 obs topics → /joint_commands (URDF frame)
```

### Data flow

```
REAL:  motors → motor_bus → joint_agg (motor→URDF) → /joint_states_aggregated
SIM:   Gazebo → control_bridge → /joint_states_aggregated

/joint_states_aggregated → state_estimator → /state_estimate
/camera/depth/* → height_scan → /height_scan
/joy → teleop → /cmd_vel

/policy_node reads 4 obs topics → /joint_commands (URDF frame)
  REAL:  /joint_commands → motor_command_bridge (URDF→motor) → /joint_commands_motor → motor_bus
  SIM:   /joint_commands → control_bridge → Gazebo controller
```

### Policy node FSM

`PASSIVE` → (posture_command=true) → `STANDUP` → (ramp done) → `WAIT` → (cmd_vel + validation) → `POLICY` → (posture_command=false) → `LIEDOWN` → `PASSIVE`. `FAULT` is a terminal safe state (kp=0.5, kd=0.1).

The state machine is implemented in `policy_node.py` (`_PHASE_*` constants, `_tick()` dispatcher). Transitions only happen inside `_tick()` — posture_command changes are latched until the next timer tick.

## Launch files

| File | What it starts |
|------|---------------|
| `real.launch.py` | Hardware only: motors, joint_agg, motor_bridge, odin, realsense, joy, urdf_bridge, odin TF |
| `test.launch.py` | real + processing + viz (kp=kd=0). Use for data-link verification. |
| `robot.launch.py` | Main entry. `mode:=passive` or `mode:=policy`. Includes real + processing + viz + policy. |
| `gazebo_physics.launch.py` | Simulation with physics |
| `gazebo_sim.launch.py` | Full sim stack: physics + control_bridge + processing + policy + viz |

Useful launch args:
- `legs:=FR,FL` — start only specific legs (passive/stand testing)
- `dry_run:=true` — motor_bus_node runs without serial port (offline testing)
- `model_path:=/path/to/model.pt` — override policy model
- `serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1` — override serial ports

## Key quirks

- **Two serial ports, two bus nodes.** FR/FL share `/dev/ttyUSB0` (front), RR/RL share `/dev/ttyUSB1` (rear). `motor_bus_node` per port.
- **Graceful estop.** `/joint_commands` stops → 0.5s hold → 2s kp→0 (kd stays for damping).
- **Gear ratio.** `motor_bus_node` divides readings, multiplies commands by `gear_ratio`.
- **Direction/zero_offset only in `real/`.** `joint_aggregator` does motor→URDF. `motor_command_bridge` inverts URDF→motor. `policy_node` works in pure URDF frame.
- **URDF is optional.** Launch files skip `robot_state_publisher`/`rviz2` if `dog_urdf` package not found. `src/dog_urdf` is a git submodule — use `git clone --recursive`.
- **Motor IDs** per-joint in `robot.yaml`. Bus node validates `data.correct` + `data.motor_id`.
- **Odin driver needs LD_PRELOAD.** Launch files automatically preload `libusb-1.0.so.0` for Odin's `host_sdk_sample`.
- **Joint ordering is not sequential in robot.yaml.** motor_id order: FR(0,1,2), FL(3,4,5), RR(6,7,11), RL(9,10,8). RR_calf=11, RL_calf=8.
- **Policy ordering differs from YAML ordering.** YAML order is FR→FL→RR→RL per-leg. Policy order is FL→FR paired (hip, thigh, calf pairs between legs). `_POLICY_TO_YAML` and `_reorder_policy_to_yaml()` handle conversion.
- **README references deprecated modes.** The README mentions `mode:=stand` and `mode:=position_control` which do not exist in `robot.launch.py` (only `passive` / `policy`). `position_control_sim.launch.py` also does not exist. Do not attempt to use these modes.

## Motors, joints, kinematics

- **URDF joint names**: `FR_hip_joint`, `FR_thigh_joint`, `FR_calf_joint`, etc.
- **kinematics.py API** (`legged_control/legged_control/kinematics.py`):
  - `forward_kinematics(leg, joints)` → foot pos in hip frame
  - `inverse_kinematics(leg, foot_pos, preferred_joints=None)` → angles or None
  - `leg_kinematic_velocity(leg, q_urdf, dq_urdf)` → foot linear velocity
  - `projected_gravity_from_quat(qx, qy, qz, qw)` → gravity vector in body frame
  - `yaw_rotation_matrix(qx, qy, qz, qw)` → 2D yaw rotation matrix
  - `_smoothstep(x)` — exported smoothing function (also used by policy_node)
  - Leg constants: `LEFT_LEGS`, `RIGHT_LEGS`, `FRONT_LEGS`, `REAR_LEGS`

## Tests

Run individually after sourcing the workspace (the ROS2 `launch-pytest` plugin breaks batch glob collection):

```bash
source install/setup.bash
/usr/bin/python3 -m pytest src/legged_control/tests/test_kinematics.py
/usr/bin/python3 -m pytest src/legged_control/tests/test_motor_bus_node.py
# ... etc (see src/legged_control/tests/ for all test files)
```

Or loop: `for f in src/legged_control/tests/test_*.py; do /usr/bin/python3 -m pytest "$f"; done`

**Known: 2 test files are out of sync** with the implementation:
- `test_robot_launch.py` — imports `_leg_group`/`_parse_legs` from `robot.launch.py`, but they moved to `real.launch.py`
- `test_policy_node.py` — `_decode_action` signature changed (added `soft_q_max_urdf`), `_reorder_yaml_to_policy` was removed
- `test_gazebo_control_bridge.py` — skipped by design (`pytest.skip` at module level)

**Working tests** (safe to run): `test_kinematics.py`, `test_motor_bus_node.py`, `test_joint_aggregator.py`, `test_state_estimator.py`, `test_teleop_node.py`, `test_height_scan.py`.

## Runtime gain tuning

```bash
# Global kp/kd (all motors on a bus)
ros2 param set /motor_bus_front kp 5.0
ros2 param set /motor_bus_front kd 0.3
ros2 param set /motor_bus_rear  kp 5.0

# Per-joint override (higher priority than global)
ros2 param set /motor_bus_front kp_FR_hip 4.0
ros2 param set /motor_bus_front kd_FR_hip 0.2
```

Changes take effect immediately. Update `robot.yaml` when satisfied.

## Safety

| Protection | Where |
|-----------|-------|
| E-stop (B button) | teleop_node → posture_command=false + zero Twist |
| Hardware angle clip (only angle clip layer) | motor_command_bridge (q_min/q_max in robot.yaml) |
| Obs validation gate | policy_node (before WAIT→POLICY) |
| raw_action divergence → FAULT trip | policy_node |
| Motor temp/error | motor_bus_node (data.temp, data.merror) |

See `docs/safety.md` for the full safety architecture and e-stop response flow.
