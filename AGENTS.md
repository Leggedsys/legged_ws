# AGENTS.md

ROS2 workspace for a legged robot. Three packages: `legged_control` (ament_python), `unitree_actuator_sdk` (ament_python + C extension), `odin_ros_driver` (ament_cmake, C++17).

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
│   ├── gazebo_control_bridge.py  Gazebo joints → /joint_states_aggregated (URDF, YAML order)
│   └── policy_node_sim.py        (deprecated, being removed)
├── processing/     shared obs computation (real + sim)
│   ├── state_estimator_node.py   IMU + VIO odom → /state_estimate
│   ├── height_scan_node.py       depth → /height_scan (325 floats)
│   ├── teleop_node.py            /joy → /cmd_vel + /posture_command
│   └── obs_assembler.py          4 topics → /observation (373 floats)
├── test/           visualization
│   ├── obs_monitor_node.py       terminal 2Hz obs display
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

## Launch files

| File | What it starts |
|------|---------------|
| `real.launch.py` | Hardware only: motors, joint_agg, motor_bridge, odin, realsense, joy, urdf_bridge, odin TF |
| `test.launch.py` | real + processing + viz (kp=kd=0). Use for data-link verification. |
| `robot.launch.py` | Main entry. `mode:=passive` or `mode:=policy`. Includes real + processing + viz + policy. |
| `gazebo_physics.launch.py` | Simulation with physics |
| `gazebo_sim.launch.py` | Full sim stack: physics + control_bridge + processing + policy + viz |

## Key quirks

- **Two serial ports, two bus nodes.** FR/FL share `/dev/ttyUSB0` (front), RR/RL share `/dev/ttyUSB1` (rear). `motor_bus_node` per port.
- **Graceful estop.** `/joint_commands` stops → 0.5s hold → 2s kp→0 (kd stays for damping).
- **Gear ratio.** `motor_bus_node` divides readings, multiplies commands by `gear_ratio`.
- **Direction/zero_offset only in `real/`.** `joint_aggregator` does motor→URDF. `motor_command_bridge` inverts URDF→motor. `policy_node` works in pure URDF frame.
- **URDF is optional.** Launch files skip `robot_state_publisher`/`rviz2` if `dog_urdf` package not found. `src/dog_urdf` is a git submodule — `git clone --recursive` to pull it.
- **Motor IDs** per-joint in `robot.yaml`. Bus node validates `data.correct` + `data.motor_id`.
- **No test framework, no CI.** Tests run via `python3 -m pytest src/legged_control/tests/test_*.py`.

## Runtime gain tuning

```bash
ros2 param set /motor_bus_front kp 5.0
ros2 param set /motor_bus_front kd 0.3
```

Changes take effect immediately. Update `robot.yaml` when satisfied.

## Safety

| Protection | Where |
|-----------|-------|
| E-stop (B button) | teleop_node → posture_command=false + zero Twist |
| Joint speed limit | motor_command_bridge (max_joint_speed in robot.yaml) |
| Hardware angle clip | motor_command_bridge (q_min/q_max in robot.yaml) |
| Soft angle limits | policy_node (policy.yaml soft limits) |
| Obs validation gate | policy_node (before WAIT→POLICY) |
| Spike filter | motor_bus_node (>±1.0 rad/tick rejected) |
| Motor temp/error | motor_bus_node (data.temp, data.merror) |
