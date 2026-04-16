# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 workspace for controlling a legged robot. Current packages:
- `odin_ros_driver` — C++ driver for Odin1 LiDAR (supports both ROS1/ROS2)
- `unitree_actuator_sdk` — Python ROS2 driver for Unitree GO-M8010-6 brushless motors (12 motors)
- `legged_control` — Robot control stack: initialization, standing pose, and policy execution

## Build Commands

```bash
# Source ROS2 environment first
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Build a single package
colcon build --packages-select legged_control
colcon build --packages-select unitree_actuator_sdk
colcon build --packages-select odin_ros_driver

# Source the workspace after build
source install/setup.bash

# Clean build for odin driver
./src/odin_ros_driver/script/build_ros2.sh -c
```

## Running the Robot (`legged_control`)

All robot modes share a single launch file. By default all 12 motors are started; use `legs` to activate a subset.

```bash
# Mode 1 — passive: zero torque, read positions by hand (initialization)
ros2 launch legged_control robot.launch.py

# Mode 2 — stand: hold default_q with PD gains (PID tuning)
ros2 launch legged_control robot.launch.py mode:=stand

# Override serial ports (default comes from config/robot.yaml)
ros2 launch legged_control robot.launch.py serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1
```

### Launch parameters

| Parameter | Default | Options |
|-----------|---------|---------|
| `mode` | `passive` | `passive`, `stand`, `standup`, `policy` |
| `legs` | `all` | `all`, `FR`, `FL`, `RR`, `RL`, or comma-separated e.g. `FR,FL` |
| `serial_port_front` | from `robot.yaml` | serial port for FR/FL motors |
| `serial_port_rear`  | from `robot.yaml` | serial port for RR/RL motors |

Note: `policy` mode enforces `legs=all` (the policy model requires a fixed 12-joint contract).

### Mode descriptions

**`passive`** — Motors output zero torque (kp=kd=0). Operator moves joints by hand. Terminal prints all 12 joint positions at 2 Hz, one line per leg:
```
[passive]  FR  hip=  0.312  thigh=  0.841  calf= -1.502
           FL  hip=  0.000  thigh=  0.823  calf= -1.499
           RR  hip= -0.015  thigh=  0.810  calf= -1.510
           RL  hip=  0.003  thigh=  0.834  calf= -1.495
```
Use this to determine `q_min`, `q_max`, and `default_q` values, then edit `config/robot.yaml`.

**`standup`** — Three-phase stand-up sequence: hold at 0 for 2 s (motors settle) → smooth-step ramp from 0 to `default_q` over 8 s → hold at `default_q`. Runtime-tunable: `ros2 param set /standup_node ramp_duration 10.0`. Also starts `passive_monitor_node` for live position feedback.

**`stand`** — Motors hold all joints at their `default_q` using PD gains from `robot.yaml`. Also starts `passive_monitor_node` so the 4-line position display remains active. `stand_node` supports runtime gain tuning: changing `kp`/`kd` in `robot.yaml` and restarting is the typical workflow, but gains can also be updated live via `AsyncParametersClient` without relaunch.

**`policy`** — Three-phase startup then locomotion policy at 50 Hz. **Phase 1 STANDUP**: ramps all joints from 0 (power-on position) to `default_q` over `standup.ramp_duration` seconds using a smooth S-curve (no sensor requirement). **Phase 2 WAIT**: holds `default_q` until IMU, odometry, and joint states are all fresh. **Phase 3 RUN**: 50 Hz inference loop — builds 45-dim observation → TorchScript model → publishes `/joint_commands`. Requires Odin1 LiDAR online (`/odin1/imu` and `/odin1/odometry`); publishing stops if any sensor is silent >0.5 s. Velocity commands come from `/cmd_vel` via `teleop_node` (gamepad via `joy_node`); falls back to zero if no gamepad. Also starts `policy_monitor_node`. Set `policy.model_path` and `standup.*` in `robot.yaml` before launching.

## Architecture

### `legged_control`
- Build type: `ament_python`
- Config: `config/robot.yaml` — joint limits, default angles, PD gains, serial ports
- `motor_bus_node.py` — owns one serial port exclusively; cycles all assigned joints sequentially at 1000 Hz via `sendRecv`; validates `data.correct` and `data.motor_id`; publishes `/<ns>/joint_states` per joint. Two instances: `/motor_bus_front` (FR/FL, `serial_port_front`) and `/motor_bus_rear` (RR/RL, `serial_port_rear`). Runtime-tunable `kp`/`kd` parameters.
- `passive_monitor_node.py` — subscribes to all 12 `/<ns>/joint_states`, prints positions at 2 Hz
- `stand_node.py` — publishes `default_q` for all 12 joints at 50 Hz on `/joint_commands`; broadcasts kp/kd updates to both bus nodes via `AsyncParametersClient`
- `joint_aggregator.py` — subscribes to all 12 `/<ns>/joint_states`, publishes `/joint_states_aggregated` in canonical joint order (motor frame, no coordinate conversion); triggers on every incoming message
- `policy_node.py` — 50 Hz inference loop; converts motor→URDF frame, builds 45-dim obs, runs TorchScript model, decodes 12-dim action to motor-frame `/joint_commands`; stops if IMU, odometry, or joint states silent >0.5 s
- `standup_node.py` — three-phase stand-up: hold at 0 (`hold_duration` s) → smooth-step ramp to `default_q` (`ramp_duration` s) → hold; logs a progress bar during ramp; same runtime kp/kd broadcast as `stand_node`.
- `teleop_node.py` — subscribes `/joy` (gamepad via `joy_node`); applies deadzone + scaling from `robot.yaml teleop` section; publishes `geometry_msgs/Twist` to `/cmd_vel` for `policy_node`. Auto-started in `policy` mode alongside `joy_node`. Run standalone: `ros2 run legged_control teleop_node`.
- `policy_monitor_node.py` — read-only terminal dashboard for `policy` mode; subscribes to `/odin1/imu`, `/odin1/odometry`, `/cmd_vel`, `/joint_states_aggregated`, `/joint_commands`; refreshes at 2 Hz showing sensor freshness labels (OK/STALE/--), command velocities, IMU angular velocity, gravity vector, per-leg mean joint deviation from default, and count of joints hitting limits.
- `launch/robot.launch.py` — single entry point, starts 2 `motor_bus_node` instances (one per serial port) + mode-specific nodes

### `unitree_actuator_sdk`
- Build type: `ament_python` with a C++ extension compiled via `setup.py`
- At runtime, `sdk_loader.py` uses `ctypes` with `RTLD_GLOBAL` to load the platform-specific prebuilt SDK (`libUnitreeMotorSDK_Linux64.so` or `libUnitreeMotorSDK_Arm64.so`), then loads the compiled Python extension `_unitree_actuator_sdk`
- `go_m8010_6_node.py` — legacy single-motor node (1000 Hz loop, one motor per instance); kept in package but no longer used in `robot.launch.py` (replaced by `motor_bus_node`)

### `odin_ros_driver`
- Build type: `ament_cmake`, C++17, links against prebuilt `liblydHostApi_amd.a` / `liblydHostApi_arm.a`
- `host_sdk_sample.cpp` is the main node; uses C API callbacks from the binary SDK for data streaming
- Configuration is centralized in `config/control_command.yaml` — the launch file loads this file as parameters
- Key operating modes controlled by `custom_map_mode`: `0`=Odometry only, `1`=SLAM (map building), `2`=Relocalization (load existing map)
- Separate `_ros.cpp`/`_ros2.cpp` source variants exist for ROS1/ROS2; the CMakeLists selects the correct one via preprocessor detection

```bash
# Odin1 LiDAR (full stack: driver + processing nodes + RViz2)
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/<ns>/joint_states` | `sensor_msgs/JointState` | `motor_bus_node` (e.g. `/fr/hip/joint_states`) |
| `/joint_commands` | `sensor_msgs/JointState` | `stand_node` (stand mode) or `policy_node` (policy mode) → `motor_bus_node` instances |
| `/joint_states_aggregated` | `sensor_msgs/JointState` | `joint_aggregator` → `policy_node` |
| `/joy` | `sensor_msgs/Joy` | `joy_node` (gamepad driver) |
| `/cmd_vel` | `geometry_msgs/Twist` | `teleop_node` → `policy_node` |
| `odin1/cloud_raw` | `sensor_msgs/PointCloud2` | LiDAR (raw, with reflectivity/confidence fields) |
| `odin1/cloud_slam` | `sensor_msgs/PointCloud2` | LiDAR (SLAM cloud, RGB-colored) |
| `odin1/imu` | `sensor_msgs/Imu` | LiDAR |
| `odin1/odometry` | `nav_msgs/Odometry` | LiDAR |
| `odin1/image` | `sensor_msgs/Image` | LiDAR RGB camera |
| `tf` | — | LiDAR (odom→base_link) |

### Motor Namespace Map

```
FR_hip → fr/hip    FR_thigh → fr/thigh    FR_calf → fr/calf
FL_hip → fl/hip    FL_thigh → fl/thigh    FL_calf → fl/calf
RR_hip → rr/hip    RR_thigh → rr/thigh    RR_calf → rr/calf
RL_hip → rl/hip    RL_thigh → rl/thigh    RL_calf → rl/calf
```

## Configuration (`config/robot.yaml`)

```yaml
joints:
  # Canonical order for observation/action vectors (index 0-11)
  - name: FR_hip
    motor_id: 0
    gear_ratio: 6.33          # total reduction ratio (motor_bus_node divides readings, multiplies commands)
    default_q: -0.14          # standing pose angle in motor frame (rad)
    q_min: -0.484
    q_max:  0.521
    direction:    1           # +1 if motor direction == URDF direction, -1 if reversed
    zero_offset:  0.0         # static offset in URDF frame (rad); q_urdf = direction * q_motor + zero_offset
  # ... 11 more joints (FL_hip/thigh/calf, RR_hip/thigh/calf, RL_hip/thigh/calf, FR_thigh/calf)

control:
  serial_port_front: /dev/ttyUSB0   # FR, FL legs
  serial_port_rear:  /dev/ttyUSB1   # RR, RL legs
  motor_hz: 1000.0
  policy_hz: 50.0             # policy inference rate (Hz)
  kp: 1.5                     # position gain (used in stand and policy modes)
  kd: 0.2                     # velocity damping gain
  # Policy action is a delta from default_q, scaled per joint group:
  #   target_q[i] = default_q[i] + action_scale[leg_group][joint_type] * action[i]
  action_scale:
    front:                    # FR/FL legs
      hip:   0.025
      thigh: 0.035
      calf:  0.02
    rear:                     # RR/RL legs
      hip:   0.03
      thigh: 0.045
      calf:  0.025

teleop:
  max_vx:  1.0                # m/s  forward/backward
  max_vy:  0.5                # m/s  lateral
  max_yaw: 1.0                # rad/s yaw rate
  deadzone: 0.05              # joystick axis deadzone (fraction of full range)
  axis_vx:  1                 # left stick vertical   (inverted)
  axis_vy:  0                 # left stick horizontal
  axis_yaw: 3                 # right stick horizontal (inverted)
  invert_vx:  true
  invert_vy:  false
  invert_yaw: true
  btn_emergency_stop: -1      # gamepad button index; -1 = not configured

policy:
  model_path: ''              # path to TorchScript model (.pt); required for policy mode
  obs_mean: []                # optional per-dim observation normalization mean (45 values)
  obs_std:  []                # optional per-dim observation normalization std  (45 values)
```

Edit this file to update joint limits, default angles, PD gains, or policy settings. Restart the launch after any change.

## Hardware Setup

**Unitree Motors (Serial):**
```bash
# Add user to dialout group for /dev/ttyUSB* access
sudo usermod -a -G dialout $USER
```

**Odin1 LiDAR (USB):**
```bash
# USB device access (vendor: 2207, product: 0019)
sudo vim /etc/udev/rules.d/99-odin-usb.rules
# SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
sudo udevadm control --reload && sudo udevadm trigger
```

## LiDAR Configuration Notes

- `config/control_command.yaml` controls all LiDAR features; `calib.yaml` is auto-fetched from the device
- `cloud_raw_confidence_threshold`: recommended 30–35 (higher = fewer but more reliable points)
- `dtof_fps`: `100` = 10 fps, `145` = 14.5 fps
- `senddepth` and `sendreprojection` are CPU-intensive; disabled by default
- SLAM mode requires USB 3.0 for map file transfer; set `strict_usb3.0_check: 1` to enforce

## Dependencies

System: `libopencv-dev` (≥4.5), `libeigen3-dev`, `libpcl-dev`, `libyaml-cpp-dev`, `libssl-dev`, `libcurl4-openssl-dev`, `libusb-1.0-0-dev`

ROS2 (Humble): `rclcpp`, `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `cv_bridge`, `image_transport`, `pcl_conversions`, `tf2_ros`
