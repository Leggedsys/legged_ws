# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 workspace for controlling a legged robot. Current packages:
- `odin_ros_driver` — C++ driver for Odin1 LiDAR (supports both ROS1/ROS2)
- `unitree_actuator_sdk` — Python ROS2 driver for Unitree GO-M8010-6 brushless motors (12 motors)
- `legged_control` — Robot control stack: initialization, standing pose, and (planned) policy execution

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

All robot modes share a single launch file. All 12 motors are always started.

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
| `mode` | `passive` | `passive`, `stand`, `policy` (not yet implemented) |
| `serial_port_front` | from `robot.yaml` | serial port for FR/FL motors |
| `serial_port_rear`  | from `robot.yaml` | serial port for RR/RL motors |

### Mode descriptions

**`passive`** — Motors output zero torque (kp=kd=0). Operator moves joints by hand. Terminal prints all 12 joint positions at 2 Hz, one line per leg:
```
[passive]  FR  hip=  0.312  thigh=  0.841  calf= -1.502
           FL  hip=  0.000  thigh=  0.823  calf= -1.499
           RR  hip= -0.015  thigh=  0.810  calf= -1.510
           RL  hip=  0.003  thigh=  0.834  calf= -1.495
```
Use this to determine `q_min`, `q_max`, and `default_q` values, then edit `config/robot.yaml`.

**`stand`** — Motors hold all joints at their `default_q` using PD gains from `robot.yaml`. Use this to tune `kp`/`kd`: edit `robot.yaml` and restart the launch.

**`policy`** — Not yet implemented. Will run the learned locomotion policy.

## Architecture

### `legged_control`
- Build type: `ament_python`
- Config: `config/robot.yaml` — joint limits, default angles, PD gains, serial ports
- `motor_bus_node.py` — owns one serial port exclusively; cycles all assigned joints sequentially at 1000 Hz via `sendRecv`; validates `data.correct` and `data.motor_id`; publishes `/<ns>/joint_states` per joint. Two instances: `/motor_bus_front` (FR/FL, `serial_port_front`) and `/motor_bus_rear` (RR/RL, `serial_port_rear`). Runtime-tunable `kp`/`kd` parameters.
- `passive_monitor_node.py` — subscribes to all 12 `/<ns>/joint_states`, prints positions at 2 Hz
- `stand_node.py` — publishes `default_q` for all 12 joints at 50 Hz on `/joint_commands`; broadcasts kp/kd updates to both bus nodes via `AsyncParametersClient`
- `launch/robot.launch.py` — single entry point, starts 2 `motor_bus_node` instances (one per serial port) + mode-specific node

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
| `/joint_commands` | `sensor_msgs/JointState` | `stand_node` → `motor_bus_node` instances |
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
  - name: FR_hip
    motor_id: 0
    default_q: 0.0    # standing pose angle (rad)
    q_min: -1.047     # -60 deg
    q_max:  1.047     #  60 deg
  # ... 11 more joints

control:
  serial_port_front: /dev/ttyUSB0   # FR, FL legs
  serial_port_rear:  /dev/ttyUSB1   # RR, RL legs
  motor_hz: 1000.0
  kp: 20.0            # position gain
  kd: 0.5             # velocity damping gain
```

Edit this file to update joint limits, default angles, or PD gains. Restart the launch after any change.

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
