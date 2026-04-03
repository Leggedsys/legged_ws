# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 workspace for controlling a legged robot. Current packages:
- `odin_ros_driver` — C++ driver for Odin1 LiDAR (supports both ROS1/ROS2)
- `unitree_actuator_sdk` — Python ROS2 driver for Unitree GO-M8010-6 brushless motors (12 motors for a legged robot)

Planned additions: inference service, remote control service.

## Build Commands

```bash
# Source ROS2 environment first
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Build a single package
colcon build --packages-select unitree_actuator_sdk
colcon build --packages-select odin_ros_driver

# Source the workspace after build
source install/setup.bash

# Clean build using package scripts
./src/odin_ros_driver/script/build_ros2.sh -c
```

## Running Nodes

```bash
# Odin1 LiDAR (full stack: driver + processing nodes + RViz2)
ros2 launch odin_ros_driver odin1_ros2.launch.py

# Unitree motor driver (single motor, default /dev/ttyUSB0, motor ID 3)
ros2 run unitree_actuator_sdk go_m8010_6_node

# Motor driver with custom parameters
ros2 run unitree_actuator_sdk go_m8010_6_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p motor_id:=0 -p loop_hz:=1000.0
```

## Architecture

### Motor Driver (`unitree_actuator_sdk`)
- Build type: `ament_python` with a C++ extension compiled via `setup.py`
- At runtime, `sdk_loader.py` uses `ctypes` with `RTLD_GLOBAL` to load the platform-specific prebuilt SDK (`libUnitreeMotorSDK_Linux64.so` or `libUnitreeMotorSDK_Arm64.so`), then loads the compiled Python extension `_unitree_actuator_sdk`
- `go_m8010_6_node.py` runs a 1000 Hz control loop; publishes `sensor_msgs/JointState` on `joint_states`
- Each node instance controls one motor. To drive 12 motors, launch 12 node instances with different `motor_id` and `serial_port` parameters

### LiDAR Driver (`odin_ros_driver`)
- Build type: `ament_cmake`, C++17, links against prebuilt `liblydHostApi_amd.a` / `liblydHostApi_arm.a`
- `host_sdk_sample.cpp` is the main node; uses C API callbacks from the binary SDK for data streaming
- Configuration is centralized in `config/control_command.yaml` — the launch file loads this file as parameters
- Key operating modes controlled by `custom_map_mode`: `0`=Odometry only, `1`=SLAM (map building), `2`=Relocalization (load existing map)
- Separate `_ros.cpp`/`_ros2.cpp` source variants exist for ROS1/ROS2; the CMakeLists selects the correct one via preprocessor detection

### Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `joint_states` | `sensor_msgs/JointState` | Motor driver |
| `odin1/cloud_raw` | `sensor_msgs/PointCloud2` | LiDAR (raw, with reflectivity/confidence fields) |
| `odin1/cloud_slam` | `sensor_msgs/PointCloud2` | LiDAR (SLAM cloud, RGB-colored) |
| `odin1/imu` | `sensor_msgs/Imu` | LiDAR |
| `odin1/odometry` | `nav_msgs/Odometry` | LiDAR |
| `odin1/image` | `sensor_msgs/Image` | LiDAR RGB camera |
| `tf` | — | LiDAR (odom→base_link) |

## Hardware Setup

**Odin1 LiDAR (USB):**
```bash
# USB device access (vendor: 2207, product: 0019)
sudo vim /etc/udev/rules.d/99-odin-usb.rules
# SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
sudo udevadm control --reload && sudo udevadm trigger
```

**Unitree Motors (Serial):**
```bash
# Add user to dialout group for /dev/ttyUSB* access
sudo usermod -a -G dialout $USER
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
