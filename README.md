# Legged Robot Workspace

ROS2 (Humble) workspace for a 12-DOF quadruped robot with Unitree GO-M8010-6 motors.

## Packages

| Package | Language | Description |
|---------|----------|-------------|
| `legged_control` | Python | Robot control: initialization, standing pose, policy execution (planned) |
| `unitree_actuator_sdk` | Python + C ext | Low-level motor driver for Unitree GO-M8010-6 |
| `odin_ros_driver` | C++ | Driver for Odin1 LiDAR |

## Quick Start

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Build
colcon build

# 3. Source workspace
source install/setup.bash

# 4. Connect motors via /dev/ttyUSB0, then launch
ros2 launch legged_control robot.launch.py
```

## Robot Modes

Single launch file, `mode` argument selects behavior. All 12 motors always start.

### `passive` (default) — Find joint limits and default positions

```bash
ros2 launch legged_control robot.launch.py
```

Motors output zero torque. Move joints by hand. Terminal shows all 12 positions at 2 Hz:

```
[passive]  FR  hip=  0.000  thigh=  0.800  calf= -1.500
           FL  hip=  0.000  thigh=  0.800  calf= -1.500
           RR  hip=  0.000  thigh=  0.800  calf= -1.500
           RL  hip=  0.000  thigh=  0.800  calf= -1.500
```

Record the values you want and update `src/legged_control/config/robot.yaml`.

### `stand` — Tune PD gains in standing pose

```bash
ros2 launch legged_control robot.launch.py mode:=stand
```

Motors hold all joints at `default_q` with PD control. Edit `kp`/`kd` in `robot.yaml` and restart to apply.

### `policy` — Run locomotion policy *(not yet implemented)*

```bash
ros2 launch legged_control robot.launch.py mode:=policy
```

### Serial port override

```bash
ros2 launch legged_control robot.launch.py serial_port:=/dev/ttyUSB1
```

## Configuration

All robot parameters live in `src/legged_control/config/robot.yaml`. Rebuild after editing:

```bash
colcon build --packages-select legged_control && source install/setup.bash
```

Key fields:

```yaml
joints:
  - name: FR_hip       # FR/FL/RR/RL × hip/thigh/calf
    motor_id: 0        # 0–11
    default_q:  0.0    # standing pose angle (rad)
    q_min: -1.047      # joint limit (rad)
    q_max:  1.047

control:
  serial_port: /dev/ttyUSB0
  motor_hz: 1000.0     # motor control loop rate
  kp: 20.0             # position gain
  kd: 0.5              # velocity damping
```

### Joint layout

```
motor_id:   0        1        2        3        4        5
joint:    FR_hip  FR_thigh FR_calf  FL_hip  FL_thigh FL_calf

motor_id:   6        7        8        9       10       11
joint:    RR_hip  RR_thigh RR_calf  RL_hip  RL_thigh RL_calf
```

## Hardware Setup

**Motors — serial port access:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for the group to take effect
```

**LiDAR — USB access:**
```bash
sudo tee /etc/udev/rules.d/99-odin-usb.rules <<EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload && sudo udevadm trigger
```

## System Dependencies

```bash
# System libraries (for odin_ros_driver)
sudo apt install libopencv-dev libeigen3-dev libpcl-dev \
  libyaml-cpp-dev libssl-dev libcurl4-openssl-dev libusb-1.0-0-dev

# ROS2 packages
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs \
  ros-humble-geometry-msgs ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-pcl-conversions ros-humble-tf2-ros
```

## LiDAR

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

Operating mode set via `custom_map_mode` in `config/control_command.yaml`:
- `0` — Odometry only
- `1` — SLAM (map building)
- `2` — Relocalization (load existing map)
