# unitree_actuator_sdk

This package has been trimmed to a ROS 2 Python node only.

## What remains

- A ROS 2 node that wraps the GO-M8010-6 Python example
- The prebuilt Unitree shared libraries in `unitree_motor_ros2/native/`
- The prebuilt Python extension in `unitree_motor_ros2/native/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so`

## Run

```bash
source /opt/ros/humble/setup.bash
cd /home/shijue/legged_ws
colcon build --packages-select unitree_actuator_sdk
source install/setup.bash
ros2 run unitree_actuator_sdk go_m8010_6_node
```

## Parameters

- `serial_port` default: `/dev/ttyUSB0`
- `motor_id` default: `0`
- `loop_hz` default: `500.0`
- `target_dq` default: `6.28`
- `target_q` default: `0.0`
- `kp` default: `0.0`
- `kd` default: `0.01`
- `tau` default: `0.0`

## Notes

- The Python extension is rebuilt during `colcon build`, so it matches the active ROS 2 Python runtime.
- Access to `/dev/ttyUSB0` usually requires `sudo` or membership in the `dialout` group.
