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

Three layers, all in `legged_control`:

| Layer | Dir | Purpose |
|-------|-----|---------|
| `real/` | `legged_control/real/` | Hardware: motor_bus_node, joint_aggregator |
| `real2sim/` | `legged_control/real2sim/` | Processing: state estimator, height scan, teleop, URDF bridge |
| policy | `legged_control/policy_node.py` | Learned policy execution |
| test | `legged_control/test/` | Visualization: velocity arrows, obs monitor |

## Launch files

| File | What it starts |
|------|---------------|
| `real.launch.py` | Hardware only: motors (kp/kd active), IMU, camera, gamepad |
| `test.launch.py` | Full hardware + processing stack with RViz, motors in passive (kp=kd=0). Use for data-link verification. |
| `robot.launch.py` | Main entry. `mode:=passive` (zero torque + viz) or `mode:=policy` (kp/kd active + policy node). |
| `gazebo_physics.launch.py` | Simulation with physics |
| `gazebo_policy.launch.py` | Simulation with policy |

## Key quirks

- **Two serial ports, two bus nodes.** FR/FL share `/dev/ttyUSB0` (front), RR/RL share `/dev/ttyUSB1` (rear). One `motor_bus_node` per port cycles all assigned motors sequentially.
- **Graceful estop.** When `/joint_commands` stops arriving, `motor_bus_node` holds the last target for 0.5s then fades kp to 0 over 2s (kd stays active for damping). Robot settles gently on shutdown.
- **Gear ratio.** `motor_bus_node` divides position readings by `gear_ratio` and multiplies commands — `robot.yaml` joint angles are in output-shaft radians.
- **Direction/zero_offset.** `q_urdf = direction * q_motor + zero_offset`. The `urdf_joint_state_bridge` node handles this conversion.
- **URDF is optional.** `robot.launch.py` and `test.launch.py` gracefully skip `robot_state_publisher` and `rviz2` if `dog_urdf` package not found.
- **Motor IDs** are configured per-joint in `robot.yaml`. The bus node validates `data.correct` and `data.motor_id` on every read.
- **`legged_control` entries** in `setup.py` are the canonical list of installed execs. No invisible entry points.
- **No lint/typecheck/test config** exists. No CI.

## Runtime gain tuning

```bash
ros2 param set /motor_bus_front kp 5.0
ros2 param set /motor_bus_front kd 0.3
```

Changes take effect immediately on the next motor tick. Update `robot.yaml` when satisfied so the value survives restart.
