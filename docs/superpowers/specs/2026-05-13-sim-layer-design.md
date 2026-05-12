# Simulation Layer Design

## Overview

Refactor `legged_control` codebase into clean separation: data sources vs data processing. The processing layer is shared between real hardware and Gazebo simulation — only the data source layer differs.

## Architecture

```
legged_control/legged_control/
├── real/                          # Hardware data source
│   ├── motor_bus_node.py          #   RS485 → /<ns>/joint_states (per-joint)
│   └── joint_aggregator.py        #   12 per-joint topics → /joint_states_aggregated (motor frame, YAML order)

├── sim/                           # Simulation data source
│   ├── gazebo_control_bridge.py   #   Gazebo ↔ /joint_states_aggregated (bidirectional, URDF↔motor frame conversion)
│   └── policy_node_sim.py         #   Temporary — delete once real policy_node runs directly in sim

├── processing/                    # Data processing (shared: real and sim)
│   ├── state_estimator_node.py    #   /odin1/imu/filtered + /joint_states_agg → /state_estimate (9 floats)
│   ├── height_scan_node.py        #   /camera/depth/* → /height_scan (325 floats)
│   ├── teleop_node.py             #   /joy → /cmd_vel
│   └── urdf_joint_state_bridge.py #   motor frame → URDF frame /joint_states (TF tree for RViz)

├── test/                          # Visualization / debugging
│   ├── obs_monitor_node.py        #   Terminal 2 Hz obs display
│   └── vel_viz_node.py            #   Velocity arrows MarkerArray for RViz

├── kinematics.py                  # Pure math library: FK, IK, Jacobian, gravity projection, yaw rotation
└── policy_node.py                 # Policy: reads 4 obs topics → /joint_commands
```

## External Data Sources

| Signal | Real Source | Sim Source |
|--------|-------------|------------|
| Joints | `real/motor_bus_node` + `joint_aggregator` | `sim/gazebo_control_bridge` |
| IMU | `odin_ros_driver/host_sdk_sample` (same repo) | URDF Gazebo IMU plugin |
| Depth | `realsense2_camera` (apt) | URDF Gazebo depth plugin |
| IMU filter | `imu_filter_madgwick` (apt) | Same |
| Gamepad | `joy` (apt) | Same |

## Data Flow (Real)

```
motor_bus ×2 ──→ joint_agg ──→ /joint_states_aggregated ──┐
host_sdk_sample → imu_filter → /odin1/imu/filtered ───────┤
realsense2_camera ──→ /camera/depth/* ────────────────────┤
joy_node → teleop → /cmd_vel ─────────────────────────────┤
                                                           ├──→ policy_node → /joint_commands
              ┌─ processing layer ────────────────────────┘
              │
state_estimator → /state_estimate
height_scan     → /height_scan
teleop          → /cmd_vel
```

## Data Flow (Sim)

```
Gazebo IMU (URDF plugin) → /odin1/imu → imu_filter → /odin1/imu/filtered ──┐
Gazebo joints → control_bridge → /joint_states_aggregated ──────────────────┤
Gazebo depth (URDF plugin) → /camera/depth/* → height_scan → /height_scan ──┤
joy_node → teleop → /cmd_vel ───────────────────────────────────────────────┤
                                                                              ├──→ policy_node
                                                                              │
                                  same processing layer as real
```

Same 4 obs topics consumed by `policy_node.py` in both environments.

## Observation Vector (373 dims)

| Index | Len | Field | Source |
|-------|-----|-------|--------|
| 0-2 | 3 | base_lin_vel | `/state_estimate[0:3]` |
| 3-5 | 3 | base_ang_vel | `/state_estimate[3:6]` |
| 6-8 | 3 | projected_gravity | `/state_estimate[6:9]` |
| 9-11 | 3 | velocity_commands | `/cmd_vel` |
| 12-23 | 12 | joint_pos_rel | `/joint_states_aggregated` (motor frame, reordered) |
| 24-35 | 12 | joint_vel | `/joint_states_aggregated` |
| 36-47 | 12 | last_action | Internal |
| 48-372 | 325 | height_scan | `/height_scan` |

## Launch Files

| File | What it starts |
|------|---------------|
| `real.launch.py` | Hardware data sources only |
| `test.launch.py` | Real data sources + processing + RViz |
| `robot.launch.py` | Real data sources + processing + policy (modes: passive/policy) |
| `gazebo_physics.launch.py` | Gazebo + robot model + ros2_control (unchanged) |
| `gazebo_sim.launch.py` | **New** — simulation data source + processing + policy |

## Implementation Tasks

### Phase 1: Refactor directory structure
1. Rename `real2sim/` → `processing/`, update all imports
2. Update `setup.py` entry_points for new paths
3. Update test imports
4. Update launch files for new executable names (none should change)

### Phase 2: Add depth camera to URDF
1. Add `<sensor type="depth">` Gazebo plugin to `dog_urdf.urdf` on `base_link`
2. Configure to publish `/camera/depth/image_rect_raw` + camera_info

### Phase 3: Create `gazebo_sim.launch.py`
1. Wraps `gazebo_physics.launch.py`
2. Starts `gazebo_control_bridge`, `imu_filter_madgwick`, all processing nodes, `policy_node`
3. Deprecates `gazebo_policy.launch.py`

### Phase 4: Verify end-to-end
1. Verify `gazebo_control_bridge` produces correct `/joint_states_aggregated`
2. Verify all 4 obs topics flow in simulation
3. Verify `policy_node.py` runs unchanged in sim
4. Delete `policy_node_sim.py` after verification

### Phase 5: Deployment script
1. `scripts/install_deps.sh` — apt packages + udev rules

## Configuration Changes

- `odin_ros_driver/config/control_command.yaml`: `custom_map_mode: 0` (already done)
- URDF: add depth camera sensor (new)
- `config/robot_sim.yaml`: review/update for new naming

## Key Design Decisions

1. **processing/ is topic-agnostic**: All nodes read standard ROS2 topics. They don't know or care whether data comes from real hardware, Gazebo, or a replay tool.
2. **real/ and sim/ produce identical topic interfaces**: `/joint_states_aggregated` format is the same in both environments.
3. **policy_node unchanged**: Runs identically in real and sim — same code, same topics, same config.
4. **kinematics.py stays top-level**: Pure math, used by both `state_estimator_node` and `policy_node`.
