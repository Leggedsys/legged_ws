# sim 层

仿真数据源层。将 Gazebo 产出的传感器数据转换为标准 ROS2 topic 格式，对接 `processing/` 层。本层不涉及真实硬件。

---

## 节点列表

### gazebo_control_bridge

**职责：** 双向桥接 Gazebo ros2_control 与 legged_control topic 格式。

- **Gazebo → legged_control：** 订阅 `/joint_states`（`_joint` 后缀名），转换为 YAML 顺序，发布 `/joint_states_aggregated`（URDF frame）
- **legged_control → Gazebo：** 订阅 `/joint_commands`（YAML 顺序，URDF frame），转换为 sim 顺序，发布到 `/gait_position_controller/commands`

全部工作在 URDF 坐标系，不做 direction/zero_offset 转换。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_states_aggregated` | `sensor_msgs/JointState` (URDF frame, YAML 顺序) |
| 发布 | `/gait_position_controller/commands` | `std_msgs/Float64MultiArray` |
| 订阅 | `/joint_commands` | `sensor_msgs/JointState` (URDF frame, YAML 顺序) |
| 订阅 | `/joint_states` | `sensor_msgs/JointState` (Gazebo 原始，`_joint` 后缀) |

### policy_node_sim（暂保留）

仿真版策略节点，简化版（无状态机，直接 URDF 帧）。预计在 `policy_node.py` 能直接在仿真运行后移除。

---

## 数据流

```
Gazebo IMU (URDF 插件) ──→ /odin1/imu → imu_filter → /odin1/imu/filtered ──┐
Gazebo joints ──→ control_bridge ──→ /joint_states_aggregated ──────────────┤
joy_node ──→ teleop ──→ /cmd_vel + /height_command ─────────────────────────┤
                                                                              ├──→ policy_node
                            (盲策略：49 维观测，无高程扫描)                    │
                                                                              │
                               以上全部是 processing/ 层节点，sim 只做数据源
```

## 启动

```bash
ros2 launch legged_control gazebo_sim.launch.py
# 或 headless:
ros2 launch legged_control gazebo_sim.launch.py gui:=false rviz:=true
```
