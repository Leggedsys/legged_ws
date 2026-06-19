# real 层

硬件数据源层。与物理硬件直接通信，将原始传感器数据和电机指令转换为标准 URDF 坐标系 topic，供 `processing/` 和 `policy_node` 消费。

---

## 节点列表

### motor_bus_node

**职责：** 管理单条 RS485 总线上的所有电机，采集关节状态，下发目标位置。

每条总线一个实例：`motor_bus_front`（FR/FL）、`motor_bus_rear`（RR/RL）。在每个 tick 内依次执行 `sendRecv`，保证总线独占。

软停逻辑：指令停止后保持 0.5s，再 2s 内 kp 线性降至 0（kd 保持），机器人缓慢趴下。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/<ns>/joint_states` ×12 | `sensor_msgs/JointState` (motor frame) |
| 订阅 | `/joint_commands_motor` | `sensor_msgs/JointState` (motor frame) |
| 参数 | `kp` / `kd` / `loop_hz` / `serial_port` / `joint_names` | — |

### joint_aggregator

**职责：** 合并 12 路独立关节话题，**转换为 URDF 坐标系**后发布。

转换公式：`q_urdf = direction × q_motor + zero_offset`，`dq_urdf = direction × dq_motor`。参数来自 `robot.yaml`。YAML 顺序排列（FR_hip … RL_calf）。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_states_aggregated` | `sensor_msgs/JointState` (URDF frame) |
| 订阅 | `/<ns>/joint_states` ×12 | `sensor_msgs/JointState` (motor frame) |

### motor_command_bridge

**职责：** 将 `policy_node` 输出的 URDF 坐标系指令逆转换为电机坐标系，供 `motor_bus_node` 执行。

转换：`q_motor = direction × (q_urdf − zero_offset)`，clip 到 `q_min`/`q_max`。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_commands_motor` | `sensor_msgs/JointState` (motor frame) |
| 订阅 | `/joint_commands` | `sensor_msgs/JointState` (URDF frame) |

### urdf_joint_state_bridge

**职责：** 将 `/joint_states_aggregated`（已是 URDF 坐标系）重命名后发 `/joint_states`，供 `robot_state_publisher` 构建 TF 树。

关节名加 `_joint` 后缀匹配 URDF。无电机数据时以 1Hz 发送默认零位，保持 TF 树存活。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_states` | `sensor_msgs/JointState` |
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState` |

---

## 外部数据源（不在此层，但与其配合）

| 信号 | 节点 | 位置 |
|------|------|------|
| IMU | `host_sdk_sample` | `odin_ros_driver` (同仓库) |
| 深度 | `realsense2_camera_node` | apt 安装 |
| 滤波 | `imu_filter_madgwick` | apt 安装 |

---

## 数据流

```
motor_bus ×2 ──→ motor frame /<ns>/joint_states
                              │
                              ▼
                      joint_aggregator (motor → URDF)
                              │
                              ▼
                 /joint_states_aggregated (URDF frame)
                              │
    ┌─────────────────────────┼─────────────────────┐
    ▼                         ▼                     ▼
urdf_joint_state_bridge   state_estimator       policy_node
    │                                                    │
    ▼                                                    │
/joint_states (TF)                                /joint_commands (URDF)
                                                        │
                                                        ▼
                                               motor_command_bridge (URDF → motor)
                                                        │
                                                        ▼
                                               /joint_commands_motor
                                                        │
                                                        ▼
                                                   motor_bus ×2
```
