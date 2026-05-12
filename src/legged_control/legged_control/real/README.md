# real 层

硬件接口层。负责与所有物理硬件直接通信，向上层提供统一的传感器数据，并接收上层下发的关节目标位置写入电机。

---

## 节点列表

### motor_bus_node

**职责：** 管理单条 RS485 总线上的所有电机，同时承担关节度数采集和关节目标位置输出两个功能。

每条总线启动一个实例，共两个实例：
- `motor_bus_front`：FR、FL 腿（`serial_port_front`）
- `motor_bus_rear`：RR、RL 腿（`serial_port_rear`）

节点在每个 tick 内对所有分配关节依次执行 `sendRecv`，保证总线独占不冲突。收到的度数经齿轮比换算后发布；下发目标位置前同样乘以齿轮比还原为电机轴转角。

节点关闭时（Ctrl+C 或其他停止信号）有软停逻辑：保持最后目标 0.5 s，再用 2 s 将 kp 线性降至 0，避免机器人直接塌陷。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/<ns>/joint_states` | `sensor_msgs/JointState` |
| 订阅 | `/joint_commands` | `sensor_msgs/JointState` |
| 参数 | `kp` / `kd` / `loop_hz` / `serial_port` / `joint_names` | — |

---

### joint_aggregator

**职责：** 将 `motor_bus_node` 发布的 12 路独立关节话题合并为一条聚合消息，供上层节点统一消费。

以 `robot.yaml` 中的 joints 顺序排列（FR_hip … RL_calf，索引 0–11）。数据为电机坐标系原始值，不做 direction/zero_offset 转换。任意一路关节收到新消息时立即触发发布。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_states_aggregated` | `sensor_msgs/JointState` |
| 订阅 | `/<ns>/joint_states` ×12 | `sensor_msgs/JointState` |

---

### state_estimator_node

**职责：** 融合 IMU 与关节反馈，估计机身线速度、角速度和重力方向，输出策略所需的状态向量。

速度估计采用互补滤波：运动学速度（假设四足全部接触地面的雅可比计算）权重 0.8，IMU 积分速度权重 0.2。

输出的 9 维向量与策略观测向量 obs[0:9] 直接对应：

| 索引 | 内容 | 坐标系 |
|------|------|--------|
| 0–2 | 机身线速度 (m/s) | yaw frame |
| 3–5 | 机身角速度 (rad/s) | body frame |
| 6–8 | 归一化重力投影 | body frame |

| | Topic | 类型 |
|---|---|---|
| 发布 | `/state_estimate` | `std_msgs/Float32MultiArray` (9 floats) |
| 订阅 | `odin1/imu/filtered` | `sensor_msgs/Imu` |
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState` |

---

### height_scan_node

**职责：** 将深度相机的深度图转换为策略所需的 325 格高度扫描网格（obs[48:373]）。

扫描区域在 base_link 坐标系下：x ∈ [0.10, 1.30] m（正前方），y ∈ [−0.30, 0.30] m，分辨率 0.05 m，共 25×13 = 325 格，x 快变（index = y_idx × 25 + x_idx）。

格值含义：`sensor_z − hit_z`，正值 = 地面低于传感器（平地/坑），负值 = 地面高于传感器（台阶/障碍）。结果 clip 到 [−1, 1]。

同时发布 `/height_scan_cloud`（`sensor_msgs/PointCloud2`）供 RViz 可视化。

依赖静态 TF：`base_link → camera_link`，须由启动文件保证已发布。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/height_scan` | `std_msgs/Float32MultiArray` (325 floats) |
| 发布 | `/height_scan_cloud` | `sensor_msgs/PointCloud2` |
| 订阅 | `/camera/depth/image_rect_raw` | `sensor_msgs/Image` (16UC1, mm) |
| 订阅 | `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |

---

### teleop_node

**职责：** 将手柄输入（`/joy`）转换为速度指令和姿态指令，供上层控制节点消费。

左摇杆控制前进/侧移速度，右摇杆控制偏航速度，LT/RT 控制站立高度变化率。A 键切换站立/卧倒（发布 `/posture_command`）。所有轴映射和反转均可通过 `robot.yaml` 的 `teleop` 段配置。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/posture_command` | `std_msgs/Bool` |
| 订阅 | `/joy` | `sensor_msgs/Joy` |

---

### urdf_joint_state_bridge

**职责：** 将 `/joint_states_aggregated`（电机坐标系）转换为 URDF 坐标系的 `/joint_states`，供 `robot_state_publisher` 和 RViz 使用。

转换公式：`q_urdf = direction × q_motor + zero_offset`，参数来自 `robot.yaml`。同时将关节名加上 `_joint` 后缀以匹配 URDF 命名（如 `FR_hip` → `FR_hip_joint`）。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_states` | `sensor_msgs/JointState`（URDF frame，含 `_joint` 后缀） |
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState`（motor frame） |

---

## 层接口

```
外部硬件                     real 层对外发布（供上层订阅）
─────────────────────────────────────────────────────────
RS485 电机     →  motor_bus_node + joint_aggregator  →  /joint_states_aggregated
Odin1 IMU      →  (odin_ros_driver) + state_estimator →  /state_estimate
RealSense      →  height_scan_node                   →  /height_scan
手柄           →  teleop_node                        →  /cmd_vel, /posture_command

上层下发                     real 层对外订阅
─────────────────────────────────────────────────────────
/joint_commands  →  motor_bus_node  →  RS485 电机
```
