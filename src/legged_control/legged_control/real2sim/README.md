# real2sim 层

数据转换层。将 real 层输出的原始硬件数据处理成策略（policy）所需的标准格式。本层不与任何物理硬件直接通信。

---

## 节点列表

### state_estimator_node

**职责：** 融合 IMU 与关节反馈，估计机身状态，输出策略观测向量的前 9 维。

速度估计采用互补滤波（运动学权重 0.8，IMU 积分权重 0.2）。

| 输出索引 | 内容 | 坐标系 |
|----------|------|--------|
| 0–2 | 机身线速度 (m/s) | yaw frame |
| 3–5 | 机身角速度 (rad/s) | body frame |
| 6–8 | 归一化重力方向 | body frame |

| | Topic | 类型 |
|---|---|---|
| 发布 | `/state_estimate` | `std_msgs/Float32MultiArray` (9 floats) |
| 订阅 | `odin1/imu/filtered` | `sensor_msgs/Imu` |
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState` |

---

### height_scan_node

**职责：** 将深度相机深度图光线投射到 base_link 平面，生成策略观测向量的高度扫描段（obs[48:373]）。

扫描网格：x ∈ [0.10, 1.30] m，y ∈ [−0.30, 0.30] m，分辨率 0.05 m，25×13 = 325 格，x 快变。格值 clip 到 [−1, 1]，正值表示地面低于传感器。

依赖静态 TF：`base_link → camera_link`。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/height_scan` | `std_msgs/Float32MultiArray` (325 floats) |
| 发布 | `/height_scan_cloud` | `sensor_msgs/PointCloud2`（RViz 可视化） |
| 订阅 | `/camera/depth/image_rect_raw` | `sensor_msgs/Image` (16UC1, mm) |
| 订阅 | `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |

---

### teleop_node

**职责：** 将手柄原始轴值映射为速度指令和姿态指令，供策略消费。

左摇杆：前进/侧移；右摇杆：偏航；LT/RT：站立高度变化率；A 键：切换站立/卧倒。所有轴映射和死区均通过 `robot.yaml` 的 `teleop` 段配置。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/posture_command` | `std_msgs/Bool` |
| 订阅 | `/joy` | `sensor_msgs/Joy` |

---

### urdf_joint_state_bridge

**职责：** 将 real 层输出的电机坐标系关节数据转换为 URDF 坐标系，供可视化（`robot_state_publisher` / RViz）和需要 URDF 坐标的上层节点使用。

转换：`q_urdf = direction × q_motor + zero_offset`，参数来自 `robot.yaml`。关节名加 `_joint` 后缀以匹配 URDF（`FR_hip` → `FR_hip_joint`）。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/joint_states` | `sensor_msgs/JointState`（URDF frame） |
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState`（motor frame） |

---

## 层接口

```
real 层输入                    real2sim 处理              对外发布（供策略消费）
──────────────────────────────────────────────────────────────────────────────
/joint_states_aggregated  →  state_estimator_node   →  /state_estimate
odin1/imu/filtered        →  state_estimator_node
/camera/depth/...         →  height_scan_node       →  /height_scan
/joy                      →  teleop_node            →  /cmd_vel, /posture_command
/joint_states_aggregated  →  urdf_joint_state_bridge →  /joint_states（URDF frame）
```
