# RL Policy Modes Design

**Date:** 2026-05-11
**Branch:** dev/rl-policy
**Scope:** 在 `legged_control` 包内新增 `passive`（扩展版）、`policy`、`simulation` 三个模式，同时清理冗余代码，`position_control` 模式完全不动。

---

## 1. 代码变更清单

### 删除

| 文件 | 原因 |
|------|------|
| `legged_control/stand_node.py` | 功能完全被 `gait_node` WAIT 阶段覆盖 |
| `legged_control/standup_node.py` | 状态机逻辑完全被 `gait_node` 覆盖；`_smoothstep` 移入 `kinematics.py` |
| `legged_control/go_m8010_6_node.py` | 未注册到 entry_points，无任何引用，死代码 |
| `launch/gazebo_stand.launch.py` | stand mode 删除 |

### 保留不动

`motor_bus_node`、`joint_aggregator`、`teleop_node`、`gait_node`、`gazebo_control_bridge`、`fake_motor_bus_node`、`urdf_joint_state_bridge`、`kinematics`、`gazebo_physics.launch.py`、`position_control_sim.launch.py`

### 修改

| 文件 | 变更内容 |
|------|---------|
| `legged_control/kinematics.py` | 添加 `_smoothstep` 函数（从 standup_node 迁移） |
| `legged_control/gait_node.py` | import 改为从 `kinematics` 引入 `_smoothstep` |
| `legged_control/passive_monitor_node.py` | 改订阅 `/joint_states_aggregated`（替代 12 个独立话题）；扩展显示完整 obs（关节、IMU 状态、cmd_vel、height_scan 摘要） |
| `launch/robot.launch.py` | 移除 stand/standup mode；新增 passive（扩展版）、policy、simulation mode |
| `setup.py` | 移除 stand_node/standup_node entry_points；新增 state_estimator_node、height_scan_node、policy_node |

### 新增

| 文件 | 职责 |
|------|------|
| `legged_control/state_estimator_node.py` | IMU 姿态 + 接触运动学速度估计，发布 `/state_estimate` |
| `legged_control/height_scan_node.py` | D435 深度图 → 325 维高程向量，发布 `/height_scan` |
| `legged_control/policy_node.py` | 状态机 + 373 维 obs 组装 + TorchScript 推理，50 Hz |
| `legged_control/msg/StateEstimate.msg` | 自定义消息：`base_lin_vel[3]`、`base_ang_vel[3]`、`projected_gravity[3]` |
| `launch/gazebo_policy.launch.py` | simulation mode 入口 |
| `config/policy.yaml` | 策略路径、动作 scale、关节符号映射等策略相关参数 |

---

## 2. 话题图

```
odin_ros_driver ──────────────── odin1/imu ─────────────────────────┐
                                                                     ▼
motor_bus_node ×2 → /<ns>/joint_states → joint_aggregator → /joint_states_aggregated
                                                                     │
                                         ┌───────────────────────────┤
                                         ▼                           │
                              state_estimator_node                   │
                                         │                           │
                                         └──→ /state_estimate ───────┤
                                                                     │
realsense2_camera → /camera/depth/image_rect_raw                     │
        │                                                            │
        ▼                                                            │
height_scan_node → /height_scan ─────────────────────────────────────┤
                                                                     │
joy_node → /joy → teleop_node → /cmd_vel ────────────────────────────┤
                              → /posture_command ────────────────────┤
                                                                     ▼
                                                             policy_node
                                                                     │
                                                                     └──→ /joint_commands → motor_bus_node ×2
```

---

## 3. 四个 mode 的节点组成

### `passive`

motors kp=kd=0，所有传感器运行，终端显示完整 obs，用于传感器链路验证与参数对齐。

```
motor_bus_node ×2 (kp=kd=0)
joint_aggregator
odin_ros_driver
imu_filter_madgwick
realsense2_camera
state_estimator_node
height_scan_node
joy_node + teleop_node
passive_monitor_node (扩展版)
```

### `position_control`（不动）

```
motor_bus_node ×2
joint_aggregator
joy_node + teleop_node
gait_node
```

### `policy`

```
motor_bus_node ×2
joint_aggregator
odin_ros_driver
imu_filter_madgwick
realsense2_camera
state_estimator_node
height_scan_node
joy_node + teleop_node
policy_node
```

### `simulation`

Gazebo 物理仿真，URDF 需包含仿真 IMU 插件和深度相机插件，使用与真机完全相同的 state_estimator + height_scan 链路。

```
Gazebo 物理栈（含 IMU plugin、depth camera plugin）
gazebo_control_bridge
state_estimator_node
height_scan_node
joy_node + teleop_node
policy_node
```

---

## 4. 观测向量（373 维）

| 索引 | 维度 | 字段 | 来源 |
|------|------|------|------|
| 0–2 | 3 | `base_lin_vel` | `state_estimator_node` |
| 3–5 | 3 | `base_ang_vel` | `state_estimator_node` |
| 6–8 | 3 | `projected_gravity` | `state_estimator_node` |
| 9–11 | 3 | `velocity_commands` | `teleop_node` → `/cmd_vel` |
| 12–23 | 12 | `joint_pos_rel` | `/joint_states_aggregated` − `default_q` |
| 24–35 | 12 | `joint_vel` | `/joint_states_aggregated` |
| 36–47 | 12 | `last_action` | `policy_node` 内部状态 |
| 48–372 | 325 | `height_scan` | `height_scan_node` |

### 关节顺序（policy 期望，12 维字段共用）

```
[0] FL_hip  [1] FR_hip  [2] FL_thigh  [3] FR_thigh
[4] FL_calf [5] FR_calf [6] RL_hip    [7] RR_hip
[8] RL_thigh [9] RR_thigh [10] RL_calf [11] RR_calf
```

`policy_node` 内维护从 `robot.yaml` 顺序到 policy 顺序的静态索引映射。

---

## 5. 动作解码

```python
q_target = q_default + action * scale
```

| 关节组 | `scale` | `q_default` (URDF frame) |
|--------|---------|--------------------------|
| `*_hip` | 0.15 | 0.0 rad |
| `*_thigh` | 0.20 | 0.7 rad |
| `*_calf` | 0.15 | −1.2 rad |

**符号修正：** FR hip 和 RL hip 在电机帧输出前额外乘以 −1（USD 转换轴方向丢失问题，见 `DOG_JOINT_SIGN`）。

`q_target` 在 URDF 帧，经 `direction × (q_urdf − zero_offset)` 转换为电机帧后发送给 `motor_bus_node`，即通过已有 `robot.yaml` 中的 `direction` / `zero_offset` 字段完成。

---

## 6. `policy_node` 状态机

与 `gait_node` 结构完全一致，仅 TROT 阶段替换为 NN 推理：

```
PASSIVE ──posture_command=true──→ STANDUP ──到位且稳定──→ WAIT
  ↑                                                          │ 有 cmd_vel
  │                                                          ▼
LIE_DOWN ←──posture_command=false──── POLICY ←─────────────→ WAIT
  │                                                          
  └──到位且稳定──→ PASSIVE (kp=kd=0)
```

推理频率 50 Hz，与训练控制频率一致。

---

## 7. `state_estimator_node` 设计

**输入：** `odin1/imu`（`sensor_msgs/Imu`，含 `linear_acceleration` + `angular_velocity`，无姿态）、`/joint_states_aggregated`

**姿态估计：** 使用 `imu_filter_madgwick` 独立节点处理 `odin1/imu` → `odin1/imu/filtered`（含 `orientation`），state_estimator_node 订阅 filtered 话题。

**速度估计：** 接触运动学线性卡尔曼滤波：
- 支撑腿足端速度 ≈ 0 → 机身速度 = −J_contact · dq
- 多支撑腿加权融合
- KF 状态：`[v_x, v_y, v_z]`，观测来自运动学，预测来自 IMU 积分

**输出：** `/state_estimate`（`StateEstimate.msg`）

**注意：** Odin IMU 轴映射非标准（x/y 互换且 x 取反），state_estimator_node 内部做轴对齐，对齐方式在 `passive` 模式下通过与 URDF 可视化比对验证。

---

## 8. `height_scan_node` 设计

**输入：** `/camera/depth/image_rect_raw`（D435，`sensor_msgs/Image`，16UC1，单位 mm）

**处理：**
1. 深度图反投影为点云（相机坐标系）
2. TF 变换到 base_link 坐标系（需 `base_link → camera_link` 静态 TF）
3. 构建 2D 高程格：x∈[0.10, 1.30]，y∈[−0.30, +0.30]，分辨率 0.05m，25×13
4. 每个格子取最小 z（最高点）作为地面高度
5. `height_scan[i] = sensor_z − hit_z`，clip 到 [−1, 1]，排列顺序 x-major

**输出：** `/height_scan`（`std_msgs/Float32MultiArray`，325 维）

---

## 9. 外部依赖

| 包 | 用途 |
|----|------|
| `realsense2_camera` | D435 ROS2 驱动 |
| `imu_filter_madgwick` | IMU 姿态估计 |
| `torch`（Python） | TorchScript policy 推理 |

---

## 10. 对齐验证流程（passive mode）

1. **电机 offset/direction**：在 passive mode 终端观察关节角度，与 URDF 可视化（RViz）对比
2. **IMU 方向**：倾斜机器人，观察 `projected_gravity` 方向是否与实际一致
3. **高程图**：在终端或 RViz 中可视化 `/height_scan` 点云，确认前向扫描范围正确
4. **手柄键位**：终端显示 `velocity_commands`，确认手柄各轴映射正确
