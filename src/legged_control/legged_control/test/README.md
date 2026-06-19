# test 层

数据链路验证层。在不运行策略的情况下观测 `real/`、`sim/` 和 `processing/` 的输出。

---

## 节点列表

### obs_monitor_node

**职责：** 终端 2Hz 输出完整观测向量，快速核查数据链路。

| 显示项 | 来源 |
|--------|------|
| 关节位置偏差 (vs `policy.yaml` q_default) | `/joint_states_aggregated` (URDF frame) |
| 关节速度 | `/joint_states_aggregated` |
| 机身线速度 / 角速度 / 重力投影 | `/state_estimate` |
| 速度指令 | `/cmd_vel` |
| 高程图统计 (mean/std/min/max) | `/height_scan` |

| | Topic | 类型 |
|---|---|---|
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState` |
| 订阅 | `/state_estimate` | `std_msgs/Float32MultiArray` (9 floats) |
| 订阅 | `/height_scan` | `std_msgs/Float32MultiArray` (325 floats) |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` |

### vel_viz_node

**职责：** 速度箭头 MarkerArray 可视化。

- 🟢 绿色箭头 — 估计线速度 (`/state_estimate[0:2]`)
- 🔵 蓝色箭头 — 指令线速度 (`/cmd_vel`)

| | Topic | 类型 |
|---|---|---|
| 发布 | `/vel_viz` | `visualization_msgs/MarkerArray` |
| 订阅 | `/state_estimate` | `std_msgs/Float32MultiArray` |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` |

---

## RViz 显示面板 (`config/test.rviz`)

| 面板 | 话题 | 说明 |
|------|------|------|
| RobotModel | `/robot_description` | URDF 模型 |
| HeightScan | `/height_scan_cloud` | 高程扫描绿点云 |
| VelocityArrows | `/vel_viz` | 速度箭头 |
| OdinCloudRender | `/odin1/cloud_render` | Odin RGB 着色点云 |
| OdinCloudRaw | `/odin1/cloud_raw` | Odin 强度点云 |
| OdinPath | `/odin1/path` | Odin 里程计轨迹 |
| TF | — | 全部坐标系 |

---

## 测试操作流程

### 第一步：启动测试栈

```bash
ros2 launch legged_control test.launch.py
```

### 第二步：校准 `direction` 和 `zero_offset`

目标：使 RViz 中每个关节的视觉角度与实物一致。

**参数含义：** `q_urdf = direction × q_motor + zero_offset`（在 `robot.yaml` 配置，由 `joint_aggregator` 执行转换）

**逐关节校准（12 个关节重复）：**

1. 向 URDF 正方向拨动关节，观察终端或：
   ```bash
   ros2 topic echo /joint_states_aggregated --field name --field position
   ```
2. 读数增大 → `direction = 1`；减小 → `direction = -1`，修改 `robot.yaml`
3. 将关节摆到 URDF 零位，记录 `q_motor`，计算 `zero_offset = -direction × q_motor`，修改 `robot.yaml`
4. 重启 launch 验证：RViz 关节角度 ≈ 0，拨动方向和模型一致

### 第三步：验证传感器

| 验证项 | 操作 | 预期 |
|--------|------|------|
| 重力方向 | 机器人水平 | `proj_grav ≈ [0, 0, 1]` |
| 重力方向 | 前倾 30° | gx 明显增大 |
| 速度估计 | 静止 | `lin_vel ≈ [0, 0, 0]` |
| 速度估计 | 水平推动 | 绿色箭头方向一致 |
| 高程图 | 对准平地 | RViz 点云均匀，`mean ≈ 机身离地高度` |
| 速度指令 | 推摇杆前进 | 蓝色箭头 → +x |
| Odin 点云 | Odin 上电 | RViz 显示彩色点云和轨迹 |
