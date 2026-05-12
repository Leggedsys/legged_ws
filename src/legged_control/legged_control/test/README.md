# test 层

硬件数据链路验证层。在不运行策略的情况下，对 real 层和 real2sim 层的输出进行观测和可视化。

---

## 节点列表

### obs_monitor_node

**职责：** 在终端以 2 Hz 输出完整观测向量，用于数据链路快速核查。

显示内容：
- 各关节位置偏差（相对 `policy.yaml` 中 `q_default`）与速度
- 机身线速度、角速度、重力投影（来自 `/state_estimate`）
- 速度指令
- 高程图统计（均值、标准差、最小值、最大值）

| | Topic | 类型 |
|---|---|---|
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState` |
| 订阅 | `/state_estimate` | `std_msgs/Float32MultiArray` (9 floats) |
| 订阅 | `/height_scan` | `std_msgs/Float32MultiArray` (325 floats) |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` |

---

### vel_viz_node

**职责：** 将速度数据发布为 RViz MarkerArray 箭头，便于直观判断估计速度与指令速度是否一致。

- **绿色箭头** — 估计线速度（来自 `/state_estimate[0:2]`，yaw frame）
- **蓝色箭头** — 指令线速度（来自 `/cmd_vel`）

箭头长度与速度大小成正比，锚点在 `base_link` 原点。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/vel_viz` | `visualization_msgs/MarkerArray` |
| 订阅 | `/state_estimate` | `std_msgs/Float32MultiArray` |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` |

---

## 测试操作流程

### 第一步：启动测试栈

将机器人放在桌上（腿悬空），连接所有硬件后：

```bash
ros2 launch legged_control test.launch.py
```

---

### 第二步：校准 `direction` 和 `zero_offset`

这是测试的核心任务。目标：使 RViz 中每个关节的视觉角度与实物完全一致。

**参数含义：**

```
q_urdf = direction × q_motor + zero_offset
```

- `direction`：`+1` 表示电机正方向与 URDF 关节正方向一致，`-1` 表示相反
- `zero_offset`：电机零位与 URDF 零位之间的静态偏差（rad）

**逐关节校准步骤（每条腿重复，共 12 个关节）：**

#### 2a. 确定 `direction`

1. 用手将某关节从当前位置向**正方向**拨动（URDF 正方向通常定义为：hip 外展为正，thigh 前摆为正，calf 伸直为正）
2. 观察终端 `/joint_states_aggregated` 中该关节的读数是否**增大**
   ```bash
   ros2 topic echo /joint_states_aggregated --field name --field position
   ```
3. 如果读数增大 → `direction = 1`；如果读数减小 → `direction = -1`
4. 修改 `config/robot.yaml` 对应关节的 `direction` 字段

#### 2b. 确定 `zero_offset`

1. 将该关节**手动摆到 URDF 零位**（参考 URDF 关节定义，通常是腿自然垂直时的位置）
2. 读取此时的 `q_motor`（终端或 `ros2 topic echo`）
3. 计算：`zero_offset = -direction × q_motor`
4. 修改 `config/robot.yaml` 对应关节的 `zero_offset` 字段

#### 2c. 验证

修改后无需重新编译，直接重启 launch：

```bash
# 重启以加载新参数
ros2 launch legged_control test.launch.py
```

将该关节手动摆到 URDF 零位，RViz 中对应关节角度应显示 `≈ 0`。再向正方向拨动，RViz 模型应同向运动。

**12 个关节全部校准完成后，进行整体视觉验证：**

- 将机器人摆成近似站立姿态，RViz 中模型应与实物姿态吻合
- 各腿左右对称，没有明显镜像错误

---

### 第三步：验证其余传感器

`direction` / `zero_offset` / `default_q` 确认后，逐项检查：

| 验证项 | 操作 | 预期结果 |
|--------|------|----------|
| **重力方向** | 机器人水平放置 | 终端 `proj_grav ≈ [0, 0, -1]` |
| **重力方向** | 机器人前倾约 30° | `gx` 明显增大 |
| **速度估计** | 静止 | `lin_vel ≈ [0, 0, 0]` |
| **速度估计** | 水平推动机器人 | 绿色箭头与推动方向一致 |
| **高程图** | 相机对准平地 | RViz PointCloud2 均匀分布，`mean ≈ camera_height` |
| **速度指令** | 推摇杆前进 | 蓝色箭头指向 +x 方向 |
