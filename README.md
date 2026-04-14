# Legged Robot

## 前置

```bash
# 串口权限（只需一次，之后重新登录生效）
sudo usermod -a -G dialout $USER

# 编译
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 第一步：确定关节限位和默认位置（passive 模式）

passive 模式下电机输出零力矩，可以用手自由拨动关节。终端实时显示各关节角度。

建议从单腿开始，逐腿完成：

```bash
ros2 launch legged_control robot.launch.py legs:=FR
```

终端会持续刷新：

```
[passive]  FR  hip=  0.000  thigh=  0.800  calf= -1.500
           FL  hip=    nan  thigh=    nan  calf=    nan
           RR  hip=    nan  thigh=    nan  calf=    nan
           RL  hip=    nan  thigh=    nan  calf=    nan
```

对每个关节：

1. 把关节拨到**机械限位**，记录角度 → 填入 `q_min` / `q_max`
2. 把关节摆到**期望站姿位置**，记录角度 → 填入 `default_q`

把记录的值更新到 `src/legged_control/config/robot.yaml`，然后对下一条腿重复。

全部完成后重新编译使配置生效：

```bash
colcon build --packages-select legged_control
source install/setup.bash
```

---

## 第二步：让机器人站立（stand 模式）

stand 模式下所有关节以 PD 控制保持在 `default_q`。

```bash
ros2 launch legged_control robot.launch.py mode:=stand
```

如果姿态不对，回到第一步修正 `default_q`。

### 调整 PD 增益

默认增益在 `robot.yaml` 的 `control.kp` / `control.kd`。运行中可以实时调整，无需重启：

```bash
ros2 param set /stand_node kp 5.0
ros2 param set /stand_node kd 0.3
```

命令执行后立即对所有电机生效。满意后把最终值写回 `robot.yaml`，下次启动直接使用。

---

## 第三步：策略行走（policy 模式）

policy 模式加载 TorchScript RL 模型，以 50 Hz 运行推理，通过手柄控制机器人行走。

### 前提

- Odin1 LiDAR 已上线（提供 IMU 和里程计）
- 已导出 TorchScript 模型（`.pt` 文件）
- USB 手柄已连接

### 安装 joy 驱动（只需一次）

```bash
sudo apt install ros-humble-joy
```

### 配置模型路径

编辑 `src/legged_control/config/robot.yaml`，填入模型路径：

```yaml
policy:
  model_path: '/path/to/policy.pt'
```

重新编译使配置生效：

```bash
colcon build --packages-select legged_control
source install/setup.bash
```

### 启动

```bash
ros2 launch legged_control robot.launch.py mode:=policy
```

启动后自动运行：`motor_bus_node`（×2）、`joint_aggregator`、`joy_node`、`teleop_node`、`policy_node`。

### 手柄操作

默认轴映射（标准手柄布局）：

| 动作 | 摇杆 |
|------|------|
| 前进 / 后退 | 左摇杆 ↑↓ |
| 左移 / 右移 | 左摇杆 ←→ |
| 左转 / 右转 | 右摇杆 ←→ |

轴映射、死区、最大速度均可在 `robot.yaml` 的 `teleop` 段调整。

紧急停止按钮通过 `teleop.btn_emergency_stop` 配置（默认 `-1` = 未启用）。

### LiDAR 超时保护

若 `/odin1/imu`、`/odin1/odometry` 或关节状态任意一路超过 0.5 s 无数据，`policy_node` 停止发布 `/joint_commands`，电机保持上一帧位置。

---

## 常用选项

```bash
# 指定多条腿（逗号分隔）
ros2 launch legged_control robot.launch.py legs:=FR,FL

# 覆盖串口（前腿 FR/FL 用 serial_port_front，后腿 RR/RL 用 serial_port_rear）
ros2 launch legged_control robot.launch.py serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1

# 查看所有 launch 参数
ros2 launch legged_control robot.launch.py --show-args
```
