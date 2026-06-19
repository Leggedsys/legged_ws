# 实机测试手册

## 前置条件

### 硬件

- [ ] 12 个电机上电，两条 RS485 串口线插入主机
- [ ] Odin1 激光雷达 USB 插入主机
- [ ] RealSense D435 深度相机 USB 插入主机
- [ ] 游戏手柄连接（Betop Kunpeng 20 或兼容）
- [ ] 机器人放在地面，四条腿悬空或支撑好

### 软件

```bash
# 首次部署
git clone --recursive git@github.com:Leggedsys/legged_ws.git
cd legged_ws
git checkout dev/rl-policy
bash scripts/install_deps.sh
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 配置检查

| 文件 | 检查项 |
|------|--------|
| `config/robot.yaml` | `serial_port_front`/`serial_port_rear` 串口号、`kp`/`kd` 增益、`max_joint_speed` 速度限制 |
| `config/policy.yaml` | `model_path` 模型路径、`joint_default_q_urdf` 站姿角度 |
| `odin_ros_driver/config/control_command.yaml` | `custom_map_mode: 0`（纯里程计模式） |

---

## 第一步：Passive 模式验证

**目标：** 确认所有传感器和电机通信正常，关节角度在终端正确显示。

```bash
ros2 launch legged_control test.launch.py rviz:=true
```

### 检查项

| 检查 | 预期 | 不符合时 |
|------|------|----------|
| 终端 12 关节数据显示（非 nan） | `FR_thigh pos_rel` 等出现数值 | 检查电机供电和串口 |
| 关节数值稳定不跳变 | `vel` 在 ±0.1 以内 | 检查 `data.correct` 过滤是否正常 |
| IMU 数据显示 | `proj_grav gz≈+1.0`（狗正立时） | 检查 Odin USB、`custom_map_mode: 0` |
| 深度相机点云 | RViz 里看到环境 | 检查 RealSense USB |
| Odin 点云 | RViz 里看到彩色点云 | Odin 上电 + USB |
| RViz 关节模型随实物运动 | 拨动关节，模型同向同幅 | 校准 `direction` 和 `zero_offset` |
| 手柄摇杆→终端 `cmd_vel` | `vx`/`vy`/`wz` 有值 | 检查手柄连接 |

### 校准 direction / zero_offset

如果 RViz 模型和实物运动不一致：

1. 向 URDF 正方向拨动关节，终端看读数
2. 读数增大 → `direction=1`，减小 → `direction=-1`
3. 关节摆到 URDF 零位，记录 `q_motor`，`zero_offset = -direction × q_motor`
4. 改 `robot.yaml` 后重新 `colcon build --packages-select legged_control && source install/setup.bash`

### 限位验证

终端 joint 行左侧有 `*`（逼近下限）或 `!`（逼近上限）标记。逐步推关节到极限，确认标记出现且与实际限位一致。

---

## 第二步：Policy 模式

**目标：** 确认策略完整链路——起身→等待→策略激活→趴下。

### 2.1 启动

```bash
ros2 launch legged_control robot.launch.py mode:=policy
```

终端显示：
```
[policy] policy_node ready — 50 Hz  model=loaded  kp=3.8  kd=0.26
```

狗处于 **PASSIVE** 状态——零力矩，手可以自由拨动关节。

### 2.2 起身

按手柄 **A 键** 或命令行：

```bash
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: true}"
```

终端显示：
```
[policy] posture=true -> STANDUP
```

狗从当前位姿平滑起身到站姿（约 6 秒，`robot.yaml standup.ramp_duration`）。终端持续显示关节位置向 `q_default_urdf` 收敛。

起身完成后：
```
[policy] standup complete -> WAIT
```

### 2.3 激活策略

**推手柄前进/后退/转向**，终端显示：
```
[policy] cmd_vel received -> POLICY
```

从此刻起每 50Hz 推理：
```
[policy] raw_action: [+0.403 +0.609 ...]
[policy] q_target(URDF)=[+0.091 +0.680 ...]  pos_rel(obs first 4)=[...]  cmd_vel=[+0.50 +0.00 +0.00]
```

**obs 门禁：** 如果进入前有任何 obs 维越界，终端显示：
```
🚨 obs out of range: gz=+1.000[-1.06,-0.93], hs_mean=+0.000[0.19,0.43]
```
此时不会进入 POLICY，需检查传感器数据。

### 2.4 停止

- **正常停止：** 手柄归中 → 策略自动回 `WAIT`（保持站姿）
- **趴下：** 再按 A 键或 `posture_command false` → `LIEDOWN` → `PASSIVE`
- **急停：** 按住手柄 **B 键** → 立即趴下到 PASSIVE（零力矩）
- **强制停止：** `Ctrl+C` → 电机 0.5s 保位 + 2s kp 归零，狗缓慢落地

---

## 调试命令

```bash
# 查看关节聚合数据
ros2 topic echo /joint_states_aggregated --field name --field position

# 查看状态估计
ros2 topic echo /state_estimate --once

# 查看策略输出
ros2 topic echo /joint_commands --field position

# 查看 obs 向量
ros2 topic echo /observation --once

# 查看电机温度
ros2 launch legged_control robot.launch.py mode:=passive 2>&1 | grep -i "warm\|OVERHEAT\|FAULT"

# 直接发命令起身/趴下
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: false}"

# 直接发速度指令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# 只看策略日志
ros2 launch legged_control robot.launch.py mode:=policy 2>&1 | grep "\[policy\]"
```

---

## 常见问题

| 现象 | 排查 |
|------|------|
| 电机无响应 | 串口权限 `sudo usermod -aG dialout $USER`，检查 `/dev/ttyUSB0` 存在 |
| 电机数据跳变 | 检查电机供电、RS485 终端电阻 |
| 终端 `proj_grav gz=+1.0` 不是 `-1.0` | Odin 安装方向需要校准 IMU 坐标系 |
| `policy_node` 不进入 POLICY | 确认站立完成（`standup complete -> WAIT`），推手柄后有非零 `/cmd_vel` |
| `🚨 obs out of range` | 检查 `gz` 接近 -1.0、`height_scan mean` 在 0.19-0.43、`joint_pos` 在 ±0.3 |
| `raw_action` 超 ±10 | 检查 `joint_pos_rel` 是否减了 `q_default`，`last_action` 是否为原始输出 |
| 狗倒下 | 立即按 B 键急停或 Ctrl+C |
| 电机报 `FAULT` | 检查电机错误码含义（overheat/overcurrent/overvoltage/encoder），立即断电排查 |
