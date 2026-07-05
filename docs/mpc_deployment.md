# MPC 实机部署操作手册

**分支**：`dev/mpc-control`  
**适用版本**：MIT Cheetah MPC（τ_ff + 动态 kp/kd），2026-07-05

---

## 前置检查

```bash
# 每次上电前必做
cd ~/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash

# 确认串口已挂载
ls /dev/ttyUSB0 /dev/ttyUSB1

# 确认分支正确
git branch --show-current   # 应输出 dev/mpc-control
```

---

## 阶段 0：Dry-run 验证（不接电机）

先在台架上验证命令格式正确，不用通电。

```bash
ros2 launch legged_control robot.launch.py mode:=mpc dry_run:=true
```

观察 `/tmp/motor_command_log.csv`：

```bash
# 站立后推手柄，让机器走几步
column -t -s, /tmp/motor_command_log.csv | head -5

# 重点检查：
# 1. *_tau 列：支撑腿应有非零值（约 ±1–5 Nm），摆动腿应为 0
# 2. *_motor 列：值域是否在正常范围（hip ≈ ±2 rad，calf ≈ -15 rad）
# 3. 无 NaN / inf
```

```bash
# 同时在另一个终端检查话题
ros2 topic echo /joint_gains --once
# 期望：24 个浮点数，支撑腿 kp ≈ 0.125，摆动腿 kp ≈ 1.0

ros2 topic echo /joint_commands --once
# 期望：effort 字段有非零值
```

---

## 阶段 1：Passive 模式（零力矩）

**目的**：确认关节方向、zero_offset、电机通信正常。

```bash
ros2 launch legged_control robot.launch.py mode:=passive
```

用手拨动每条腿，在 monitor 或 rviz 观察关节角变化方向：
- 外展（腿向外张）→ hip q 应增大（正方向）
- 屈髋（大腿向前转）→ thigh q 应增大
- 屈膝（小腿向后折）→ calf q 应减小（更负）

如有方向错误，修改 `robot.yaml` 对应关节的 `direction` 或 `zero_offset`，rebuild 后重试。

```bash
# 实时监控所有关节（另开终端）
ros2 topic echo /joint_states_aggregated
```

---

## 阶段 2：MPC 站立——关闭动态增益（kp_stance_scale: 1.0）

**先关掉动态切换，用固定增益确认能稳定站立。**

修改 `robot.yaml`：

```yaml
mpc:
  kp_stance_scale: 1.0   # ← 临时改为 1.0，关掉动态切换
  kd_stance_scale: 1.0
  kp_swing_scale:  1.0   # ← 也设为 1.0
  kd_swing_scale:  1.0
```

```bash
colcon build --packages-select legged_control && source install/setup.bash
ros2 launch legged_control robot.launch.py mode:=mpc
```

**操作**：
1. 按手柄 **A 键** → 机器开始 6 秒站立 ramp
2. 站立完成后保持不动，观察 30 秒

**期望**：身体平稳，无持续振荡。

**常见问题及处理**：

| 现象 | 原因 | 处理 |
|------|------|------|
| 站立时身体前后晃动 | Iyy 偏小或 kd 不足 | 先升 kd 到 0.02，再考虑调 Iyy |
| 站立时左右摇 | Ixx 偏 | 升 kd；调 Ixx |
| 关节轻微抖动（高频） | kp 偏大 | 降 kp 到 0.3 |
| 机器无法站直（软趴趴） | kp 太小 | 升 kp 到 0.8 |
| 某条腿异常 | zero_offset 偏 | 单腿 passive 重新标定 |

**运行时调参（不用 rebuild）**：

```bash
# hip/thigh 增益
ros2 param set /motor_bus_front kp 0.5
ros2 param set /motor_bus_front kd 0.02
ros2 param set /motor_bus_rear  kp 0.5
ros2 param set /motor_bus_rear  kd 0.02

# calf 增益（约为 hip/thigh 的 1/4）
ros2 param set /motor_bus_front kp_calf 0.125
ros2 param set /motor_bus_front kd_calf 0.005
ros2 param set /motor_bus_rear  kp_calf 0.125
ros2 param set /motor_bus_rear  kd_calf 0.005

# 单关节覆盖（更高优先级）
ros2 param set /motor_bus_front kp_FR_hip 0.4
```

调好后把值写回 `robot.yaml control:` 节，rebuild 保存。

---

## 阶段 3：MPC 站立——开启动态增益

站立稳定后，恢复动态增益：

```yaml
mpc:
  kp_stance_scale: 0.25   # 支撑相低刚度，让 τ_ff 主导
  kd_stance_scale: 1.0
  kp_swing_scale:  2.0    # 摆动相高刚度，精确落脚
  kd_swing_scale:  2.0
```

```bash
colcon build --packages-select legged_control && source install/setup.bash
ros2 launch legged_control robot.launch.py mode:=mpc
```

站立后观察：
- 用手推机器 → 身体应有"柔顺感"（支撑相 kp 低，τ_ff 补偿）
- 推后松手 → 机器应自动回正（MPC 姿态控制）

如果推后机器软趴 → `kp_stance_scale` 适当升高（如 0.35）。  
如果推后机器振荡 → `kd_stance_scale` 升到 1.5。

---

## 阶段 4：慢速行走

站立稳定后，先用慢步频走直线。

```bash
# 先调慢步频（不用 rebuild，launch 参数覆盖）
ros2 launch legged_control robot.launch.py mode:=mpc
```

然后用另一个终端修改步频（或在 launch 时传参）：

```bash
ros2 param set /mpc_node gait_period 0.8    # 慢一些（默认 0.6）
ros2 param set /mpc_node step_height  0.07  # 实机地面稍抬高
ros2 param set /mpc_node stance_height 0.27
```

推手柄前进，观察：
- 四腿时序是否正确（对角腿同步，左前-右后、右前-左后）
- 落脚是否稳（不滑、不抖）

**常见行走问题**：

| 现象 | 处理 |
|------|------|
| 摆动腿落脚位置偏前/后 | 确认 `body_vel_xy` 来自 state_estimate，不是乱值 |
| 摆动腿抖动（高频） | 降 `kp_swing_scale` 到 1.5 |
| 走直线时偏航 | 检查 IMU yaw 积分漂移；`cmd_vel` yaw 轴补偿 |
| 走几步后倒 | 步频太快，先降到 gait_period 0.9 |

---

## 阶段 5：τ_ff 效果验证

```bash
# 观察支撑腿力矩前馈量级
ros2 topic echo /joint_commands | grep effort

# 期望：支撑腿 effort ≈ ±1–8 Nm，摆动腿 effort = 0.0
# 如果全为 0 → motor_command_bridge 没读到 effort（检查 msg.effort 长度）
# 如果全非零 → contact_now 判断有误
```

---

## 紧急处理

| 情况 | 操作 |
|------|------|
| 机器失控 | **按住手柄 B 键** → 立即 liedown → passive（kp=0） |
| B 键无响应 | Ctrl+C 终止 launch → 电机自动 estop fade（2 秒内 kp→0） |
| 某条腿卡死 | 立即断电，检查关节限位和 zero_offset |
| 串口断开 | motor_bus_node 自动检测 estop，kp 在 2 秒内淡出 |

---

## 调参记录表

每次实机后填写，留存：

```
日期：
分支 commit：
地面类型：
质量确认：   kg

PD 增益（最终稳定值）：
  kp / kd           :
  kp_calf / kd_calf :

MPC scale（最终值）：
  kp_stance_scale :
  kp_swing_scale  :

步态（最终值）：
  gait_period   :
  step_height   :
  stance_height :

观察到的问题：
下次待确认：
```

---

## 参考：关键话题一览

| 话题 | 含义 | 正常值 |
|------|------|--------|
| `/joint_commands` `.effort` | τ_ff（URDF 侧，Nm） | 支撑腿 ±1–10，摆动腿 0 |
| `/joint_gains` | 动态 kp/kd（24 浮点） | 支撑 kp≈0.125，摆动 kp≈1.0 |
| `/joint_states_aggregated` | 实际关节角（URDF 帧） | 站立时 thigh≈0.8，calf≈-1.5 |
| `/state_estimate` | IMU 估计（lin_vel/ang_vel/gravity） | 静止时 lin_vel≈0 |
| `/cmd_vel` | 目标速度（m/s，rad/s） | 待机时全 0 |
