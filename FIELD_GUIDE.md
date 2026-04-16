# 现场部署流程

四足机器人从上电到 policy 行走的完整操作手册。

---

## 出发前确认

- [ ] `models/policy.pt` 已放入 workspace
- [ ] `robot.yaml → policy.model_path` 已填写
- [ ] `robot.yaml → standup.ramp_duration` 已设置（建议首次用 10s）
- [ ] 已 build：`colcon build --packages-select legged_control && source install/setup.bash`
- [ ] gamepad 已配对
- [ ] LiDAR USB 线已带

---

## Step 0 — 环境准备

```bash
source /opt/ros/humble/setup.bash
source /home/grayerd/Desktop/Projects/rc/legged_ws/install/setup.bash
```

确认串口：
```bash
ls /dev/ttyUSB*
# 应看到 /dev/ttyUSB0 和 /dev/ttyUSB1
```

如串口权限报错：
```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

---

## Step 1 — 电机验证（passive 模式）

**机器人姿态：** 趴平，自然放置，不需要扶。

```bash
ros2 launch legged_control robot.launch.py
```

**操作：** 用手逐一拨动每条腿的三个关节，观察终端打印的读数变化。

```
[passive]  FR  hip=  x.xxx  thigh=  x.xxx  calf= -x.xxx
           FL  hip=  x.xxx  thigh=  x.xxx  calf= -x.xxx
           RR  hip= -x.xxx  thigh=  x.xxx  calf= -x.xxx
           RL  hip=  x.xxx  thigh=  x.xxx  calf= -x.xxx
```

| 通过 | 处理 |
|------|------|
| 12 个关节读数均随手动运动变化 | 进入 Step 2 |
| 某关节读数始终不动 | 检查该腿串口线 / 电机 ID / 电源 |
| 串口打开失败 | 检查 `/dev/ttyUSB*` 是否存在，权限是否正确 |

> Ctrl+C 停止，进入 Step 2。

---

## Step 2 — PD 调参（stand 模式）

**机器人姿态：** 手动摆到接近站姿，旁边有人扶住。

> stand 模式输出目标位置 = 0（上电位置），上电后直接保持当前姿态。

```bash
ros2 launch legged_control robot.launch.py mode:=stand
```

**调参顺序（无需重启，runtime 生效）：**

```bash
# 第一阶段：找最小可用 kp（每次等待 10 秒观察）
ros2 param set /stand_node kp 5.0
ros2 param set /stand_node kp 10.0
ros2 param set /stand_node kp 15.0
ros2 param set /stand_node kp 20.0

# 同步调 kd 消除抖动
ros2 param set /stand_node kd 0.5
ros2 param set /stand_node kd 1.0
```

| 现象 | 判断 | 操作 |
|------|------|------|
| 垮掉 / 持续下沉 | kp 不够 | 加大 kp |
| 高频抖动 | kd 太小 | 加大 kd |
| 慢速摇摆 | kd 太大 | 减小 kd |
| 小腿卡顿感 | 连杆静摩擦，正常 | 不是 PD 问题，继续 |
| 静止 30s + 轻推能回位 | **通过** | 记录 kp/kd |

**找到后写入 robot.yaml 保存：**
```yaml
control:
  kp: <找到的值>
  kd: <找到的值>
```

> Ctrl+C 停止，进入 Step 3。

---

## Step 3 — 起身测试（standup 模式）

**机器人姿态：** 趴平，上电（电机零点标定在趴姿）。

```bash
ros2 launch legged_control robot.launch.py mode:=standup
```

**观察终端进度条（全程约 10s）：**
```
[standup] ready — hold 2.0s then ramp 10.0s to default_q
[standup] RAMP started — rising to default_q
[standup] [████░░░░░░░░░░░░░░░░]  20%
[standup] [████████░░░░░░░░░░░░]  40%
...
[standup] STAND reached — holding at default_q
```

**同时观察 passive_monitor_node 的实时角度**，确认各关节值趋近 default_q。

| 通过 | 处理 |
|------|------|
| 全程平滑，站立后稳定保持 | 进入 Step 4 |
| 起身过程有振荡 | `ramp_duration` 加大，或调小 kp |
| 到位后漂移 | kp 不够，回 Step 2 调高 |
| 某条腿明显落后 | 检查该腿 motor_id |
| 需要更慢的起身速度 | `ros2 param set /standup_node ramp_duration 15.0` |

> Ctrl+C 停止，进入 Step 4。

---

## Step 4 — Policy 悬空测试

**目的：** 验证 IMU 坐标系、传感器对接、推理流程，腿部悬空无风险。

**机器人姿态：** 悬空挂起（腿部自由悬挂，不接触地面）。

**终端 1：启动 LiDAR**
```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

等待 LiDAR 正常输出（看到 `/odin1/imu` 和 `/odin1/odometry` 有数据）：
```bash
ros2 topic hz /odin1/imu        # 应约 200 Hz
ros2 topic hz /odin1/odometry   # 应约 50 Hz
```

**终端 2：启动 policy**
```bash
ros2 launch legged_control robot.launch.py mode:=policy
```

**观察三段日志依次出现：**
```
[policy_node] standup complete — holding default_q, waiting for sensors
[policy_node] sensors ready — starting inference
```

### Step 4a — IMU 坐标系验证（重要）

在 WAIT 阶段（机器人悬空静止，处于 default_q 姿态），观察 policy_monitor 面板：

```
[policy] model=policy.pt  imu=OK  odom=OK  joints=OK  cmd=--  clip=0/12
         cmd_vel  vx=  0.00  vy=  0.00  yaw=  0.00
         ang_vel  wx=  0.00  wy=  0.00  wz=  0.00    ← 静止时应接近 0
         gravity gx=  0.00  gy=  0.00  gz= -1.00    ← 站立时应接近 [0,0,-1]
         |q-default|  FL= 0.000  FR= 0.000  RL= 0.000  RR= 0.000
```

| 检查项 | 预期值 | 偏差处理 |
|--------|--------|----------|
| `gz` | ≈ −1.0 | 记录实际值，拍照，事后修正坐标系 |
| `gx`, `gy` | ≈ 0.0 | 同上 |
| `ang_vel` | ≈ 全 0 | 若有明显偏移，LiDAR IMU 零偏问题 |

> 如果 gravity 明显不对，**不要继续**，先记录下实际读数，事后加坐标系修正。

### Step 4b — 推理验证

gravity 正常后，等进入 RUN 阶段，连接 gamepad 测试零速度：

```
- 不推摇杆（零速度指令）
- 观察腿部有轻微步态节律，但不乱动
- clip=0/12（无关节截断）
- |q-default| 各腿 < 0.15 rad
```

| 通过 | 处理 |
|------|------|
| 三段日志依次出现，clip=0 | 进入 Step 5 |
| 停在 WAIT，sensors=STALE | LiDAR 未就绪，检查 topic 频率 |
| clip > 0 持续出现 | action_scale 太大，robot.yaml 各项减半 |
| policy 推理报错 | 检查模型路径 / obs 维度 (45,) |
| ang_vel 乱跳 | LiDAR IMU 数据质量问题 |

> Ctrl+C 停止，进入 Step 5。

---

## Step 5 — Policy 落地测试

**机器人姿态：** 放到地面，旁边有人准备随时托住。

重复 Step 4 的启动流程，等进入 RUN 后：

**阶段 5a：零速度站立**
```
- 不推摇杆
- 观察 30 秒，确认能站稳
- 轻推躯干，能恢复
```

**阶段 5b：慢速行走**
```
- 推左摇杆前方约 20%（约 0.2 m/s）
- 观察步态是否稳定
- 逐步增大速度
```

| 现象 | 处理 |
|------|------|
| 零速度能站稳 | 继续测行走 |
| 站立时整体偏软易倒 | kp 加大，回 Step 2 |
| 行走时某条腿动作反向 | 检查该腿 `direction` 参数 |
| 行走步态异常（腿打架）| action_scale 整体减小 20% |
| 高速时不稳但低速可以 | 正常，先验证低速 |

---

## 快速参数调整参考

```bash
# PD 增益（stand/standup 模式 runtime 生效）
ros2 param set /stand_node kp 20.0
ros2 param set /stand_node kd 1.0

# 起身速度
ros2 param set /standup_node ramp_duration 12.0

# 查看 policy 输出
ros2 topic echo /joint_commands --once

# 查看传感器频率
ros2 topic hz /odin1/imu
ros2 topic hz /odin1/odometry
ros2 topic hz /joint_states_aggregated
```

**robot.yaml 关键参数位置：**

| 参数 | 作用 | 生效方式 |
|------|------|----------|
| `control.kp` / `kd` | 全局 PD 增益 | 重启 或 runtime param set |
| `standup.ramp_duration` | 起身时长（秒） | 重启 或 runtime param set |
| `control.action_scale` | policy 动作幅度 | 重启（需 rebuild） |
| `policy.model_path` | 模型文件路径 | 重启（需 rebuild） |
| `teleop.max_vx/vy/yaw` | gamepad 速度上限 | 重启（需 rebuild） |

---

## 急停

任意终端：**Ctrl+C**

motor_bus_node 检测到 `/joint_commands` 停止后执行渐进归零：

```
指令停止 → 保持当前位置 0.5s → 2s 内 kp 线性降到 0（kd 保留，阻尼下落）→ 完全无力矩
```

终端会打印：
```
[estop] /joint_commands lost — fading kp to 0 over 2.0s
[estop] kp=0 — motors passive
```

> 2.5s 的窗口足够旁边的人托住机器人。不要依赖这个时间自行离开。
