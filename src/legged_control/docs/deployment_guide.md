# 部署文档：B+C 策略从仿真到实机

## 一、模型架构

| 项目 | 规格 |
|------|------|
| 架构 | 非对称 Actor-Critic + 帧堆叠 |
| Actor 输入 | 138 维 (46 × 3 帧) |
| Actor 输出 | 12 维 (关节动作) |
| 模型格式 | TorchScript (.pt)，含 4×tanh |
| 推理频率 | 50 Hz |
| 权重文件 | `weights/08_bc_mujoco_recovered_policy.pt` |

---

## 二、观测向量构建

### 单帧观测 (46 维)

```
索引      名称              维度   来源                缩放
────────────────────────────────────────────────────────────
[0:3]     角速度 (ωx,ωy,ωz)  3    IMU                 × 0.25
[3:6]     重力投影 (gx,gy,gz) 3    四元数 → 旋转       × 1.0
[6:9]     速度指令 (vx,vy,ψ)  3    手柄/cmd_vel        × [2.0, 2.0, 0.25]
[9]       高度指令 (h*)        1    手柄/固定值          × 1.0
[10:22]   关节角偏差           12   编码器              × 1.0
[22:34]   关节角速度           12   编码器              × 0.05
[34:46]   上一帧动作           12   策略内部状态         × 1.0
```

### 帧堆叠 (138 维)

```python
obs_history = np.zeros(138, dtype=np.float32)

# 每个策略步 (50Hz):
single = build_single_obs()              # 46 维
obs_history[:92] = obs_history[46:]       # 历史左移一帧
obs_history[92:] = single                 # 新帧放末尾
action = policy(torch.tensor(obs_history).unsqueeze(0))  # [1, 138] → [1, 12]
```

**首帧处理**：上电/复位时 obs_history 全部置零。

---

## 三、各观测分量详解

### 3.1 角速度 (obs[0:3])

```python
# 来源: IMU (如 Odin1 /odin1/imu)
ang_vel = [imu.angular_velocity.x, 
           imu.angular_velocity.y, 
           imu.angular_velocity.z]
obs[0:3] = ang_vel * 0.25
```

坐标系：右手系，x前 y左 z上。

### 3.2 重力投影 (obs[3:6])

```python
# 来源: 里程计四元数 (如 Odin1 /odin1/odometry)
# MuJoCo 四元数 (w,x,y,z)，ROS 四元数 (x,y,z,w) 需转换
qw, qx, qy, qz = quaternion  # (w,x,y,z) 格式

gx = 2.0 * (-qz * qx + qw * qy)
gy = -2.0 * (qz * qy + qw * qx)
gz = 1.0 - 2.0 * (qw * qw + qz * qz)

obs[3:6] = [gx, gy, gz]
```

| 姿态 | 期望值 |
|------|--------|
| 水平站立 | [0, 0, -1] |
| 前倾 15° | [0.26, 0, -0.97] |

**注意**：ROS 的四元数是 (x,y,z,w)，需转换为 (w,x,y,z) 再计算。

### 3.3 速度指令 (obs[6:9])

```python
# 来源: 手柄 /cmd_vel 或固定指令
vx = cmd_vel.linear.x    # [-1.5, 1.5] m/s
vy = cmd_vel.linear.y    # [-0.5, 0.5] m/s
yaw = cmd_vel.angular.z  # [-1.0, 1.0] rad/s

obs[6:9] = [vx * 2.0, vy * 2.0, yaw * 0.25]
```

### 3.4 高度指令 (obs[9])

```python
# 来源: 手柄轴或固定值
height_target = 0.25  # [0.15, 0.28] 范围
obs[9] = height_target
```

### 3.5 关节角偏差 (obs[10:22])

```python
# 来源: 电机编码器
# 需要从电机坐标系转到 URDF 坐标系
q_urdf = direction * q_motor + zero_offset
obs[10:22] = (q_urdf - DEFAULT_ANGLES) * 1.0
```

### 3.6 关节角速度 (obs[22:34])

```python
# 来源: 电机编码器 (速度估计)
dq_urdf = direction * dq_motor
obs[22:34] = dq_urdf * 0.05
```

### 3.7 上一帧动作 (obs[34:46])

```python
# 策略内部状态，首帧为全零
obs[34:46] = last_action  # 上一步策略的原始输出
```

---

## 四、关节顺序与坐标变换

### 4.1 训练时关节顺序 (URDF 顺序)

```
索引   关节名           default_q
0      FL_hip_joint      0.1
1      FL_thigh_joint    0.8
2      FL_calf_joint    -1.5
3      FR_hip_joint      0.1
4      FR_thigh_joint    0.8
5      FR_calf_joint    -1.5
6      RL_hip_joint      0.1
7      RL_thigh_joint    1.0
8      RL_calf_joint    -1.5
9      RR_hip_joint      0.1
10     RR_thigh_joint    1.0
11     RR_calf_joint    -1.5
```

### 4.2 部署时关节顺序 (legged_ws)

```
索引   关节名           motor_id
0      FR_hip            0
1      FR_thigh          1
2      FR_calf           2
3      FL_hip            3
4      FL_thigh          4
5      FL_calf           5
6      RR_hip            6
7      RR_thigh          7
8      RR_calf           8
9      RL_hip            9
10     RL_thigh         10
11     RL_calf          11
```

### 4.3 关节映射数组

```python
# 部署顺序 → 训练顺序 (读取观测时)
DEPLOY_TO_TRAIN = [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8]

# 训练顺序 → 部署顺序 (发送指令时)
TRAIN_TO_DEPLOY = [3, 4, 5,  0, 1, 2,  9, 10, 11,  6, 7, 8]

# 读取关节数据
obs_joint_pos = real_joint_pos[DEPLOY_TO_TRAIN]
obs_joint_vel = real_joint_vel[DEPLOY_TO_TRAIN]

# 发送动作指令
deploy_action = train_action[TRAIN_TO_DEPLOY]
```

### 4.4 URDF 坐标变换 (电机 ↔ URDF)

```python
# 电机 → URDF (读取观测时)
q_urdf = direction * q_motor + zero_offset

# URDF → 电机 (发送指令时)
q_motor = (q_urdf - zero_offset) / direction
```

各关节变换参数 (来自 robot.yaml):

| 关节 | direction | zero_offset |
|------|-----------|-------------|
| FR_hip | +1 | 0.0 |
| FR_thigh | +1 | +1.254 |
| FR_calf | -1 | -1.221 |
| FL_hip | +1 | 0.0 |
| FL_thigh | +1 | -1.254 |
| FL_calf | -1 | +1.221 |
| RR_hip | +1 | 0.0 |
| RR_thigh | +1 | -1.254 |
| RR_calf | -1 | +1.221 |
| RL_hip | +1 | 0.0 |
| RL_thigh | +1 | +1.254 |
| RL_calf | -1 | -1.221 |

---

## 五、动作执行

### 5.1 动作转换

```python
# 策略输出 → 目标关节角 (训练顺序)
action = policy(obs_tensor).squeeze(0).numpy()  # 12 维
target_urdf = action * 0.25 + DEFAULT_ANGLES     # action_scale = 0.25

# 关节限位裁剪
target_urdf = np.clip(target_urdf, Q_MIN, Q_MAX)

# 重排到部署顺序
target_deploy = target_urdf[TRAIN_TO_DEPLOY]

# URDF → 电机坐标系
target_motor = (target_deploy - ZERO_OFFSET) / DIRECTION

# 保存动作用于下一帧观测 (训练顺序)
last_action = action
```

### 5.2 PD 控制

```
τ = kp × (target_q − q_actual) + kd × (0 − dq_actual)

kp = 20.0 N·m/rad
kd = 0.5  N·m·s/rad
```

---

## 六、控制频率架构

```
策略推理 (50 Hz)    █               █               █
PD 控制 (200 Hz)    ████            ████            ████
电机通信 (1000 Hz)  ████████████████████████████████████████
                    |←── 20ms ──→|
```

```python
import time

POLICY_DT = 0.02    # 50 Hz
PD_DT = 0.005       # 200 Hz
MOTOR_DT = 0.001    # 1000 Hz

obs_history = np.zeros(138)
last_action = np.zeros(12)
target_q = DEFAULT_ANGLES.copy()
tau = np.zeros(12)

step = 0
while running:
    t0 = time.time()
    
    # 50 Hz: 策略推理
    if step % 20 == 0:
        single = build_single_obs(imu, encoders, cmd, last_action)
        obs_history[:92] = obs_history[46:]
        obs_history[92:] = single
        
        with torch.no_grad():
            action = policy(torch.tensor(obs_history).unsqueeze(0).float())
            action = action.squeeze(0).numpy()
        
        target_q = action * 0.25 + DEFAULT_ANGLES
        target_q = np.clip(target_q, Q_MIN, Q_MAX)
        target_q_deploy = convert_to_motor_frame(target_q)
        last_action = action
    
    # 200 Hz: PD 计算
    if step % 5 == 0:
        q_actual = read_joint_positions()
        dq_actual = read_joint_velocities()
        tau = 20.0 * (target_q_deploy - q_actual) + 0.5 * (0 - dq_actual)
    
    # 1000 Hz: 发送电机指令
    send_motor_commands(tau)
    
    step += 1
    elapsed = time.time() - t0
    if elapsed < MOTOR_DT:
        time.sleep(MOTOR_DT - elapsed)
```

---

## 七、完整数据流

```
┌─────────────────────────────────────────────────────────┐
│  真机传感器 (部署顺序: FR,FL,RR,RL)                      │
│    IMU → ang_vel (rad/s)                                 │
│    Odom → quat → gravity projection                     │
│    Gamepad → cmd_vel (vx, vy, yaw) + height             │
│    12× encoder → q_motor, dq_motor                      │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────────┐
│  坐标变换: q_urdf = direction * q_motor + zero_offset    │
│  关节重排: deploy → train (DEPLOY_TO_TRAIN)              │
│  观测缩放: ang_vel×0.25, cmd×[2,2,0.25], dof_vel×0.05  │
│  构建 46 维单帧 obs                                      │
│  帧堆叠: 左移 + 追加 → 138 维                            │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────────┐
│  TorchScript 推理: action = policy(obs[1,138]) → [1,12] │
│  保存 action 作为下一帧的 last_action                    │
└──────────────────────┬──────────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────────┐
│  目标角: target_urdf = default_q + 0.25 × action        │
│  限位裁剪: clip(target_urdf, q_min, q_max)               │
│  关节重排: train → deploy (TRAIN_TO_DEPLOY)              │
│  坐标反变换: q_motor = (q_urdf - offset) / direction     │
│  PD: τ = 20×(target-q) + 0.5×(0-dq)                    │
│  发送电机指令                                            │
└─────────────────────────────────────────────────────────┘
```

---

## 八、部署检查清单

- [ ] TorchScript 模型加载正常: 输入 [1,138] → 输出 [1,12]
- [ ] 关节映射正确: DEPLOY_TO_TRAIN / TRAIN_TO_DEPLOY
- [ ] URDF 坐标变换: direction + zero_offset 对齐
- [ ] 观测缩放: ang_vel×0.25, cmd×[2,2,0.25], dof_vel×0.05
- [ ] 重力投影公式正确 (注意 ROS 四元数格式转换)
- [ ] 帧堆叠: 正确左移 + 追加，复位时清零
- [ ] last_action: 首帧全零，后续保存策略原始输出
- [ ] action_scale = 0.25
- [ ] PD 增益: kp=20.0, kd=0.5
- [ ] 控制频率: 策略 50Hz, PD 200Hz, 电机 1000Hz
- [ ] 12 个 default_q 与真机校准值一致
- [ ] 12 个 q_min/q_max 与训练限位一致
- [ ] 高度指令范围: [0.15, 0.28]
- [ ] 速度指令范围: vx ≤ 1.5, vy ≤ 0.5, yaw ≤ 1.0
- [ ] 低速首测: cmd=[0.2, 0, 0] 验证方向正确
- [ ] 紧急停止按钮已配置

---

## 九、常见问题

### Q: 机器人不动 / 原地抖动
检查 action_scale 是否为 0.25，PD 增益是否正确，关节映射是否匹配。

### Q: 机器人走反方向
检查关节 direction 符号，或 DEPLOY_TO_TRAIN 映射是否颠倒。

### Q: 机器人侧翻
检查重力投影公式，特别是 ROS 四元数 (x,y,z,w) → (w,x,y,z) 的转换。

### Q: 机身前倾
检查前后腿的 zero_offset 是否正确。前后对称性依赖于正确的坐标变换。

### Q: 帧堆叠顺序
obs_history = [最旧帧 | 中间帧 | 最新帧]，新帧在末尾。错误的顺序会导致策略行为异常。
