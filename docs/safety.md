# 安全架构

电机安全保护层级，从实时硬件到策略层。

---

## 1. 电机总线层（`real/motor_bus_node.py`）

| 保护 | 机制 | 响应 |
|------|------|------|
| **数据校验** | `data.correct == True` 且 `data.motor_id` 匹配期望值 | 跳过无效帧，不发 |
| **毛刺过滤** | 相邻两帧位置差 > ±1.0 rad | 拒绝突变，保持上一帧有效值 |
| **软停止** | `/joint_commands` 停止到达 | 保持最后目标 0.5s → 2s 内 kp 线性归零（kd 保持，有阻尼） |
| **温度 > 80°C** | 每 tick 检查 `data.temp` | `ERROR` 日志 — 需立即关注 |
| **温度 > 65°C** | 每 tick 检查 | `WARN` 日志 — 继续但留意 |
| **电机故障码** | `data.merror ≠ 0`（1=过热, 2=过流, 3=过压, 4=编码器故障） | `ERROR` 日志，标注故障类型 |

## 2. 指令桥接层（`real/motor_command_bridge.py`）

| 保护 | 机制 | 响应 |
|------|------|------|
| **硬件角度限位** | URDF→Motor 转换后 clip 到 `robot.yaml` 的 `q_min`/`q_max` | 静默截断 |
| **关节速度限制** | 每关节相邻指令增量截断到 `max_joint_speed × dt`（`robot.yaml control.max_joint_speed`，默认 3.0 rad/s） | 静默钳制 |

## 3. 手柄层（`processing/teleop_node.py`）

| 保护 | 机制 | 响应 |
|------|------|------|
| **急停（B 键）** | 按住 B 键 | 发送 `posture_command=false`（触发趴下 → PASSIVE 零力矩）+ 速度指令归零 |
| **死区** | 摇杆轴值 < `deadzone`（默认 0.05）→ 输出零 | 避免摇杆漂移误入策略 |

## 4. 策略层（`policy_node.py`）

| 保护 | 机制 | 响应 |
|------|------|------|
| **状态机门控** | `PASSIVE` → 收到 `posture_command=true` → `STANDUP` → `WAIT` → 收到非零 `cmd_vel` → `POLICY` | 多步人工确认 |
| **起身斜坡** | `robot.yaml standup.ramp_duration` 秒（默认 8s）从当前位姿平滑到 `q_default_urdf` | 渐进起身，无跳变 |
| **起身完成检查** | 关节需在 `q_default_urdf` 的 `_STANDUP_TOL`（0.05 rad）内且速度稳定 | 未达标不进入 `WAIT` |
| **趴下斜坡** | `robot.yaml standup.lie_down_duration` 秒（默认 2s）平滑回到初始被动位姿 | 缓慢趴下 |
| **趴下完成检查** | 关节需靠近初始被动位姿且速度稳定 | 未达标不回到 `PASSIVE` |
| **超时保护** | 起身/趴下完成后 + 5s 硬超时 → 强制进入下一阶段 | 防止无限等待 |
| **软角度限位** | `q_target` clip 到 `policy.yaml joint_soft_limits` | 关节停留在训练安全范围 |
| **观测门禁** | 进入 `POLICY` 前，全部 49 组 obs 检查是否在训练 2σ 范围 | 拒绝进入，输出 🚨 |
| **观测异常监视** | `POLICY` 运行中每 inference tick 检查（3s 节流） | `WARN` 日志标注越界项 |
| **动作值打印** | 每 tick 打印 `raw_action` | 终端人工监控 |

## 5. 配置文件（`config/robot.yaml`）

```yaml
control:
  kp: 3.8
  kd: 0.26
  max_joint_speed: 3.0    # rad/s（URDF 帧）— 单帧关节指令增量上限

teleop:
  btn_emergency_stop: 1    # B 键
  deadzone: 0.05
```

## 6. 急停响应流程

```
用户按下 B 键（急停）
  │
  ▼
teleop_node：发送 posture_command=false + Twist 归零
  │
  ▼
policy_node：当前阶段 (POLICY/WAIT/STANDUP) → LIEDOWN
  │          平滑斜坡到初始被动位姿
  ▼
policy_node：LIEDOWN 完成 → PASSIVE
  │          广播 kp=kd=0 给 motor_bus_node
  ▼
motor_bus_node：电机零力矩（kp=kd=0）
  狗已安全
```

```
Ctrl+C（杀进程）
  │
  ▼
motor_bus_node：/joint_commands 停止到达
  │ 保持最后目标 0.5s
  ▼
motor_bus_node：2s 内 kp 线性归零（kd 保持阻尼）
  狗缓慢趴下
```

## 7. 日志级别

| 级别 | 含义 | 处理 |
|------|------|------|
| `INFO` | 正常运行（状态切换、模型加载） | 观察 |
| `WARN` | 异常检测（温度偏高、obs 越界、关节数据过期）| 运行后排查 |
| `ERROR` | 故障（电机过热、电机故障码、obs 门禁拦截）| 立即停止排查 |
| `🚨` | 进入 POLICY 前 obs 越界 | 人工确认后再试 |

## 8. 快速操作

```bash
# 紧急停止（硬件）
按住手柄 B 键

# 紧急停止（软件）
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: false}"

# 杀全部（硬件）
Ctrl+C 在终端

# 查看电机温度
# 温度在 motor_bus_node 日志中
ros2 launch legged_control robot.launch.py mode:=passive 2>&1 | grep -i "MOTOR\|warm\|OVERHEAT"
```
