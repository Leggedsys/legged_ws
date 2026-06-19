---
title: policy_node design spec
date: 2026-04-13
branch: dev/policy-node
---

# policy_node 设计规范

## 背景

`legged_control` 包已实现 `passive` 和 `stand` 两种运行模式。
本规范描述 `policy` 模式的实现：加载 TorchScript RL 策略模型，以 50 Hz 运行推理循环，驱动 12 个电机行走。

训练环境配置见 `DogEnvCfg`（Isaac Lab），完整 sim-to-real 接口见 `docs/policy_interface.md`。

---

## 新增节点

| 节点 | 文件 | 职责 |
|------|------|------|
| `joint_aggregator` | `legged_control/joint_aggregator.py` | 把 12 个独立 joint_states topic 合并成一条 `/joint_states_aggregated` |
| `policy_node` | `legged_control/policy_node.py` | 50 Hz 推理循环：构建观测 → 推理 → 发布 `/joint_commands` |

---

## 数据流

```
/fr/hip/joint_states ─┐
/fr/thigh/joint_states ┤
...（共 12 个）         ┤→ joint_aggregator → /joint_states_aggregated ─┐
/rl/calf/joint_states ─┘                                               │
                                                                        ▼
/odin1/imu ─────────────────────────────────────────────────────── policy_node → /joint_commands
/odin1/odometry ───────────────────────────────────────────────────────▲
/cmd_vel (零速兜底) ────────────────────────────────────────────────────┘
                                                                        │
                                                      motor_bus_front / motor_bus_rear 订阅
```

---

## `joint_aggregator` 节点

### 职责

订阅 12 个 `/<ns>/joint_states` topic，每当收到任意一个新消息时，立即将所有关节的最新已知状态合并发布。

### 关节顺序

严格按 `robot.yaml` 的 `joints` 列表顺序（FR_hip … RL_calf，索引 0–11），与观测向量对齐。

### 超时处理

若某关节超过 0.5 s 未收到新消息（motor_bus_node 掉线），继续发布：使用该关节上一帧已知值，打一次 warning 日志，不阻塞整体发布。

### 发布

- Topic：`/joint_states_aggregated`
- Type：`sensor_msgs/JointState`
- 坐标系：**motor frame**（原始值，不做 direction/zero_offset 转换）
- 触发方式：每次收到任意关节新消息后立即重发（无定时器）

### 实现要点

- 每个关节独立订阅回调，更新内部 `_latest[name]`（含 position、velocity、stamp）
- 回调结束后组装并发布，header.stamp 取当前时钟

---

## `policy_node` 节点

### 初始化

从 `robot.yaml` 读取：

- 所有关节的 `direction`、`zero_offset`、`default_q`、`q_min`/`q_max`
- action scale（按腿组 front/rear × 关节类型 hip/thigh/calf，共 6 个值）
- `policy.model_path`：TorchScript 模型路径
- `policy.obs_mean` / `policy.obs_std`：观测归一化参数（空列表则跳过归一化）
- `control.policy_hz`（默认 50.0）

启动时：
1. 加载 TorchScript 模型（`torch.jit.load`，map_location='cpu'）
2. 以全零输入做一次 warm-up 推理
3. 初始化 `last_action = zeros(12)`

### 订阅

| Topic | Type | 用途 |
|-------|------|------|
| `/joint_states_aggregated` | `sensor_msgs/JointState` | 关节位置、速度（motor frame） |
| `/odin1/imu` | `sensor_msgs/Imu` | 机体角速度 `angular_velocity` |
| `/odin1/odometry` | `nav_msgs/Odometry` | 机体姿态四元数（用于重力投影） |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令；从未收到则用 `[0, 0, 0]` |

### 50 Hz 定时器 tick 流程

```
1. LiDAR 超时检查
   - 若 /odin1/imu 或 /odin1/odometry 最后收到时间 > 0.5 s
   → 停止发布 /joint_commands，打 error 日志，return

2. 坐标系转换（motor frame → URDF frame）
   q_urdf  = direction × q_motor  + zero_offset
   dq_urdf = direction × dq_motor

3. 构建 45 维观测向量（顺序固定）
   obs[0:3]   = ang_vel (wx, wy, wz)               来自 /odin1/imu
   obs[3:6]   = projected_gravity (gx, gy, gz)      quat_rotate_inverse(q, [0,0,-1])
   obs[6:9]   = cmd_vel (vx, vy, yaw_rate)          来自 /cmd_vel
   obs[9:21]  = q_urdf − default_q_urdf             关节角偏差
   obs[21:33] = dq_urdf                             关节角速度
   obs[33:45] = last_action                         上一帧模型输出

4. 可选归一化
   obs_norm = (obs − mean) / (std + 1e-8)

5. TorchScript 推理
   action = model(obs_norm.unsqueeze(0)).squeeze(0)   shape: [12]

6. 动作解码（URDF frame）
   # 当前 robot.yaml 中所有关节 default_q（motor frame）= 0.0
   # 因此 default_q_urdf = direction × 0.0 + zero_offset = zero_offset
   default_q_urdf_i = zero_offset_i
   target_q_urdf_i  = clip(default_q_urdf_i + scale_i × action_i, q_min_i, q_max_i)

7. 转回 motor frame
   target_q_motor_i = (target_q_urdf_i − zero_offset_i) / direction_i

8. 发布 /joint_commands（motor frame），更新 last_action = action
```

### 重力投影

从 `/odin1/odometry` 取四元数 `(qw, qx, qy, qz)`，用 `quat_rotate_inverse` 把世界系重力向量 `[0, 0, -1]` 转到机体系：

```python
def _quat_rotate_inverse(q, v):
    # q = [w, x, y, z]
    qw, qvec = q[0], q[1:]
    a = v * (2 * qw**2 - 1)
    b = np.cross(qvec, v) * 2 * qw
    c = qvec * np.dot(qvec, v) * 2
    return a - b + c
```

与训练侧 `mdp.projected_gravity` 实现一致。

### action scale 映射

| 关节类型 | 腿组 | scale (rad/unit) |
|----------|------|-----------------|
| hip   | FR/FL（前腿） | 0.04 |
| thigh | FR/FL（前腿） | 0.05 |
| calf  | FR/FL（前腿） | 0.03 |
| hip   | RR/RL（后腿） | 0.05 |
| thigh | RR/RL（后腿） | 0.07 |
| calf  | RR/RL（后腿） | 0.04 |

scale 从 `robot.yaml` 读取（待补充字段，见下文配置变更）。

---

## `robot.yaml` 变更

将 `control` 段现有的 `action_scale: 0.25`（单一全局值）**替换**为按腿组和关节类型分开的嵌套结构：

```yaml
control:
  # ...（其余字段不变）
  action_scale:
    front:
      hip:   0.04
      thigh: 0.05
      calf:  0.03
    rear:
      hip:   0.05
      thigh: 0.07
      calf:  0.04
```

已有 `policy.model_path`、`policy.obs_mean`、`policy.obs_std` 字段保持不变。

---

## `robot.launch.py` 变更

替换现有 `mode == 'policy'` 的 `raise RuntimeError`：

```python
if mode == 'policy':
    kp = float(control['kp'])
    kd = float(control['kd'])
    motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
    return motors + [
        Node(
            package='legged_control',
            executable='joint_aggregator',
            name='joint_aggregator',
            output='screen',
        ),
        Node(
            package='legged_control',
            executable='policy_node',
            name='policy_node',
            output='screen',
        ),
    ]
```

---

## `setup.py` 变更

```python
entry_points={
    'console_scripts': [
        # ...（已有）
        'joint_aggregator = legged_control.joint_aggregator:main',
        'policy_node      = legged_control.policy_node:main',
    ],
},
```

---

## 新增文件

```
src/legged_control/legged_control/
  joint_aggregator.py
  policy_node.py

src/legged_control/tests/
  test_joint_aggregator.py
  test_policy_node.py
```

---

## 不在本分支范围内

- `teleop_node`（手柄 → `/cmd_vel`）：后续单独分支实现
- 观测归一化参数的自动导出脚本
- GPU 推理支持（当前仅 CPU）
