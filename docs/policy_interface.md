# 策略模型接口规范

本文档描述部署系统对 RL 策略模型的完整要求，是训练环境与真机之间的契约。  
训练时须严格对齐此处列出的所有参数，否则 sim-to-real 迁移将失败。

---

## 模型格式

| 项目 | 要求 |
|------|------|
| 格式 | TorchScript（`torch.jit.save` 导出） |
| 输入 | `[1, 45]` float32 |
| 输出 | `[1, 12]` float32 |
| 推理频率 | 50 Hz（由 `policy_node` 定时器驱动） |

导出示例：

```python
scripted = torch.jit.script(policy)   # 或 torch.jit.trace
torch.jit.save(scripted, 'policy.pt')

# 验证
m = torch.jit.load('policy.pt', map_location='cpu')
dummy = torch.zeros(1, 45)
out = m(dummy)
assert out.shape == (1, 12), out.shape
```

---

## 观测向量（45 维，顺序固定）

```
obs[0:3]   速度指令        (vx, vy, yaw_rate)         单位: m/s, m/s, rad/s
obs[3:6]   重力向量·机体系  (gx, gy, gz)               无量纲，模长≈1
obs[6:9]   机体角速度       (wx, wy, wz)               单位: rad/s
obs[9:21]  关节角偏差       q_i − default_q_i  (i=0..11)  单位: rad
obs[21:33] 关节角速度       dq_i               (i=0..11)  单位: rad/s
obs[33:45] 上一帧动作       a_{t-1}_i          (i=0..11)  无量纲（训练空间）
```

### 速度指令（obs[0:3]）

来源：手柄通过 `teleop_node` 发布到 `/cmd_vel`，再由 `policy_node` 读取。

| 分量 | 真机范围 | 说明 |
|------|----------|------|
| vx   | [−1.0, 1.0] m/s | 前后，正为前进 |
| vy   | [−0.5, 0.5] m/s | 左右，正为左移 |
| yaw_rate | [−1.0, 1.0] rad/s | 偏航，正为左转（右手系） |

**训练时**：在上述范围内随机采样或按课程采样，与真机范围保持一致。

### 重力向量（obs[3:6]）

来源：Odin1 LiDAR 的里程计四元数，经被动旋转变换到机体坐标系：

```
g_body = R^{-1} · [0, 0, -1]^T
```

实现：`policy_node._quat_rotate_inverse`，等价公式：

```
a = v * (2*qw² - 1)
b = cross(q_vec, v) * 2*qw
c = q_vec * dot(q_vec, v) * 2
g_body = a - b + c
```

| 姿态 | 期望值 |
|------|--------|
| 水平站立 | `[0, 0, −1]` |
| 前倾 15° | `gz ≈ −0.97, gx ≈ +0.26` |

**训练时**：按相同公式从仿真器四元数计算，坐标系定义须一致（x 前 y 左 z 上）。

### 机体角速度（obs[6:9]）

来源：`/odin1/imu`，直接取 `angular_velocity.{x,y,z}`，单位 rad/s。

**训练时**：直接读取仿真器 base link 角速度，无需额外变换。

### 关节角偏差（obs[9:21]）

`q_i - default_q_i`，其中 `default_q` 为校准后写入 `robot.yaml` 的站立姿态角。

关节顺序（固定，索引 0–11）：

| 索引 | 关节 | default_q (rad) | q_min | q_max |
|------|------|-----------------|-------|-------|
| 0  | FR_hip   |  0.0 | −1.047 | +1.047 |
| 1  | FR_thigh |  0.8 | −1.571 | +3.927 |
| 2  | FR_calf  | −1.5 | −2.723 | −0.837 |
| 3  | FL_hip   |  0.0 | −1.047 | +1.047 |
| 4  | FL_thigh |  0.8 | −1.571 | +3.927 |
| 5  | FL_calf  | −1.5 | −2.723 | −0.837 |
| 6  | RR_hip   |  0.0 | −1.047 | +1.047 |
| 7  | RR_thigh |  0.8 | −1.571 | +3.927 |
| 8  | RR_calf  | −1.5 | −2.723 | −0.837 |
| 9  | RL_hip   |  0.0 | −1.047 | +1.047 |
| 10 | RL_thigh |  0.8 | −1.571 | +3.927 |
| 11 | RL_calf  | −1.5 | −2.723 | −0.837 |

> `default_q` 的值会随真机校准结果微调。训练时可使用上表中的初始值；若真机校准后偏差超过 0.1 rad，建议以实测值重新训练或 fine-tune。

**训练时**：仿真关节角减去相同 default_q 即可。关节顺序必须与上表一致。

### 关节角速度（obs[21:33]）

来源：各电机节点发布的 `/*/joint_states`，经 `joint_aggregator` 聚合为 `/joint_states_aggregated`，顺序与上表相同，单位 rad/s。

**训练时**：直接读取仿真关节速度。注意仿真器可能存在速度噪声，与真机电机编码器噪声特性不同；训练时可适当添加白噪声（±0.01 rad/s 量级）提升鲁棒性。

### 上一帧动作（obs[33:45]）

存储 `policy_node` 上一次的输出（训练空间的原始值，未经 action_scale 缩放）。首帧为全零。

**训练时**：须在仿真环境中同样维护 `last_action` 状态，首帧置零。

---

## 动作向量（12 维）

策略输出的 12 个值是各关节相对于 default_q 的**无量纲偏差**，部署时按如下方式转换为目标角：

```
target_q_i = clip(default_q_i + action_scale × a_i,  q_min_i,  q_max_i)
```

当前参数（来自 `robot.yaml`）：

```yaml
control:
  action_scale: 0.25   # rad per unit action
  kp: 20.0             # N·m/rad
  kd:  0.5             # N·m·s/rad
```

电机层执行 PD 控制（1000 Hz）：

```
τ = kp × (target_q − q) + kd × (0 − dq)
```

**训练时关键约束：**

- `action_scale = 0.25`：动作范围 [−1, 1] 对应关节偏转 [−0.25, +0.25] rad（约 ±14°）
- 仿真中须使用完全相同的 `action_scale`、`kp`、`kd`、`q_min/q_max`
- 若仿真器不支持电机级 PD，需在仿真侧以相同参数手动实现

---

## 控制时序

```
电机驱动 (1000 Hz)  ████████████████████████████████████
策略输出  (50 Hz)   █   █   █   █   █   █   █   █   █
策略延迟            |← ~5 ms 计算 →|
```

- 电机在两次策略输出之间**保持上一帧 target_q**，以 1000 Hz PD 伺服
- 真机单步延迟约 5–10 ms（推理 + ROS 通信）
- **训练时**：建议在仿真环境中加入 1–2 个控制步（20–40 ms）的随机动作延迟，增强对延迟的鲁棒性

---

## 观测归一化

如训练时对观测量做了归一化（常见于 RSL-RL / legged_gym），部署时需将 mean 和 std 写入 `robot.yaml`：

```yaml
policy:
  model_path: '/path/to/policy.pt'
  obs_mean: [v0, v1, ..., v44]   # 45 个 float
  obs_std:  [s0, s1, ..., s44]   # 45 个 float，须 > 0
```

部署系统执行：`obs_normalized = (obs_raw - mean) / (std + 1e-8)`

若训练时未做归一化，留空即可：

```yaml
  obs_mean: []
  obs_std:  []
```

从 legged_gym / RSL-RL 导出 mean/std 的方法：

```python
# 训练结束后，从 env.obs_buf 统计（或从 runner 的 normalizer 中读取）
import torch
mean = env_alg.obs_normalizer.running_mean.cpu().tolist()
std  = env_alg.obs_normalizer.running_var.sqrt().cpu().tolist()
```

---

## Sim-to-Real 注意事项

### 坐标系

系统使用右手系，x 前、y 左、z 上。仿真器须保持一致，否则重力向量和角速度符号会反。

### 传感器噪声

| 信号 | 真机特性 |
|------|----------|
| 关节角 | 编码器分辨率约 0.001 rad，噪声很小 |
| 关节速度 | 电机内部差分估计，高速时噪声约 ±0.05 rad/s |
| 角速度 (IMU) | LiDAR 内置 IMU，零偏约 ±0.01 rad/s，快速旋转时有轻微延迟 |
| 重力向量 | 由里程计四元数推算，动态运动时存在约 10–30 ms 滤波延迟 |

训练时建议对各信号加入对应量级的白噪声（Domain Randomization）。

### 关节限位

部署层在 `target_q = clip(...)` 处做了硬限位。训练时也应在奖励函数中惩罚接近限位的动作，避免真机频繁触碰软限。

### 速度指令范围

手柄发出的速度指令经过死区处理和线性缩放，真机最大值为 `max_vx=1.0, max_vy=0.5, max_yaw=1.0`。训练时命令采样范围不应超出此区间，超出部分真机无法复现。

### 地面摩擦

真机测试地面通常为瓷砖或地胶（摩擦系数约 0.5–0.8）。若训练时使用高摩擦地面，真机可能打滑；建议 Domain Randomization 覆盖 μ ∈ [0.4, 1.0]。

---

## 与 robot.yaml 的同步检查清单

训练完成、导出模型前，确认以下参数与 `robot.yaml` **完全一致**：

- [ ] `action_scale = 0.25`
- [ ] `kp = 20.0`，`kd = 0.5`
- [ ] 12 个关节的 `default_q`（核对校准后的实测值）
- [ ] 12 个关节的 `q_min` / `q_max`
- [ ] 关节顺序：FR_hip … RL_calf（索引 0–11）
- [ ] 速度指令范围 vx ≤ 1.0，vy ≤ 0.5，yaw ≤ 1.0
- [ ] 观测量顺序和维数恰好为 45
- [ ] 动作维数恰好为 12
- [ ] 如有归一化，mean/std 已从训练代码中导出并填入 `policy.obs_mean/obs_std`
