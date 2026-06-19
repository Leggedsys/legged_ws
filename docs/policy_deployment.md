# Policy Deployment Reference

## Model

文件：`models/policy.pt`（TorchScript JIT，内置 obs normalizer）

```python
import torch
policy = torch.jit.load("policy.pt", map_location="cpu")
policy.eval()
with torch.inference_mode():
    action = policy(obs)  # obs: (1,373) float32, action: (1,12)
```

**不要用 `model_21500.pt`**——它是 PPO checkpoint，没有独立 normalizer。

正常 action 范围：训练 2σ ≈ [-3, +5]，静止/慢行时在 ±1 附近。超过 ±10 说明 obs 有维度跑出训练分布。

---

## 观测向量 (373 dims, float32)

| 索引 | 字段 | 单位 | 训练 2σ 范围 |
|------|------|------|---------------|
| 0-2 | base_lin_vel [vx,vy,vz] | m/s, yaw frame | ±1.1 |
| 3-5 | base_ang_vel [wx,wy,wz] | rad/s | ±2.0 |
| 6-8 | projected_gravity [gx,gy,gz] | 归一化, body frame | gz≈-1.0 |
| 9-11 | velocity_commands | [vx, vy, ωz] | ±1.4 |
| 12-23 | joint_pos_rel | rad, q_urdf - q_default | ±0.3 |
| 24-35 | joint_vel | rad/s | ±6 |
| 36-47 | last_action | policy 原始输出 | ±3 |
| 48-372 | height_scan | m, sensor_z - hit_z | 0.19-0.43 |

### 关节顺序 (12 维)

所有关节字段共用此顺序（policy order）：

```
[0]FL_hip  [1]FR_hip  [2]FL_thigh  [3]FR_thigh  [4]FL_calf  [5]FR_calf
[6]RL_hip  [7]RR_hip  [8]RL_thigh  [9]RR_thigh  [10]RL_calf [11]RR_calf
```

### joint_pos_rel 计算

```python
q_default = {
    "FL_hip": 0.0, "FR_hip": 0.0, "RL_hip": 0.0, "RR_hip": 0.0,
    "FL_thigh": 0.7, "FR_thigh": 0.7, "RL_thigh": 0.7, "RR_thigh": 0.7,
    "FL_calf": -1.2, "FR_calf": -1.2, "RL_calf": -1.2, "RR_calf": -1.2,
}
joint_pos_rel[i] = q_urdf[i] - q_default[i]  # 必须先减再重排到 policy order
```

### last_action

必须是 policy 的原始输出，不是 q_target：
```python
last_action = action  # model(obs) 的直接输出
# 错误: last_action = q_target - q_default
# 错误: last_action = q_target
```

### proj_gravity[2] (gz)

**训练期望 gz ≈ -1.0（站立时）。**
代码 `projected_gravity_from_quat` 返回 `R.T @ [0,0,-1]`，直立时 gz=-1.0。正确。

### height_scan

`height_scan_node` 产出，325 格 x-major。训练均值 0.306（平地时 sensor_z ≈ 0.306 above ground）。

---

## 动作解码

控制频率：**50 Hz**（仿真 dt=5ms, decimation=4）

```python
scale   = [0.15, 0.15, 0.20, 0.20, 0.15, 0.15,   # FL/FR hip/thigh/calf
           0.15, 0.15, 0.20, 0.20, 0.15, 0.15]    # RL/RR hip/thigh/calf
default = [0.0,  0.0,  0.7,  0.7, -1.2, -1.2,
           0.0,  0.0,  0.7,  0.7, -1.2, -1.2]

q_target = default + action * scale
```

### sign_flip

训练 URDF 与部署 URDF 的 hip 关节符号不一致（Isaac Lab USD 导入时丢失轴方向）：

```python
sign = [+1, -1, +1, +1, +1, +1,   # FL_hip=+1, FR_hip=-1
        -1, +1, +1, +1, +1, +1]   # RL_hip=-1, RR_hip=+1
action[FR_hip] *= -1   # policy order index 1
action[RL_hip] *= -1   # policy order index 6
```

### 安全兜底 clip（推荐）

```python
soft_lo = [-0.45,-0.45,-1.48,-1.48,-2.30,-2.30, -0.45,-0.45,-1.48,-1.48,-2.30,-2.30]
soft_hi = [+0.45,+0.45,+0.68,+0.68,-0.41,-0.41, +0.45,+0.45,+0.68,+0.68,-0.41,-0.41]
q_target = np.clip(q_target, soft_lo, soft_hi)
```

训练时 `clip_actions=null` 未限制，但真机部署必须有。

---

## Code Map

| 组件 | 文件 |
|------|------|
| 策略节点 | `legged_control/policy_node.py` |
| 状态机 | PASSIVE → STANDUP → WAIT → POLICY → LIEDOWN |
| 观测拼装 | `_assemble_obs()` + `_run_inference()` |
| 动作解码 | `_decode_action()` |
| 关节配置 | `config/robot.yaml` (direction/zero_offset/q_min/q_max) |
| 策略配置 | `config/policy.yaml` (q_default, action_scale, sign_flip, soft_limits) |
| 模型文件 | `models/policy.pt` |

## 排查

action 超过 ±10 时按优先级检查：

1. 确认加载的是 `policy.pt`，不是 checkpoint
2. `joint_pos_rel` 是否减了 `q_default`（thigh=0.7, calf=-1.2 最容易漏）
3. `last_action` 是否为原始 policy 输出，不是 q_target
4. `proj_gravity[2]` (gz) 是否接近 -1.0（直立时；+1 说明符号反了）
5. `base_lin_vel` 是否在 yaw frame
