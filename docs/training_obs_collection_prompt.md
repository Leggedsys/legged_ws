# 训练侧 obs 收集脚本 — 提示词

> 把下面整段作为提示词交给训练侧（IsaacGym/legged_gym 等）的 AI 或工程师，
> 用来编写一个与部署侧**完全一致**的 observation 收集脚本，便于 sim2real obs 逐项对比。

---

## 任务

在训练环境里写一个脚本，在策略 rollout 时把**每一步喂给 actor 网络的 observation**逐帧写入 CSV，
格式与部署侧 `obs_log_node` 完全一致，以便我们对比训练侧 vs 部署侧的 obs 分布，定位 sim2real 差异。

## obs 向量规范（单帧，dim = 46，float32）

最终对整帧做 `clip(obs, -100, 100)`。各分量及缩放：

| 索引 | 内容 | scale |
|------|------|-------|
| 0:3 | base_ang_vel（体坐标角速度 rad/s） | ×0.25 |
| 3:6 | projected_gravity（重力在体坐标的投影，单位向量） | ×1.0 |
| 6:9 | cmd = (vx, vy, yaw_rate) | ×(2.0, 2.0, 0.25) |
| 9 | height_cmd（站高指令） | ×1.0（不缩放） |
| 10:22 | (q − q_default)（关节位置相对默认姿态） | ×1.0 |
| 22:34 | dof_vel（关节速度） | ×0.05 |
| 34:46 | last_action（上一步策略**原始**输出 a_{t-1}） | raw（不缩放） |

## 关节顺序（三个 12 维块 qpos/qvel/action 都用这个序）

URDF / IsaacGym DOF 声明序，按腿顺排（每条腿 hip→thigh→calf）：

```
FL_hip, FL_thigh, FL_calf,
FR_hip, FR_thigh, FR_calf,
RL_hip, RL_thigh, RL_calf,
RR_hip, RR_thigh, RR_calf
```

## 必须确认一致的关键约定

1. **不含 base_lin_vel**：actor obs 是非对称的，线速度仅作为 critic 的特权输入。
   actor obs 维度必须是 **46**，不含线速度。若训练侧 actor obs 含线速度，请明确指出。
2. **帧堆叠**：单帧 46 维，推理时堆叠 **3 帧 → 138**，顺序 `[最旧, 中, 最新]`（最新帧在末尾）。
3. **action_scale = 0.25**：`q_target = q_default + action × 0.25`（解码约定，须与部署一致）。
4. **符号翻转**：部署侧对部分 hip 关节的 `joint_pos_rel` 和 `joint_vel` 做了硬件方向符号翻转；
   **训练侧不做翻转**。对比时以训练侧（无翻转）为基准。
5. 确认 `obs_scales`（ang_vel=0.25, lin_vel=2.0, dof_pos=1.0, dof_vel=0.05, commands=[2,2,0.25]）
   与训练 config 完全一致——这是最常见的 obs 偏差来源。

## 使用标准化固定命令（不用遥控、不用随机采样）

为了让对比只反映 sim2real 差异，两侧都用**同一条写死的标准命令**，以训练侧定义为准：

- **训练侧**：rollout 时**不要用环境的随机命令课程**，把命令直接覆盖成下面这条固定值，
  整段 rollout 保持恒定。
- **部署侧**：**不接手柄**，用固定命令注入节点（`fixed_cmd_node`）发布同一条命令。

标准命令（两侧必须完全一致）：

| 命令 | 值 |
|------|-----|
| cmd_vx | 0.4 m/s（前进） |
| cmd_vy | 0.0 |
| cmd_wz | 0.0 |
| height_cmd | 0.25 m |

注意：这里写的是**物理量原值**；进 obs 时再按规范乘 scale（vx,vy ×2.0，wz ×0.25，height 不缩放）。
仿真控制频率须与部署一致（50Hz）。若要改命令值，两侧同步改。

## CSV 输出格式


每帧一行，表头与部署侧一致：

```
t, ang_vel_x, ang_vel_y, ang_vel_z, grav_x, grav_y, grav_z,
cmd_vx, cmd_vy, cmd_wz, height_cmd,
qpos_FL_hip, qpos_FL_thigh, qpos_FL_calf, qpos_FR_hip, ... qpos_RR_calf,
qvel_FL_hip, ... qvel_RR_calf,
act_FL_hip,  ... act_RR_calf
```

（qpos/qvel/act 各 12 列，关节按上面 URDF DOF 序展开。）

## 交付

- 一个可直接跑的脚本（指明依赖与运行方式），命令固定为上面的标准命令、整段 rollout 恒定。
- 跑一段（≥30s）输出 `obs_train.csv`（表头与部署侧一致）。
- 同时给出每一维的统计（mean / std / min / max），方便和部署侧 `obs.csv` 逐维 diff。
