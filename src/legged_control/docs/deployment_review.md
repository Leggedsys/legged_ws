# 部署框架审查记录（policy 部署链路）

> 范围：传感器 → state_estimator → obs_assembler → policy_node → motor_command_bridge → motor_bus
> 分支：`dev/policy-obs49`　创建日期：2026-06-20
> 状态图例：`[ ]` 待处理　`[~]` 待硬件确认　`[x]` 已修复

---

## 🔴 严重（上真机前必须解决）

### [~] 1. PD 增益与训练不匹配
- **位置**：`config/robot.yaml`（`kp=1, kd=0.15`） vs 训练 `dog_urdf_config.control`（`stiffness=20, damping=0.5`）
- **问题**：策略在“刚度 20 跟踪位置目标”下训练；部署给 1，机器人过软，关节跟不上目标 → 反馈进 obs 的 `joint_pos_rel` 与策略预期不符（OOD），易趴下/乱动。
- **难点**：Unitree 电机 kp/kd 单位 ≠ Isaac stiffness，且涉及 6.33 减速比；不能直接填 20。
- **行动**：先确认电机 kp 单位与 sim stiffness 的换算关系，再定值。代码目前完全没有这层换算。

### [ ] 2. 状态估计的机身线速度坐标系错误
- **位置**：`legged_control/processing/state_estimator_node.py:111-119`
- **问题**：训练 `base_lin_vel` 是机身系；VIO `_odom_lin_vel` 是世界系，`R_body = R_yaw.T` 算了却没乘 → 发布的是世界系速度。腿运动学 fallback 返回机身系 → 两路坐标系还不一致。航向一变 obs[0:3] 即错。
- **行动**：把 VIO 速度用 `R_body` 旋到机身系；统一两条路径坐标系；加单测。（纯代码 bug，可直接修）

---

## 🟠 高（正确性 / 安全）

### [~] 3. 关节正负号/偏置在两处处理，易重复或漏改
- **位置**：`robot.yaml` 的 `direction`/`zero_offset` + `policy.yaml` 的 `hip_sign_flip: [FR_hip, RR_hip]`
- **问题**：两套机制叠加，易“正反抵消”或“少翻一次”。需逐关节实测。
- **建议**：把 robot.yaml 标定到让 `/joint_states_aggregated` 直接是训练 URDF 系，`hip_sign_flip` 永久留空，消除双重修正。

### [x] 4. 电机限速 3.0 rad/s 可能掐住正常步态
- **位置**：`config/robot.yaml` `max_joint_speed`；`legged_control/real/motor_command_bridge.py`
- **已修复**：`max_joint_speed` 3.0 → **12.0** rad/s（接近 dog_urdf 硬件上限 12–14，只拦异常跳变不削正常摆腿）。

### [x] 5. 启动瞬间电机非被动
- **位置**：`policy_node._broadcast_gains` / PASSIVE 分支
- **背景**：motor_bus 增益用的是**比值机制** `cmd.kp = (当前global_kp/初始global_kp)*kp_<name>`，所以**不能**在 launch 里把 kp 设 0（会导致比值恒为 0、增益永远上不来）。因此从 policy_node 侧修。
- **已修复**：`_broadcast_gains` 现在返回是否真正下发成功（所有 param service 就绪才发）；PASSIVE 阶段**反复重试 0/0 广播直到成功**才标记完成，STANDUP 也只在增益下发成功后才进入。消除了“广播被丢弃 → 电机停在 kp=1 硬撑”的窗口。

---

## 🟡 中（鲁棒性）

### [x] 6. obs_assembler 与 policy_node 各跑独立 50Hz 定时器
- **已修复（加保护，未重构）**：没有合并两个节点，而是在 policy_node 加**新鲜度门控**——`_on_observation` 记录时间戳，新增 `_is_fresh/_inputs_usable` 纯函数；obs 超过 `_INPUT_MAX_AGE=0.1s` 视为过期。WAIT→POLICY 前要求 obs 新鲜，POLICY 中若 obs 失鲜则保持默认位姿。架构层面的“tick 内直接装配”留作后续可选优化。

### [x] 7. proj_grav 更新门限 `raw[2] < -0.1`
- **位置**：`state_estimator_node._accept_gravity`
- **已修复**：去掉 `raw[2] < -0.1` 方向门限，改为只校验是否单位向量（`0.9<|g|<1.1`）。大俯仰/翻滚时重力估计不再冻结。提取为纯函数 `_accept_gravity` 并加测试。

### [x] 8. 缺 IMU 健康 → 禁止进 POLICY 的硬保护
- **已修复**：`state_estimate` 增加第 10 维**健康标志**（1.0 可用 / 0.0 IMU 未就绪）；obs_assembler 仍只读 `[:9]`，不受影响。policy_node 订阅 `/state_estimate`，`_inputs_usable` 要求估计**新鲜且健康**才允许进/留在 POLICY，否则保持默认位姿并告警。

---

## ⚪ 小（清理）

- [x] 9. 死代码：`motor_command_bridge._max_delta` 已删。`state_estimator.R_body` **暂留**——它和 #2（线速度坐标系）绑定，等 #2 测定后一并处理（要么用它、要么删）。
- [x] 10. **更正：并非死代码。** `motor_bus_node.py:93-94` 确实用 `kp_calf/kd_calf` 作为 calf 关节的分组增益回退。无需改动。真正需要关注的是“统一 vs 分关节增益”，归入 #1（增益）一并定。
- [x] 11. **判定为有意为之，不合并。** 真机 `_decode_action` 需要 policy→yaml 重排，仿真不需要（sim 序==policy 序）；两者本质不同，强行合并只会引入耦合。保留两份，已在各自文件注明。

---

## 进度小结（2026-06-20）
- 已修复并通过测试（96 passed / 1 skipped）：**#4 #5 #6 #7 #8 #9(部分) #10 #11**。
- 待硬件确认后处理：**#1（PD 增益换算）**、**#3（正负号/偏置双重修正合并）**。
- 待 #2 测定（twist 是否世界系）后处理：**#2** 及随附的 `R_body` 去留（#9 剩余部分）。

## 处理顺序建议
1. **#2**：按方法 1/2 实测 Odin twist 坐标系 → 决定加/删 R_body。
2. **#1**：确认 Unitree 电机 kp 单位与 sim stiffness 换算 → 定增益值。
3. **#3**：把 robot.yaml 标定到训练 URDF 系，消除与 hip_sign_flip 的双重修正。
