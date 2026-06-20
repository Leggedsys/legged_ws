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

### [x] 2. VIO 线速度坐标系 — 查证后判定无需 R_body（结论：机身系）
- **依据**：驱动 `host_sdk_sample.h:1100 publishOdometry` 设 `frame_id="odom"`、`child_frame_id="odin1_base_link"`，`twist.linear` 取自 SDK `linear_velocity`。按 ROS REP-105 / `nav_msgs/Odometry` 约定，**twist 表达在 child_frame（odin1_base_link 机身系）**。话题名 `odin1/odometry` 与订阅一致（非 bug）。
- **结合**：用户已实测 Odin 系与机身**同向** → 速度已在机器人机身系，**无需 R_body**；原代码（没乘）即正确，腿运动学 fallback 也是机身系，两路一致。
- **已修复**：删除未用的 `R_yaw/R_body` 及其 import，更正 `_odom_lin_vel` 注释为 body frame，并说明依据。
- **保留确认**：厂商无文档白纸黑字写 twist 的系；最终以 yaw 实验坐实（原地转 yaw + 固定世界方向平移，看 twist 跟不跟机头）。轴向另需确认为 FLU（x前/y左/z上）以对齐训练。

---

## 🟠 高（正确性 / 安全）

### [x] 3. 关节正负号/偏置在两处处理，易重复或漏改
- **位置**：`robot.yaml` 的 `direction`/`zero_offset` + `policy.yaml` 的 `hip_sign_flip`
- **已修复**：`hip_sign_flip` 置空（`[]`），统一由 robot.yaml `direction`/`zero_offset` 把电机系映射到训练 URDF 系，消除双重修正。删除 policy_node 中未使用的 `_DEFAULT_HIP_SIGN_FLIP_POLICY_IDX`（避免误以为有默认翻转）。读取机制保留（`.get("hip_sign_flip", [])`），将来若实测某髋方向相反，仅改 yaml 即可。
- **前提/待确认**：这依赖 robot.yaml 的 `direction`/`zero_offset` 已正确映射到训练 URDF 系——上真机前在 passive 模式逐关节核对方向与零位。

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

- [x] 9. 死代码：`motor_command_bridge._max_delta` 已删；`state_estimator` 的 `R_yaw/R_body` 及其 import 已随 #2 一并删除。
- [x] 10. **更正：并非死代码。** `motor_bus_node.py:93-94` 确实用 `kp_calf/kd_calf` 作为 calf 关节的分组增益回退。无需改动。真正需要关注的是“统一 vs 分关节增益”，归入 #1（增益）一并定。
- [x] 11. **判定为有意为之，不合并。** 真机 `_decode_action` 需要 policy→yaml 重排，仿真不需要（sim 序==policy 序）；两者本质不同，强行合并只会引入耦合。保留两份，已在各自文件注明。

---

## 进度小结（2026-06-20）
- 已处理：**#2 #3 #4 #5 #6 #7 #8 #9 #10 #11**。
- 待硬件确认后处理：**#1（PD 增益换算）** —— 唯一剩余实质项。
- 上真机前的核对清单（passive 模式）：关节顺序、各关节方向/零位（#3 前提）、yaw 实验坐实 twist 为机身系且轴向 FLU（#2 收尾）。

## 处理顺序建议
1. **#1**：确认 Unitree 电机 kp 单位与 sim stiffness 换算 → 定增益值。
2. 真机 passive 核对：方向/零位（#3）、yaw + FLU（#2）。
