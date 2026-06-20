# 策略部署调试记录

## 1. 相机 TF 冲突修复

**问题**: 终端显示 `hs_mean=-0.27`（地面在机器人上方），高度扫描为负值。

**原因**: URDF 中 `camera_depth_joint`（Gazebo 仿真用）和 Realsense 驱动同时发布 `base_link → camera_depth_optical_frame` 的 TF，两个变换位置和旋转不一致，互相覆盖导致 TF 混乱。同时 `camera_joint` 的 pitch=+30°（朝上），相机看不到地面。

**修改**:
- `dog_urdf.urdf:948` — 删除 `camera_depth_joint` 和 `camera_depth_optical_frame` link（仅 Gazebo 需要，实物与 Realsense 冲突）
- `dog_urdf.urdf:948` — `camera_joint` pitch 从 `0.5236`（+30°朝上）改为 `0.6981`（+40°朝下，根据实物测量）

**效果**: TF 只有一条路径，不再竞争。hs 变为正值（地面在下方）。

---

## 2. 电机限位坐标系统一

**问题**: `robot.yaml` 中 `q_min/q_max` 是电机坐标系，`policy.yaml` 中 soft_limits 是 URDF 坐标系，两套限位坐标系不一致，容易搞混。

**原因**: 原设计 `motor_command_bridge` 在电机端裁 `q_motor`，而 `policy_node` 在 URDF 端裁 `q_target`，维护和理解不便。

**修改**:
- `robot.yaml` — 12 个关节的 `q_min/q_max` 全部从电机坐标系转换为 URDF 坐标系（用 `direction` 和 `zero_offset` 公式转换）。同类型关节值统一：hip [-0.395, 0.605]，thigh [-1.750, 1.400]，calf [-2.540, 0.270]
- `motor_command_bridge.py:89-93` — 裁剪逻辑从 `max(q_min_motor, min(q_max_motor, q_motor))` 改为 `max(q_min_urdf, min(q_max_urdf, q_urdf))`，在 URDF 端裁完再转电机坐标系

**效果**: 三套限位（robot.yaml / policy.yaml / policy_node）全部 URDF 坐标系，不再需要来回换算。

---

## 3. IMU 重力方向修正

**问题**: 终端显示 `gz=+0.98`（重力方向朝上），训练期望 `gz≈-1.0`（朝下）。

**原因**: `state_estimator_node.py:80` 对 Odin IMU 的四元数做了共轭 `(-x,-y,-z,w)`，翻转了旋转方向。Odin IMU 的 z 轴与 base_link 的 z 轴同向（都朝上），不需要翻转。

**修改**:
- `state_estimator_node.py:80` — `(-o.x, -o.y, -o.z, o.w)` 改为 `(o.x, o.y, o.z, o.w)`，去除共轭

**效果**: gz 从 +0.98 变为 -1.0，重力方向正确。

---

## 4. 高度扫描 sentinel 值修复（关键）

**问题**: 策略进入 POLICY 第一帧 raw_action 就达到 ±30~70，GRU 直接爆炸。

**原因**: `height_scan_node.py` 的 `_build_height_scan` 对 terrain_z>0 的点（地面在机器人上方、或拍到机器人自身）输出 -1.0 sentinel 值。离线验证：仅 1.5% 的格子为 -1.0，模型输出就从 0.98 跳到 34.41。训练时 hs 全在 [0.19, 0.43]，从没见过负值。

**修改**:
- `height_scan_node.py:66-68` — `grid > 0.0` 时不再输出 -1.0，改为输出 `_NOMINAL_HEIGHT`（0.30），和缺数据格统一处理
- 后续进一步改为全部输出 0.30（平地测试模式，`_build_height_scan` 直接 `return np.full(_N_CELLS, _NOMINAL_HEIGHT)`）

**效果**: GRU 第一帧不再爆炸，raw_action 降到 ±1~2。这是最关键的修复。

---

## 5. 电机保护与调试增强

**问题**: 调试过程中 raw_action 爆炸会损坏机械结构。

**修改**:
- `policy_node.py:150` — 新增 `policy_dry_run` 参数（默认 True→后改 False），True 时推理照跑但电机只收 q_default
- `policy_node.py:470-489` — POLICY 阶段增加逐帧 obs 验证守卫，异常时发 q_default 保电机
- `policy_node.py:120-121` — 新增 `_RAW_LIMIT=20.0` 和 `_RAW_FAULT_N=3`，raw_action 连续 3 帧超 ±20 切 FAULT 状态
- `teleop_node.py:132` — A 按钮按下时增加日志 `POSTURE: toggle -> standing=true/false`

**效果**: 调试期间电机始终受保护，E-stop 随时可用。

---

## 6. GRU 反馈回路修复

**问题**: 进入 POLICY 后 raw_action 从 ±1.7 在 2-3 帧内暴涨到 ±70，触发 divergence guard。

**原因**: `last_action`（策略自己的上一帧输出）喂回 GRU 时形成正反馈——模型输出大动作 → 存为 last_action → 下帧看到大 last_action → 输出更大动作。同时 velocity 反馈延迟（VIO 100-200ms）导致模型在反馈到来前已经跑了 5-10 帧。

**修改**:
- `policy_node.py:362` — `obs[36:48] = np.clip(self._last_action, -5.0, 5.0)`，截断 last_action 到训练分布内
- `policy_node.py:484-486` — 进入 POLICY 时重新加载模型 `torch.jit.load()`，重置 GRU 隐状态
- `policy_node.py:486` — 进入 POLICY 时 `_raw_high_count = 0`，`_last_action = np.zeros(12)`
- `policy_node.py` — 去掉 `_OBS_CHECKS` 中 last_action 的 ±3 验证（避免守卫每 2 帧归零打断反馈）

**效果**: GRU 不再发散，raw_action 稳定在 ±2.8 以内。

---

## 7. 速度估计改为运动学优先

**问题**: VIO 里程计速度反馈延迟 100-200ms，GRU 在反馈到来前已发散。

**原因**: `state_estimator_node.py` 原逻辑优先使用 VIO 里程计，仅在 VIO 超时（0.15s）后回退到腿部运动学。VIO 有数据时即使值不准确也不触发回退。

**修改**:
- `state_estimator_node.py:115-135` — 优先级颠倒：腿部运动学优先（零延迟，关节速度 1kHz 更新），VIO 仅作后备

**效果**: 腿一动 vel 立即非零，GRU 更快看到反馈。

---

## 8. 命令平滑

**问题**: 摇杆从 0 突然跳到 0.5~1.0 m/s，模型面对突变的速度命令产生大幅度响应。

**修改**:
- `obs_assembler.py:125-130` — cmd_vel 增加指数平滑 `(1-α)*current + α*target`，α=0.15，约 10 帧到 80%
- `robot.yaml:124` — max_vx 从 1.0 降到 0.5 m/s

**效果**: 命令缓慢增加，不刺激 GRU 剧烈反应。

---

## 9. 观测验证范围放宽

**问题**: 训练时在仿真中采集的 2σ 验证范围对实物过紧——走路时自然的身体晃动、关节速度波动都会超限。

**修改** (`policy_node.py:86-95`):
- `proj_grav`: gx [-0.04,0.18]→[-0.25,0.35], gy [-0.17,0.08]→[-0.30,0.25], gz [-1.06,-0.93]→[-1.10,-0.55]
- `base_lin_vel`: vx [-0.70,1.47]→[-1.00,2.00], vy [-0.34,0.38]→[-0.50,0.50], vz [-0.36,0.47]→[-0.50,0.60]
- `base_ang_vel`: wx [-2.25,2.16]→[-3.00,3.00], wy [-1.61,1.67]→[-2.50,2.50], wz [-1.27,1.35]→[-2.00,2.00]
- `height_mean`: hs [0.19,0.43]→[0.08,0.43]（实物离地高度低于仿真）

**效果**: 正常走路不再触发守卫，策略连续运行。

---

## 10. 重力估计改回四元数

**问题**: 之前从四元数改为加速度器计算重力，但 Madgwick 滤波后的 IMU 消息不包含加速度数据，导致 proj_grav 冻结不更新。

**修改**:
- `state_estimator_node.py:78-82` — 恢复 `projected_gravity_from_quat(*self._quat)` 计算重力
- 删除加速度器低通滤波逻辑

**效果**: proj_grav 每帧更新，站立时 gz≈-1.0。

---

## 11. TF 抗漂移

**问题**: Madgwick 滤波器默认增益 0.041 太弱，陀螺仪偏置累积导致重力方向漂移。

**修改**:
- `real.launch.py:134` — `imu_filter_madgwick` 增加 `gain: 0.1`

**效果**: 加速度计修正权重提高，漂移减小。

---

## 12. 其他

- `obs_assembler.py:135` — 增加 raw_action 接收日志（调试用，后删除）
- `policy_node.py:352,376` — 增加 `last_action_in/out` 详细日志（调试用，保留）
- `policy_node.py:393-404` — 增加 `pos_rel_all` 和 `last_a_all` 完整打印
