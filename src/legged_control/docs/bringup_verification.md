# B+C 策略实机上电验证手册

适用分支：`dev/policy-bc`。按顺序逐项做，**每项过了再做下一项**。带 ⚠️ 的是"做错会摔狗/伤硬件"的关键项。

## 0. 前置

```bash
cd ~/legged_ws
git fetch && git checkout dev/policy-bc && git pull
colcon build --packages-select legged_control   # 配置/代码/模型变了必须重建
source install/setup.bash
```

- ⚠️ **狗先架空**（吊起来或垫高，四脚离地），所有加增益的步骤都在架空下做。
- ⚠️ 确认**急停**可用（手柄急停键或随时能拔电/`Ctrl-C`）。
- 速度指令首测一律低速。

参考约定：
- 关节顺序（部署/yaml 序）：`FR, FL, RR, RL`，每腿 `hip, thigh, calf`。
- default_q（URDF系）：hip 0.1；thigh 前腿(FR/FL) 0.8、后腿(RR/RL) 1.0；calf 全 -1.5。
- `q_urdf = direction × q_motor + zero_offset`。

---

## 1. Passive 模式：逐关节核对方向 / 零位 / 范围 ⚠️

目的：确认 `robot.yaml` 的 `direction` / `zero_offset` 把电机系正确映射到训练 URDF 系。**这步错了后面全错**。

```bash
ros2 launch legged_control test.launch.py    # 电机被动(kp=kd=0)，可手动掰
ros2 topic echo /joint_states_aggregated     # 看 URDF 系关节角
```

逐关节检查（用手摆动单关节）：

| 检查 | 期望 | 不对怎么办 |
|------|------|-----------|
| **方向** | 关节朝 URDF 正方向转时，`/joint_states_aggregated` 对应值**增大** | 翻 `robot.yaml` 该关节 `direction` 符号 |
| **零位** | 摆到机械/URDF 零位时读数 ≈ 0 | 调该关节 `zero_offset` |
| **左右对称** | 左右同名关节，同样的物理动作读数符号一致 | 检查左右 `direction` |

> 训练用的是对称 URDF；如果某个髋左右读数相反，可在 `policy.yaml` 的 `hip_sign_flip` 里列出该关节（当前为空 `[]`）。**优先靠 `direction` 修正，hip_sign_flip 只作兜底。**

---

## 2. ⚠️ 小腿（calf）限位复核 —— 本轮重点

背景：之前前腿 calf `q_max=-0.27`、后腿 `-0.5` 不对称会让步态偏；本轮**四腿统一改成 -0.5**。需实机确认 -0.5 这个上限是否合适。

`q_max` 是 calf **最伸直**方向的上限（URDF系，越接近 0 越直；当前 -0.5）。

1. Passive 下把每条 calf 朝**伸直**方向慢慢掰，读 `/joint_states_aggregated`：
   - 确认**机械上能安全到达 -0.5**、且 **-0.5 之前不会有干涉/打死**。
   - 若 -0.5 处已经顶到限位/线缆/结构 → 把四腿 `q_max` 一起收紧（如 -0.6），**四腿保持一致**。
   - 若 -0.5 还很宽松、机械能更直且训练需要 → 可放宽（但别超训练硬限 -0.28），**四腿一起改**。
2. 改完同步：`robot.yaml` 四个 calf 的 `q_max` 与 `policy.yaml` 的 `joint_soft_limits.calf` 上限**保持一致**（当前都 -0.5）。
3. 顺带核对 hip/thigh 四腿范围一致（当前：hip [-0.395, 0.605]、thigh [-1.75, 1.4]、calf [-2.65, -0.5]，均已对称）。

> 两道裁剪：`policy_node` 按 `policy.yaml` 软限位裁策略输出，`motor_command_bridge` 再按 `robot.yaml` q_min/q_max 兜底。约定 **软限位 ⊆ 硬件限位**，改限位时两边一起改。

---

## 3. 站立（standup）：核对 default_q

手柄按 `A`（`/posture_command=true`）→ 进 STANDUP，斜坡升到 default_q。

- 架空下观察是否到达预期站姿；`/joint_states_aggregated` 应接近 default_q（hip 0.1 / thigh 0.8或1.0 / calf -1.5）。
- 明显偏离 → default_q 坐标系或 zero_offset 有问题，回第 1 步。
- 再按 `A`（false）应平滑趴下回到被动。

---

## 4. ⚠️ 状态估计 —— 重力（本轮改动，重点验证）

本轮把重力来源改成**里程计四元数**、并去掉了之前的符号翻转补丁。必须确认符号对。

```bash
# test.launch.py 已起 state_estimator + obs_monitor + vel_viz
# 看 obs_monitor 面板的 proj_grav gx/gy/gz
```

| 姿态 | 期望 proj_grav | 不对怎么办 |
|------|---------------|-----------|
| **水平站立** | gz ≈ **-1**，gx≈gy≈0 | 若 gz≈**+1** → 里程计四元数约定不同，**告诉我在源头修**（别再加翻转） |
| **前倾 ~15°** | gx 变正、gz 往 0 靠（约 [0.26, 0, -0.97]，见指南 §3.2） | 方向反 → 轴向/符号问题，记录后找我 |
| **左右倾** | gy 随倾向变化 | — |

其它：
- **角速度 ang_vel**：转动机身，obs_monitor 的 wx/wy/wz 应跟手、量级合理（来自 IMU）。
- **线速度箭头 vel_viz**（绿）：B+C 策略**不吃线速度**（不进 obs），这里只是估计器自检——搬动狗时箭头应大致指向真实运动方向即可，不必苛求。

---

## 5. 指令核对（cmd_vel / 高度）

- 手柄推杆，`ros2 topic echo /cmd_vel`：前/后/左/右/转向**方向与符号正确**（参考 `robot.yaml` 的 `teleop` 轴配置）。走反 → 改对应 `invert_*`。
- 高度：`ros2 topic echo /height_command` 应**持续有值**（即使手柄不动，靠定时器发），范围被钳在 **[0.15, 0.28]**，LT/RT 能升降。

---

## 6. ⚠️ PD 增益台架标定（关键，决定能不能站稳走稳）

详见 `robot.yaml` 增益处注释。目标：**关节刚度 ≈ 20 N·m/rad、阻尼 ≈ 0.5**。

- SDK 增益是**转子侧**：`K_joint = kp × gear_ratio²`。理论 `kp = 20/gr²`：hip/thigh(6.33)→0.50、calf(12.66)→0.125。
- 当前手调值 `kp=1.5 / kd=0.2`（约 3× 理论），是更可信的真机量级，先用着。
- ⚠️ **确定的待办：calf 的 kp/kd 应是 hip/thigh 的 1/4**（gr² 比 = 4）。现在 `kp_calf=kp` 是相等的，需拆分。

台架测刚度（架空、单关节）：
1. 给定 kp，让关节夹住某目标角。
2. 在小腿末端挂**已知重量**（如 1 kg），量力臂长 → 力矩 τ。
3. 量该关节被压下的角度偏差 Δθ。
4. `K_joint = τ / Δθ`，对比是否 ≈ 20，按比例调 kp。**calf 单独测**，应得到约 hip/thigh 的 1/4 的 SDK 值。

---

## 7. ⚠️ 策略首跑（仍架空）

确认前 1–6 都过，再加策略。**保持架空**，先 dry-run。

```bash
# 用真正跑策略的 launch（robot.launch.py），先开 dry-run 只看不发力
ros2 launch legged_control robot.launch.py
ros2 param set /policy_node policy_dry_run true   # POLICY 阶段只推理、发 default 不发动作
```

1. 按 `A` 站起 → 进 WAIT。
2. 给一个**很小**的前进指令 `cmd=[0.2, 0, 0]` 触发进 POLICY。
3. dry-run 下看日志/`/raw_policy_action`：动作幅度合理（|raw|≤4）、不发散、obs 无 `out of range` 报警。
4. 帧堆叠：进 POLICY 时帧栈清零，前几帧含零属正常。
5. 确认无异常后，关 dry-run（`policy_dry_run false`），**仍架空**，给 `cmd=[0.2,0,0]`，看腿部动作方向/节奏是否像正常 trot、左右对称。
6. 一切正常 + 增益标定完，再考虑落地低速首走，旁边人随时急停。

异常对照（指南 §九）：原地抖→增益/action_scale/映射；走反→`direction` 或关节映射；侧翻→重力符号（第 4 步）；前倾→前后腿 zero_offset。

---

## 8. 验证记录表（建议填）

| 项 | 结果 | 备注 |
|----|------|------|
| 1 关节方向/零位/对称 | ☐ | |
| 2 calf 限位 -0.5 是否合适 | ☐ | 实际安全上限 = ___ |
| 3 default_q 站姿 | ☐ | |
| 4 重力 gz≈-1 / 前倾对 | ☐ | 水平 gz=___ |
| 5 cmd/height 方向范围 | ☐ | |
| 6 增益标定（含 calf=1/4） | ☐ | 实测 K_joint=___ |
| 7 策略 dry-run / 架空首跑 | ☐ | |

把第 2、4、6 项的实测值发我，尤其第 4 步水平站立的 gz 符号。
