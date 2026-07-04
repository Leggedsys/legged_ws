# MIT Cheetah MPC — 力矩前馈 + 动态 kp/kd 设计文档

**日期**：2026-07-05  
**分支**：dev/mpc-control  
**参考**：Di Carlo et al., "Dynamic Locomotion in the MIT Cheetah 3 Through Convex MPC", IROS 2018

---

## 背景与目标

当前 MPC 输出关节位置目标，通过 `Δq = J^T·f / K_joint` 近似将地面反力（GRF）转换为位置修正。这是一个静态近似，在动态场景（大扰动、高速）下误差放大，且固定 kp/kd 导致支撑相 PD 力矩抵消 MPC 前馈效果。

目标：还原 MIT Cheetah 标准做法——
1. GRF 直接转为关节力矩前馈（τ_ff = J^T · f），不再除以 K_joint
2. 支撑相切低 kp/kd（让 τ_ff 主导），摆动相切高 kp/kd（精确轨迹跟踪）
3. 全程不影响 RL policy 路径（policy_node 零修改）

---

## 架构

### 话题变更

| 话题 | 消息类型 | 发布者 | 说明 |
|------|---------|--------|------|
| `/joint_commands` | `JointState` | mpc_node / policy_node | effort 字段新增 τ_ff；policy 不填 → 0 |
| `/joint_gains` | `Float32MultiArray` | mpc_node 独占 | 新增话题，24 浮点：kp₀,kd₀,…,kp₁₁,kd₁₁，YAML_JOINTS 顺序 |

### 数据流

```
mpc_node
  ├─ /joint_commands (q, dq, τ_ff)          ──▶  motor_command_bridge
  └─ /joint_gains (kp₀,kd₀…kp₁₁,kd₁₁)     ──────────────────────────▶  motor_bus_node
                                                                               ▲
motor_command_bridge                                                           │
  q_motor    = direction * (q_urdf - zero_offset) * gr                        │
  dq_motor   = direction * dq_urdf * gr                                       │
  tau_motor  = direction * tau_urdf / gr   ← 新增                             │
        │ /joint_commands_motor (含 effort)                                    │
        └──────────────────────────────────────────────────────────────────────┘

motor_bus_node
  cmd.q   = q_motor
  cmd.dq  = dq_motor
  cmd.tau = tau_motor      ← 原硬编码 0，现从 /joint_commands_motor effort 读取
  cmd.kp  = kp_dynamic[name] 或 default   ← 从 /joint_gains 读取，超时回退默认
  cmd.kd  = kd_dynamic[name] 或 default
```

`/joint_gains` 直接订阅，**不经过** motor_command_bridge——kp/kd 基准值在 robot.yaml 中已是电机侧单位，scale 乘后直接可用，无需 direction/gear_ratio 换算。

### policy / passive 路径

- policy_node 不发 effort → motor_command_bridge 输出 tau_motor=0（安全）
- policy_node 不发 /joint_gains → motor_bus_node 超时（>200ms）回退默认 kp/kd
- passive 模式：launch 参数 kp_override=0 仍然覆盖，行为不变

---

## 配置（robot.yaml `mpc:` 节新增）

```yaml
mpc:
  # 动态 kp/kd 比例系数（相对各关节现有 kp/kd 基准值）
  kp_stance_scale: 0.25   # 支撑相：base_kp × 0.25（让 τ_ff 主导）
  kd_stance_scale: 1.0    # kd 不变
  kp_swing_scale:  2.0    # 摆动相：base_kp × 2.0（精确轨迹跟踪）
  kd_swing_scale:  2.0    # kd 略加大
```

使用比例系数而非绝对值，使基准 kp/kd 调整时 stance/swing 自动跟随。

**当前基准（motor 侧，已在实机验证）**：
- hip/thigh: kp=1.5, kd=0.05
- calf: kp=0.5, kd=0.02

**由此计算的运行值**：

| 相位 | hip/thigh kp | calf kp | 备注 |
|------|-------------|---------|------|
| 支撑 | 0.375 | 0.125 | ×0.25 |
| 摆动 | 3.0   | 1.0   | ×2.0  |

---

## 各模块改动

### mpc_node.py

**新增数据结构**：
```python
@dataclass
class JointCommand:
    q:   list[float]   # 12 个位置目标（YAML_JOINTS 顺序）
    dq:  list[float]   # 12 个速度前馈
    tau: list[float]   # 12 个力矩前馈（stance=J^T·f, swing=0）
    kp:  list[float]   # 12 个 kp（按相位切换）
    kd:  list[float]   # 12 个 kd
```

**力矩前馈（stance 腿）**：
```
τ_ff = J^T · f_mpc
```
去掉原来的 `/ K_joint`，直接输出 Nm。

**速度前馈（swing 腿）**：
对摆动轨迹位置做数值微分：`dq = (q_now - q_prev) / dt`。

**`_publish()` 同步发两个话题**：
- `/joint_commands`：position=q, velocity=dq, effort=τ_ff
- `/joint_gains`：data=[kp₀,kd₀,…,kp₁₁,kd₁₁]

**读取 scale**（`__init__` 里）：
```python
self._kp_stance_scale = float(mpc_cfg.get("kp_stance_scale", 1.0))
self._kd_stance_scale = float(mpc_cfg.get("kd_stance_scale", 1.0))
self._kp_swing_scale  = float(mpc_cfg.get("kp_swing_scale",  1.0))
self._kd_swing_scale  = float(mpc_cfg.get("kd_swing_scale",  1.0))
# 从 robot.yaml joints 读取各关节基准 kp/kd
self._base_kp: dict[str, float] = {j["name"]: float(j.get("kp", control_kp)) ...}
self._base_kd: dict[str, float] = ...
```

### motor_command_bridge.py

新增力矩换算（torque 与 position 的 gear_ratio 方向相反）：
```python
tau_urdf  = float(effort_map.get(name, 0.0))
tau_motor = direction * tau_urdf / gear_ratio
```
输出 JointState 的 effort 字段传至 motor_bus_node。

### motor_bus_node.py

**新增 `/joint_gains` 订阅**：
```python
self._kp_dynamic: dict[str, float | None] = {n: None for n in names}
self._kd_dynamic: dict[str, float | None] = {n: None for n in names}
self._gains_stamp: float | None = None
_GAINS_TIMEOUT = 0.2  # s
```

**tick 里**：
```python
# gains 超时回退默认
gains_fresh = self._gains_stamp and (now - self._gains_stamp) < _GAINS_TIMEOUT
kp_use = self._kp_dynamic[name] if gains_fresh else default_kp
kd_use = self._kd_dynamic[name] if gains_fresh else default_kd

cmd.kp  = kp_use
cmd.kd  = kd_use
cmd.tau = self._tau_targets.get(name, 0.0)   # 原硬编码 0
```

**新增 `_tau_targets`**：motor_bus_node 已有 `/joint_commands` 订阅（launch 中 remap 自 `/joint_commands_motor`），在 `_on_joint_cmd` 里同时读取 `msg.effort` 存入 `_tau_targets`，tick 时赋给 `cmd.tau`。

---

## 测试策略

### 单元测试（test_mpc.py 新增）

- τ_ff 方向验证：已知 GRF 输入，J^T·f 输出力矩量级不超过电机额定（约 23 Nm）
- JointCommand 结构：stance 腿 kp=scale×base，swing 腿 kp=swing_scale×base，τ 正确
- kp_scale 读取：mock yaml，验证计算值
- motor_bus_node 无 /joint_gains：kp/kd 回退默认值（policy 路径隔离测试）

### MPC Preview 验证

```bash
ros2 launch legged_control mpc_preview.launch.py
ros2 topic echo /joint_gains   # 支撑腿 kp 低，摆动腿 kp 高
ros2 topic echo /joint_commands  # effort 字段非零
```

### 干跑验证

```bash
ros2 launch legged_control robot.launch.py mode:=mpc dry_run:=true
# 检查 /tmp/motor_command_log.csv：stance τ 方向合理，swing τ=0
```

### 实机调参顺序

1. `kp_stance_scale: 1.0`（不切换）跑通站立，确认基线
2. 降至 `kp_stance_scale: 0.25`，观察支撑柔顺性
3. 升 `kp_swing_scale: 2.0`，观察落脚精度
4. 调 MPC Q 矩阵权重精调姿态

---

## 不变的部分

- `srbd_mpc.py`（求解器）：不改
- `gait_scheduler.py`：不改
- `swing_trajectory.py`：不改
- `policy_node.py`：零修改
- `robot.yaml` 现有字段：全部保留，仅在 `mpc:` 节追加
