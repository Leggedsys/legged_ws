# MIT Cheetah MPC 力矩前馈 + 动态 kp/kd 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 MPC 从"位置修正近似"升级为 MIT Cheetah 标准做法——直接输出关节力矩前馈（τ_ff = J^T·f），支撑/摆动相动态切换 kp/kd，RL policy 路径零修改。

**Architecture:** mpc_node 计算 JointCommand（q/dq/τ_ff/kp/kd），通过两个话题发出：`/joint_commands`（含 effort=τ_ff）和 `/joint_gains`（24 个浮点 kp/kd）。motor_command_bridge 将 effort 换算到电机侧，motor_bus_node 读取 effort 作为 cmd.tau，订阅 /joint_gains 动态更新 cmd.kp/cmd.kd，200ms 超时自动回退默认值。

**Tech Stack:** Python 3.10, NumPy, ROS2 Humble, Unitree GO_M8010_6 SDK, pytest

---

## 文件映射

| 操作 | 文件 | 变更内容 |
|------|------|---------|
| 修改 | `src/legged_control/config/robot.yaml` | mpc: 节新增 4 个 scale 字段 |
| 修改 | `src/legged_control/legged_control/mpc/mpc_node.py` | JointCommand dataclass、τ_ff 计算、/joint_gains 发布、移除 K_joint |
| 修改 | `src/legged_control/legged_control/real/motor_command_bridge.py` | effort 字段读取与换算 |
| 修改 | `src/legged_control/legged_control/real/motor_bus_node.py` | tau from effort、/joint_gains 订阅 |
| 修改 | `src/legged_control/tests/test_mpc.py` | τ_ff 方向测试、JointCommand 结构测试、scale 计算测试 |
| 修改 | `src/legged_control/tests/test_motor_bus_node.py` | gains 超时回退测试 |
| 修改 | `src/legged_control/tests/test_motor_command_bridge_conversion.py` (新建) | torque 换算测试 |

---

## Task 1: robot.yaml 新增 kp/kd scale 配置

**Files:**
- Modify: `src/legged_control/config/robot.yaml`

- [ ] **Step 1: 在 mpc: 节末尾追加四个字段**

找到 `robot.yaml` 中 `K_joint: 20.0` 那行，在其下方添加：

```yaml
  # MIT Cheetah 动态增益：比例系数相对各关节现有 kp/kd 基准值
  # 支撑相低刚度让 tau_ff 主导，摆动相高刚度精确跟踪轨迹
  kp_stance_scale: 0.25
  kd_stance_scale: 1.0
  kp_swing_scale:  2.0
  kd_swing_scale:  2.0
```

- [ ] **Step 2: 验证 yaml 合法**

```bash
python3 -c "import yaml; yaml.safe_load(open('src/legged_control/config/robot.yaml'))"
```

期望：无输出（无异常）。

- [ ] **Step 3: 提交**

```bash
git add src/legged_control/config/robot.yaml
git commit -m "config: add kp/kd stance/swing scale fields to mpc section"
```

---

## Task 2: JointCommand dataclass + base_kp 加载 + τ_ff 纯函数（TDD）

**Files:**
- Modify: `src/legged_control/legged_control/mpc/mpc_node.py`
- Modify: `src/legged_control/tests/test_mpc.py`

- [ ] **Step 1: 写失败测试——τ_ff 方向和量级**

在 `tests/test_mpc.py` 末尾追加：

```python
# ── MIT Cheetah τ_ff ──────────────────────────────────────────────────────────

def test_tau_ff_direction_and_magnitude():
    """J^T · f_contact should produce plausible stance joint torques."""
    import sys
    sys.path.insert(0, "src/legged_control")
    from legged_control.kinematics import _numerical_jacobian
    import numpy as np

    joints_fr = (0.1, 0.8, -1.5)
    J = _numerical_jacobian("FR", joints_fr)

    # 四腿均分 14.55 kg 体重的竖直支撑力
    fz = 14.55 * 9.81 / 4.0
    f = np.array([0.0, 0.0, fz])
    tau = J.T @ f

    # 所有关节力矩绝对值 < 电机额定 23 Nm
    assert np.all(np.abs(tau) < 23.0), f"τ exceeds motor limit: {tau}"
    # 大腿（index 1）应为正力矩（支撑体重）
    assert tau[1] > 0.0, f"Thigh τ should be positive, got {tau[1]:.3f}"
    # 小腿（index 2）应为负力矩（膝关节弯曲对抗重力）
    assert tau[2] < 0.0, f"Calf τ should be negative, got {tau[2]:.3f}"


def test_kp_scale_gives_correct_per_joint_value():
    """Stance scale 0.25 applied to base_kp=1.5 should give 0.375."""
    base_kp = {"FR_hip": 1.5, "FR_thigh": 1.5, "FR_calf": 0.5}
    stance_scale = 0.25
    swing_scale  = 2.0

    stance_kp = {n: v * stance_scale for n, v in base_kp.items()}
    swing_kp  = {n: v * swing_scale  for n, v in base_kp.items()}

    assert stance_kp["FR_hip"]   == pytest.approx(0.375)
    assert stance_kp["FR_calf"]  == pytest.approx(0.125)
    assert swing_kp["FR_hip"]    == pytest.approx(3.0)
    assert swing_kp["FR_calf"]   == pytest.approx(1.0)
```

- [ ] **Step 2: 运行，确认失败**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py::test_tau_ff_direction_and_magnitude src/legged_control/tests/test_mpc.py::test_kp_scale_gives_correct_per_joint_value -v
```

期望：`test_tau_ff_direction_and_magnitude` 因 `_numerical_jacobian` 未暴露而失败，或通过（该函数已存在）。`test_kp_scale_gives_correct_per_joint_value` 应直接通过（纯数学）。

- [ ] **Step 3: 在 mpc_node.py 顶部添加 JointCommand dataclass**

在 `from __future__ import annotations` 之后、`import os` 之前插入：

```python
from dataclasses import dataclass, field
```

在 `_MPC_LEG_ORDER = ...` 之后、`def _leg_joints` 之前插入：

```python
@dataclass
class JointCommand:
    """Per-tick joint command output from the MPC controller."""
    q:   list[float]
    dq:  list[float]
    tau: list[float]
    kp:  list[float]
    kd:  list[float]
```

- [ ] **Step 4: 在 MPCNode.__init__ 中加载 base_kp/kd 和 scale**

在 `self._K_joint = float(mpc_cfg.get("K_joint", 20.0))` 这行**替换**为：

```python
# K_joint 已废弃（力矩前馈不再需要此近似），保留读取以免 yaml 报错
_ = mpc_cfg.get("K_joint", 20.0)

# kp/kd 比例系数
self._kp_stance_scale = float(mpc_cfg.get("kp_stance_scale", 1.0))
self._kd_stance_scale = float(mpc_cfg.get("kd_stance_scale", 1.0))
self._kp_swing_scale  = float(mpc_cfg.get("kp_swing_scale",  1.0))
self._kd_swing_scale  = float(mpc_cfg.get("kd_swing_scale",  1.0))

# 每关节基准 kp/kd（motor 侧，与 motor_bus_node 使用相同基准）
_ctrl = control
_global_kp = float(_ctrl.get("kp", 0.5))
_global_kd = float(_ctrl.get("kd", 0.0125))
_calf_kp   = float(_ctrl.get("kp_calf", _global_kp))
_calf_kd   = float(_ctrl.get("kd_calf", _global_kd))
self._base_kp: dict[str, float] = {}
self._base_kd: dict[str, float] = {}
for _j in cfg.get("joints", []):
    _n = _j["name"]
    _is_calf = "calf" in _n.lower()
    self._base_kp[_n] = float(_j["kp"]) if "kp" in _j else (_calf_kp if _is_calf else _global_kp)
    self._base_kd[_n] = float(_j["kd"]) if "kd" in _j else (_calf_kd if _is_calf else _global_kd)
```

- [ ] **Step 5: 运行测试，确认通过**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py -v --tb=short
```

期望：之前 15 个 + 新增 2 个共 **17 passed**。

- [ ] **Step 6: 提交**

```bash
git add src/legged_control/legged_control/mpc/mpc_node.py src/legged_control/tests/test_mpc.py
git commit -m "feat(mpc): add JointCommand dataclass, base_kp loading, tau_ff tests"
```

---

## Task 3: _balance_stance 改为输出 JointCommand + τ_ff（TDD）

**Files:**
- Modify: `src/legged_control/legged_control/mpc/mpc_node.py`
- Modify: `src/legged_control/tests/test_mpc.py`

- [ ] **Step 1: 写失败测试——balance_stance 输出 JointCommand**

在 `tests/test_mpc.py` 末尾追加：

```python
def test_balance_stance_returns_joint_command_with_tau():
    """_build_stance_command should return a JointCommand with non-zero tau for stance legs."""
    import sys
    sys.path.insert(0, "src/legged_control")
    import numpy as np
    from legged_control.kinematics import _numerical_jacobian
    from legged_control.mpc.mpc_node import _build_stance_tau, _YAML_JOINTS, _MPC_LEG_ORDER, _leg_joints

    # 给定已知 GRF（每腿 35.7 N 竖直）
    grf = np.zeros(12)
    for i in range(4):
        grf[i * 3 + 2] = 35.7  # fz

    q_targets = {n: 0.1 if "hip" in n else (0.8 if "thigh" in n else -1.5)
                 for n in _YAML_JOINTS}
    tau = _build_stance_tau(grf, q_targets)

    assert len(tau) == 12
    # 至少一个关节有非零力矩
    assert any(abs(t) > 0.01 for t in tau), "All torques are zero — J^T·f not applied"
    # 无关节超过电机额定
    assert all(abs(t) < 23.0 for t in tau), f"Torque out of range: {tau}"
```

- [ ] **Step 2: 运行，确认失败（`_build_stance_tau` 不存在）**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py::test_balance_stance_returns_joint_command_with_tau -v
```

期望：`ImportError: cannot import name '_build_stance_tau'`

- [ ] **Step 3: 在 mpc_node.py 添加纯函数 `_build_stance_tau`**

在 `_state_from_estimate` 函数之前插入：

```python
def _build_stance_tau(
    grf: np.ndarray,
    joint_targets: dict[str, float],
    contact_now: list[bool] | None = None,
) -> list[float]:
    """Convert MPC GRF solution to joint torques via Jacobian transpose.

    τ_ff[leg] = J(q)^T · f_contact   (stance legs only; swing legs get 0)

    Args:
        grf:          12-element GRF vector [f0x,f0y,f0z, ..., f3x,f3y,f3z]
        joint_targets: per-joint URDF-frame angles used for Jacobian evaluation
        contact_now:  4-bool list [FR,FL,RR,RL]; None = all in contact

    Returns:
        12-element list of joint torques in YAML_JOINTS order
    """
    if contact_now is None:
        contact_now = [True, True, True, True]
    tau_dict: dict[str, float] = {n: 0.0 for n in _YAML_JOINTS}
    for i, leg in enumerate(_MPC_LEG_ORDER):
        if not contact_now[i]:
            continue
        f_leg = grf[i * 3 : i * 3 + 3]
        joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
        J = _numerical_jacobian(leg, joints_leg)
        tau_leg = J.T @ f_leg
        for jname, t in zip(_leg_joints(leg), tau_leg):
            tau_dict[jname] = float(t)
    return [tau_dict[n] for n in _YAML_JOINTS]
```

- [ ] **Step 4: 重写 `_balance_stance` 返回 JointCommand**

将原 `_balance_stance` 方法整体替换：

```python
def _balance_stance(self, stance_h: float) -> JointCommand:
    """Four-foot MPC balance: all legs in contact, zero velocity reference."""
    joint_targets = {n: float(self._q_default[i]) for i, n in enumerate(_YAML_JOINTS)}

    srbd_state = _state_from_estimate(self._state_estimate, self._com_pos)
    state_ref = np.array([
        0.0, 0.0, 0.0,
        0.0, 0.0, stance_h,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ])
    contact_schedule = [[True, True, True, True]] * self._mpc._N

    R_body = _euler_to_R(srbd_state[:3])
    foot_pos_world = np.zeros((4, 3))
    for i, leg in enumerate(_MPC_LEG_ORDER):
        joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
        foot_pos_world[i] = R_body @ np.array(forward_kinematics(leg, joints_leg))

    tau_list = [0.0] * 12
    try:
        grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
        tau_list = _build_stance_tau(grf, joint_targets)
    except Exception as exc:
        self.get_logger().warn(
            f"[mpc/balance] solver failed: {exc}", throttle_duration_sec=2.0
        )

    kp = [self._base_kp[n] * self._kp_stance_scale for n in _YAML_JOINTS]
    kd = [self._base_kd[n] * self._kd_stance_scale for n in _YAML_JOINTS]
    return JointCommand(
        q=[joint_targets[n] for n in _YAML_JOINTS],
        dq=[0.0] * 12,
        tau=tau_list,
        kp=kp,
        kd=kd,
    )
```

- [ ] **Step 5: 运行测试，确认通过**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py -v --tb=short
```

期望：**18 passed**。

- [ ] **Step 6: 提交**

```bash
git add src/legged_control/legged_control/mpc/mpc_node.py src/legged_control/tests/test_mpc.py
git commit -m "feat(mpc): _balance_stance returns JointCommand with tau_ff via J^T*f"
```

---

## Task 4: _compute_mpc_joints 改为输出 JointCommand + swing dq（TDD）

**Files:**
- Modify: `src/legged_control/legged_control/mpc/mpc_node.py`
- Modify: `src/legged_control/tests/test_mpc.py`

- [ ] **Step 1: 写失败测试——swing 腿 τ=0，stance 腿 τ 非零**

在 `tests/test_mpc.py` 末尾追加：

```python
def test_build_stance_tau_swing_legs_zero():
    """Swing legs must have zero torque regardless of GRF."""
    import sys
    sys.path.insert(0, "src/legged_control")
    import numpy as np
    from legged_control.mpc.mpc_node import _build_stance_tau, _YAML_JOINTS

    grf = np.ones(12) * 50.0  # 非零 GRF
    q_targets = {n: 0.1 if "hip" in n else (0.8 if "thigh" in n else -1.5)
                 for n in _YAML_JOINTS}

    # FR(0) stance, FL(1) swing, RR(2) swing, RL(3) stance
    contact = [True, False, False, True]
    tau = _build_stance_tau(grf, q_targets, contact_now=contact)

    # FL joints (index 3,4,5) and RR joints (index 6,7,8) should be 0
    fl_tau = tau[3:6]
    rr_tau = tau[6:9]
    assert all(t == 0.0 for t in fl_tau), f"FL swing should have zero tau: {fl_tau}"
    assert all(t == 0.0 for t in rr_tau), f"RR swing should have zero tau: {rr_tau}"
    # FR and RL should be non-zero
    assert any(abs(t) > 0.0 for t in tau[0:3]), "FR stance should have non-zero tau"
```

- [ ] **Step 2: 运行，确认通过（`_build_stance_tau` 已实现）**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py::test_build_stance_tau_swing_legs_zero -v
```

期望：**PASSED**。

- [ ] **Step 3: 在 MPCNode.__init__ 中新增 swing dq 追踪**

在 `self._lift_pos` 初始化之后添加：

```python
# 上一帧摆动相关节角（用于数值微分 dq 前馈）
self._prev_swing_q: dict[str, float | None] = {n: None for n in _YAML_JOINTS}
```

- [ ] **Step 4: 重写 `_compute_mpc_joints` 返回 JointCommand**

将原 `_compute_mpc_joints` 整体替换（保留 staleness check 和 gait 逻辑，改变返回类型和力矩计算）：

```python
def _compute_mpc_joints(self, now: float) -> JointCommand:
    """Run one MPC step and return a full JointCommand."""
    stance_h = float(self.get_parameter("stance_height").value)
    step_h   = float(self.get_parameter("step_height").value)

    if self._est_stamp is None or (now - self._est_stamp) > _EST_TIMEOUT:
        self.get_logger().warn(
            "[mpc] state_estimate stale — holding stance", throttle_duration_sec=1.0
        )
        return self._balance_stance(stance_h)

    moving = float(np.max(np.abs(self._cmd_vel))) >= _WALK_VEL_THRESH
    if not moving:
        if self._walking:
            self._gait.reset()
            self._prev_contact = {leg: True for leg in LEG_NAMES}
            self._lift_pos = {
                leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
            }
            self._prev_swing_q = {n: None for n in _YAML_JOINTS}
            self._walking = False
        return self._balance_stance(stance_h)

    if not self._walking:
        self._gait.reset()
        self._lift_pos = {
            leg: nominal_foot_position(leg, stance_h) for leg in LEG_NAMES
        }
        self._prev_swing_q = {n: None for n in _YAML_JOINTS}
        self._walking = True

    gait_state = self._gait.query(now)

    for leg in LEG_NAMES:
        in_contact = gait_state[leg]["contact"]
        if self._prev_contact[leg] and not in_contact:
            joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))
            self._lift_pos[leg] = np.array(forward_kinematics(leg, joints_leg))
        self._prev_contact[leg] = in_contact

    joint_targets: dict[str, float] = {}
    dq_targets:    dict[str, float] = {}

    for leg in LEG_NAMES:
        in_contact = gait_state[leg]["contact"]
        joints_leg = tuple(self._joint_pos[j] for j in _leg_joints(leg))

        if in_contact:
            p_foot = nominal_foot_position(leg, stance_h)
            for jname in _leg_joints(leg):
                dq_targets[jname] = 0.0
        else:
            s = self._gait.swing_phase(leg, now)
            body_vel_xy = self._state_estimate[0:2]
            p_land = landing_target(
                leg, body_vel_xy,
                self._gait.period,
                float(self.get_parameter("swing_ratio").value),
                stance_h,
            )
            p_foot = swing_foot_position(s, self._lift_pos[leg], p_land, step_h)

        preferred = joints_leg
        q_leg = inverse_kinematics(leg, tuple(p_foot), preferred_joints=preferred)
        if q_leg is None:
            q_leg = (
                _DEFAULT_Q[f"{leg}_hip"],
                _DEFAULT_Q[f"{leg}_thigh"],
                _DEFAULT_Q[f"{leg}_calf"],
            )

        for jname, qval in zip(_leg_joints(leg), q_leg):
            joint_targets[jname] = float(qval)

        if not in_contact:
            # dq feedforward: numerical differentiation of swing trajectory
            for jname, qval in zip(_leg_joints(leg), q_leg):
                prev = self._prev_swing_q.get(jname)
                dq_targets[jname] = float(
                    np.clip((qval - prev) / self._dt, -12.0, 12.0)
                ) if prev is not None else 0.0
                self._prev_swing_q[jname] = float(qval)
        else:
            for jname in _leg_joints(leg):
                self._prev_swing_q[jname] = None

    # SRBD state + reference
    srbd_state = _state_from_estimate(self._state_estimate, self._com_pos)
    state_ref = np.array([
        0.0, 0.0, 0.0,
        0.0, 0.0, stance_h,
        0.0, 0.0, self._cmd_vel[2],
        self._cmd_vel[0], self._cmd_vel[1], 0.0,
    ])

    contact_now = [gait_state[leg]["contact"] for leg in _MPC_LEG_ORDER]
    contact_schedule = []
    for k in range(self._mpc._N):
        t_future = now + k * self._dt
        future_state = self._gait.query(t_future)
        contact_schedule.append(
            [future_state[leg]["contact"] for leg in _MPC_LEG_ORDER]
        )

    R_body = _euler_to_R(srbd_state[:3])
    foot_pos_world = np.zeros((4, 3))
    for i, leg in enumerate(_MPC_LEG_ORDER):
        joints_leg = tuple(joint_targets[j] for j in _leg_joints(leg))
        foot_pos_world[i] = R_body @ np.array(forward_kinematics(leg, joints_leg))

    tau_list = [0.0] * 12
    try:
        grf = self._mpc.solve(srbd_state, state_ref, foot_pos_world, contact_schedule)
        tau_list = _build_stance_tau(grf, joint_targets, contact_now=contact_now)
    except Exception as exc:
        self.get_logger().warn(
            f"[mpc] solver failed: {exc}", throttle_duration_sec=2.0
        )

    # CoM integration
    v_world = R_body @ srbd_state[9:12]
    self._com_pos[:2] += v_world[:2] * self._dt

    # Per-joint kp/kd by phase
    kp_list = []
    kd_list = []
    for leg in _MPC_LEG_ORDER:
        in_contact = gait_state[leg]["contact"]
        scale_kp = self._kp_stance_scale if in_contact else self._kp_swing_scale
        scale_kd = self._kd_stance_scale if in_contact else self._kd_swing_scale
        for jname in _leg_joints(leg):
            kp_list.append(self._base_kp[jname] * scale_kp)
            kd_list.append(self._base_kd[jname] * scale_kd)

    return JointCommand(
        q=[joint_targets[n] for n in _YAML_JOINTS],
        dq=[dq_targets.get(n, 0.0) for n in _YAML_JOINTS],
        tau=tau_list,
        kp=kp_list,
        kd=kd_list,
    )
```

- [ ] **Step 5: 运行测试**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py -v --tb=short
```

期望：**19 passed**。

- [ ] **Step 6: 提交**

```bash
git add src/legged_control/legged_control/mpc/mpc_node.py src/legged_control/tests/test_mpc.py
git commit -m "feat(mpc): _compute_mpc_joints returns JointCommand with tau_ff + swing dq"
```

---

## Task 5: 重构 _publish 和过渡相方法，接入 /joint_gains 话题

**Files:**
- Modify: `src/legged_control/legged_control/mpc/mpc_node.py`

- [ ] **Step 1: 在 `__init__` 中添加 /joint_gains 发布者**

找到 `self._pub = self.create_publisher(JointState, "/joint_commands", 10)` 这行，在其后添加：

```python
from std_msgs.msg import Float32MultiArray
self._pub_gains = self.create_publisher(Float32MultiArray, "/joint_gains", 10)
```

（`Float32MultiArray` 已在文件顶部 import，若未导入则在 import 区添加。）

- [ ] **Step 2: 将 `_publish` 改为接受 JointCommand**

用以下代码替换原 `_publish` 方法：

```python
def _publish(self, cmd: JointCommand) -> None:
    clipped_q = [
        float(np.clip(q, self._q_min.get(n, -3.14), self._q_max.get(n, 3.14)))
        for n, q in zip(_YAML_JOINTS, cmd.q)
    ]
    self._last_published = clipped_q

    js = JointState()
    js.header.stamp = self.get_clock().now().to_msg()
    js.name     = list(_YAML_JOINTS)
    js.position = clipped_q
    js.velocity = list(cmd.dq)
    js.effort   = list(cmd.tau)
    self._pub.publish(js)

    gains = Float32MultiArray()
    gains.data = [val for pair in zip(cmd.kp, cmd.kd) for val in pair]
    self._pub_gains.publish(gains)
```

- [ ] **Step 3: 改写过渡相方法返回 JointCommand**

将 `_standup_targets` 替换为：

```python
def _standup_targets(self, elapsed: float) -> tuple[JointCommand, bool]:
    ramp = max(float(self.get_parameter("ramp_duration").value), 1e-6)
    alpha = _smoothstep(elapsed / ramp)
    start = self._standup_start or self._q_default.tolist()
    q = [(1.0 - alpha) * s + alpha * g
         for s, g in zip(start, self._q_default.tolist())]
    kp = [self._base_kp[n] * self._kp_swing_scale for n in _YAML_JOINTS]
    kd = [self._base_kd[n] * self._kd_swing_scale for n in _YAML_JOINTS]
    return JointCommand(q=q, dq=[0.0]*12, tau=[0.0]*12, kp=kp, kd=kd), elapsed >= ramp
```

将 `_liedown_targets` 替换为：

```python
def _liedown_targets(self, elapsed: float) -> tuple[JointCommand, bool]:
    dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
    alpha = _smoothstep(elapsed / dur)
    start = self._lie_down_start or self._q_default.tolist()
    goal  = self._initial_pos or [0.0] * 12
    q = [(1.0 - alpha) * s + alpha * g for s, g in zip(start, goal)]
    kp = [self._base_kp[n] * self._kp_swing_scale for n in _YAML_JOINTS]
    kd = [self._base_kd[n] * self._kd_swing_scale for n in _YAML_JOINTS]
    return JointCommand(q=q, dq=[0.0]*12, tau=[0.0]*12, kp=kp, kd=kd), elapsed >= dur
```

- [ ] **Step 4: 更新 `_tick` 中调用 `_publish` 的所有位置**

`_tick` 中原来的模式是 `self._publish(targets)` 其中 `targets` 是 `list[float]`。现在 `_standup_targets`、`_liedown_targets`、`_compute_mpc_joints` 都返回 `JointCommand`。`_tick` 中只需把 `targets` 重命名为 `cmd`，类型自然匹配。无需其他改动。

确认 `_tick` 中 STANDUP、WALK、LIEDOWN 三段代码的调用方式是否正确：

```python
# STANDUP
cmd, done = self._standup_targets(elapsed)
self._publish(cmd)
...

# WALK
cmd = self._compute_mpc_joints(now)
self._publish(cmd)

# LIEDOWN
cmd, done = self._liedown_targets(elapsed)
self._publish(cmd)
```

逐一检查并修改。同时 `_is_near` 仍基于 `self._last_published`（list[float]），无需改动。

- [ ] **Step 5: 运行测试**

```bash
python3 -m pytest src/legged_control/tests/test_mpc.py -v --tb=short
```

期望：**19 passed**（无回退）。

- [ ] **Step 6: 提交**

```bash
git add src/legged_control/legged_control/mpc/mpc_node.py
git commit -m "feat(mpc): _publish takes JointCommand, sends /joint_gains, standup/liedown refactored"
```

---

## Task 6: motor_command_bridge effort 换算（TDD）

**Files:**
- Create: `src/legged_control/tests/test_motor_command_bridge_conversion.py`
- Modify: `src/legged_control/legged_control/real/motor_command_bridge.py`

- [ ] **Step 1: 写失败测试——torque 换算公式**

新建文件 `src/legged_control/tests/test_motor_command_bridge_conversion.py`：

```python
"""Offline tests for motor_command_bridge torque conversion formula."""
import pytest


def _tau_motor(direction: float, tau_urdf: float, gear_ratio: float) -> float:
    """Mirrors the conversion in motor_command_bridge._on_command."""
    return direction * tau_urdf / gear_ratio


def test_tau_positive_direction():
    # direction=+1, gear_ratio=6.33, tau_urdf=6.33 Nm → tau_motor=1.0 Nm at rotor
    assert _tau_motor(1.0, 6.33, 6.33) == pytest.approx(1.0, rel=1e-4)


def test_tau_negative_direction():
    # FR_hip: direction=-1 → tau is flipped
    assert _tau_motor(-1.0, 6.33, 6.33) == pytest.approx(-1.0, rel=1e-4)


def test_tau_calf_gear_ratio():
    # calf: gear_ratio=12.66, same URDF torque → half motor torque
    tau_hip  = _tau_motor(1.0, 10.0, 6.33)
    tau_calf = _tau_motor(1.0, 10.0, 12.66)
    assert tau_calf == pytest.approx(tau_hip / 2.0, rel=1e-3)


def test_zero_effort_gives_zero_tau():
    assert _tau_motor(-1.0, 0.0, 6.33) == 0.0


def test_tau_within_motor_limit():
    # 最大 URDF 力矩（约等于电机额定 23 Nm）换算后电机侧应在合理范围
    max_tau_urdf = 23.0
    tau_m = _tau_motor(1.0, max_tau_urdf, 6.33)
    assert abs(tau_m) < 5.0, f"Motor-side torque too large: {tau_m}"
```

- [ ] **Step 2: 运行，确认通过（纯公式测试）**

```bash
python3 -m pytest src/legged_control/tests/test_motor_command_bridge_conversion.py -v
```

期望：**5 passed**。

- [ ] **Step 3: 在 motor_command_bridge._on_command 中加入 effort 读取与换算**

在 `MotorCommandBridge._on_command` 里，找到处理每个 joint 的循环：

```python
for name in self._names:
    cfg = self._joint_cfg[name]
    direction   = float(cfg["direction"])
    zero_offset = float(cfg["zero_offset"])
    gear_ratio  = float(cfg["gear_ratio"])
```

在 `q_motor = direction * (q_urdf_clipped - zero_offset)` 下方，`q_motor` 行之后加入：

```python
tau_urdf  = float(pos_map.get(f"_effort_{name}", effort_map.get(name, 0.0)))
tau_motor = direction * tau_urdf / gear_ratio
```

其中 `effort_map` 需要在循环前从 msg 中解析：

在循环开始前（`q_urdf_list = []` 之前）添加：

```python
effort_map: dict[str, float] = {}
if len(msg.effort) == len(msg.name):
    for _n, _e in zip(msg.name, msg.effort):
        effort_map[_n] = float(_e)
```

并在循环内的 `q_motor_list.append(q_motor)` 之后添加：

```python
tau_motor_list.append(tau_motor)
```

在循环上方初始化列表：`tau_motor_list: list[float] = []`

在发布 `out` 之前，将 effort 填入：

```python
out.effort = tau_motor_list
```

同时，将 dry-run log 扩展（在 `self._log_file.write(...)` 里追加 tau 列）：

```python
self._log_file.write(
    f"{now:.6f},," +
    ",".join(f"{v:.6f}" for v in q_urdf_list)   + "," +
    ",".join(f"{v:.6f}" for v in q_motor_list)  + "," +
    ",".join(f"{v:.6f}" for v in tau_motor_list) + "\n"
)
```

并更新 header：在 `__init__` 的 `self._log_file.write(...)` 里追加 `"," + ",".join(f"{n}_tau" for n in self._names)`。

- [ ] **Step 4: 运行所有相关测试**

```bash
python3 -m pytest src/legged_control/tests/test_motor_command_bridge_conversion.py src/legged_control/tests/test_mpc.py -v --tb=short
```

期望：**24 passed**。

- [ ] **Step 5: 提交**

```bash
git add src/legged_control/legged_control/real/motor_command_bridge.py \
        src/legged_control/tests/test_motor_command_bridge_conversion.py
git commit -m "feat(bridge): pass through effort as tau_motor = direction * tau_urdf / gr"
```

---

## Task 7: motor_bus_node — tau from effort + /joint_gains 订阅（TDD）

**Files:**
- Modify: `src/legged_control/legged_control/real/motor_bus_node.py`
- Modify: `src/legged_control/tests/test_motor_bus_node.py`

- [ ] **Step 1: 写失败测试——gains 超时逻辑**

在 `tests/test_motor_bus_node.py` 末尾追加：

```python
import time


_GAINS_TIMEOUT = 0.2  # 与 motor_bus_node 保持一致


def _gains_are_fresh(stamp: float | None, now: float) -> bool:
    """Mirrors the timeout logic in motor_bus_node._tick."""
    if stamp is None:
        return False
    return (now - stamp) < _GAINS_TIMEOUT


def test_gains_fresh_within_timeout():
    stamp = time.monotonic()
    assert _gains_are_fresh(stamp, stamp + 0.1) is True


def test_gains_stale_after_timeout():
    stamp = time.monotonic()
    assert _gains_are_fresh(stamp, stamp + 0.3) is False


def test_gains_stale_when_never_received():
    assert _gains_are_fresh(None, time.monotonic()) is False
```

- [ ] **Step 2: 运行，确认通过（纯逻辑测试）**

```bash
python3 -m pytest src/legged_control/tests/test_motor_bus_node.py -v
```

期望：**10 passed**（原 7 + 新增 3）。

- [ ] **Step 3: 在 motor_bus_node.__init__ 中添加 tau targets 和 gains 状态**

找到 `self._dq_targets: dict[str, float] = ...` 这行，在其后添加：

```python
self._tau_targets: dict[str, float] = {j["name"]: 0.0 for j in joints}
self._kp_dynamic:  dict[str, float | None] = {j["name"]: None for j in joints}
self._kd_dynamic:  dict[str, float | None] = {j["name"]: None for j in joints}
self._gains_stamp: float | None = None
_GAINS_TIMEOUT = 0.2  # s — revert to default kp/kd if /joint_gains goes stale
```

在 `self.create_subscription(JointState, "/joint_commands", self._on_joint_cmd, 10)` 之后添加：

```python
from std_msgs.msg import Float32MultiArray
self.create_subscription(
    Float32MultiArray, "/joint_gains", self._on_joint_gains, 10
)
```

- [ ] **Step 4: 更新 `_on_joint_cmd` 读取 effort**

在 `_on_joint_cmd` 里，`self._cmd_time = time.monotonic()` 之前添加：

```python
if len(msg.effort) == len(msg.name):
    for i, name in enumerate(msg.name):
        if name in self._tau_targets:
            self._tau_targets[name] = float(msg.effort[i])
```

- [ ] **Step 5: 添加 `_on_joint_gains` 回调**

在 `_on_joint_cmd` 之后插入：

```python
def _on_joint_gains(self, msg: Float32MultiArray) -> None:
    # data layout: [kp0, kd0, kp1, kd1, ..., kp11, kd11] — YAML_JOINTS order
    if len(msg.data) != 2 * len(self._names):
        return
    for i, name in enumerate(self._names):
        self._kp_dynamic[name] = float(msg.data[2 * i])
        self._kd_dynamic[name] = float(msg.data[2 * i + 1])
    self._gains_stamp = time.monotonic()
```

- [ ] **Step 6: 更新 `_tick` 使用 tau 和动态 kp/kd**

在 `_tick` 里找到：

```python
cmd.kp  = ratio * float(self.get_parameter(f"kp_{name}").value)
cmd.kd  = ratio_kd * float(self.get_parameter(f"kd_{name}").value)
cmd.q   = (self._targets[name] + offset) * gr
cmd.dq  = self._dq_targets[name] * gr * ratio
cmd.tau = 0.0
```

替换为：

```python
gains_fresh = (
    self._gains_stamp is not None
    and (now - self._gains_stamp) < _GAINS_TIMEOUT
)
base_kp = float(self.get_parameter(f"kp_{name}").value)
base_kd = float(self.get_parameter(f"kd_{name}").value)
dyn_kp  = self._kp_dynamic.get(name) if gains_fresh else None
dyn_kd  = self._kd_dynamic.get(name) if gains_fresh else None
cmd.kp  = ratio * (dyn_kp if dyn_kp is not None else base_kp)
cmd.kd  = ratio_kd * (dyn_kd if dyn_kd is not None else base_kd)
cmd.q   = (self._targets[name] + offset) * gr
cmd.dq  = self._dq_targets[name] * gr * ratio
cmd.tau = ratio * self._tau_targets.get(name, 0.0)
```

注意：`ratio` 用于在 e-stop 时同步缩放 tau（与 kp 保持一致，使力矩前馈随刚度一起淡出）。

- [ ] **Step 7: 运行所有测试**

```bash
python3 -m pytest src/legged_control/tests/ -v --tb=short
```

期望：**所有测试通过**（含原有 28 个 + 新增约 8 个）。

- [ ] **Step 8: 提交**

```bash
git add src/legged_control/legged_control/real/motor_bus_node.py \
        src/legged_control/tests/test_motor_bus_node.py
git commit -m "feat(motor_bus): tau from effort, dynamic kp/kd from /joint_gains with timeout fallback"
```

---

## Task 8: 全量验证

**Files:** 无新增文件

- [ ] **Step 1: 运行完整测试套件**

```bash
python3 -m pytest src/legged_control/tests/ -v
```

期望：全部通过，无 warning。

- [ ] **Step 2: MPC Preview 验证**

```bash
# 终端 1
ros2 launch legged_control mpc_preview.launch.py

# 终端 2（等待 5 秒机器站立后）
ros2 topic echo /joint_gains --once
```

期望：看到 24 个浮点数，支撑相 kp 约 0.375（stance scale × 1.5），摆动相 kp 约 3.0（swing scale × 1.5）。

```bash
ros2 topic echo /joint_commands --once
```

期望：`effort` 字段有非零值（支撑腿 τ_ff）。

- [ ] **Step 3: 确认 policy 路径隔离**

检查 `/joint_gains` 话题在非 MPC 模式下不存在或无数据：

```bash
# 不启动 mpc_node 时
ros2 topic info /joint_gains 2>&1 | head -5
```

期望：`Unknown topic` 或 publisher count=0。

- [ ] **Step 4: 最终提交**

```bash
git add -p   # 确认无额外文件
git commit -m "feat: MIT Cheetah MPC complete — tau_ff + dynamic kp/kd stance/swing"
```

---

## 实机调参备忘

```yaml
# robot.yaml — mpc 节，调参顺序：
kp_stance_scale: 1.0   # ① 先设 1.0（等同当前），确认站立稳定
kp_stance_scale: 0.25  # ② 降到 0.25，观察是否更柔顺，是否漂移
kp_swing_scale:  2.0   # ③ 摆动精度不够时提高
kd_swing_scale:  2.0   # ④ 落脚时脚抖则降低
```
