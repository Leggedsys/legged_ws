# 校准脚本使用手册

`calibrate.py` 是一个交互式初始化脚本，引导你完成从安全悬挂状态到地面验证的全套机器狗初始化流程，并将测量结果自动写入 `robot.yaml`。

---

## 前置条件

| 项目 | 要求 |
|------|------|
| 操作环境 | 板载计算机，已接好12个电机和USB手柄 |
| ROS2 | Humble，已 source |
| Python 包 | `ruamel.yaml`、`colorama`（见下方安装） |
| 串口权限 | 用户已加入 `dialout` 组 |

安装 Python 依赖：
```bash
pip install ruamel.yaml colorama
```

串口权限（仅需执行一次，需重新登录生效）：
```bash
sudo usermod -a -G dialout $USER
```

---

## 运行方式

```bash
# source 环境
source /opt/ros/humble/setup.bash
source ~/rc/legged_ws/install/setup.bash

# 运行脚本（使用安装目录下的 robot.yaml）
python3 src/legged_control/scripts/calibrate.py

# 指定自定义配置文件
python3 src/legged_control/scripts/calibrate.py --config /path/to/robot.yaml
```

脚本会顺序执行四个阶段，每个阶段开始前都会等待你确认，中途按 **Ctrl+C** 或手柄急停键可以随时暂停。

---

## 阶段说明

### Phase 0 — 环境检查 + 手柄绑定

自动运行，无需手动操作。

检查项目：
- 电机串口（`/dev/ttyUSB0`）是否存在
- ROS2 环境是否已 source
- `robot.yaml` 是否可写
- Python 依赖是否安装

手柄绑定：
1. 检测到 `/dev/input/js0` 后，提示按下你想用作**急停**的按键
2. 再按一次同一个键确认
3. 按键编号写入 `robot.yaml`，此后所有阶段的提示行都会显示急停键名称

若未检测到手柄，脚本进入**仅键盘模式**（Ctrl+C 是唯一急停方式）。

---

### Phase 2 — 悬挂状态 · 默认站立姿态采样

**前提：仍悬挂在空中，电机零力矩。**

流程：
1. 手动将四条腿摆到大致的站立姿态
2. 按 Enter，脚本连续采样 0.5 秒（50 Hz），取均值
3. 显示采样结果表格
4. 确认写入 `[y]` 或重新采样 `[n]`

精度要求不高，作为基准值，后续地面验证阶段可以继续微调。

**输出：** `robot.yaml` 中每个关节的 `default_q` 字段更新完毕。

---

### Phase 3 — 悬挂状态 · PD 增益粗调

**前提：仍悬挂在空中。**

分两步：先调 `kp`，再调 `kd`。

#### kp 扫描

从 `kp=5` 开始，每步加 5，上限 `kp=40`：
1. 施加当前 kp，监测 2 秒
2. 若检测到震荡（速度方向反转 >3 次/秒）→ 自动退回上一档，停止扫描
3. 若稳定 → 询问「继续升高？`[y/n]`」

扫描结束后，推荐值 = 0.7 × 上限稳定档，可直接回车接受或手动输入。

#### kd 调整

以固定 kp 对 `FR_thigh` 发送 0.1 rad 阶跃信号，测量峰值超调量，自动推荐 kd：

| 超调率 | 建议 |
|--------|------|
| > 100% 步长 | kd 增大 50% |
| < 20% 步长 | kd 减小 30% |
| 其余 | kd 不变 |

同样可回车接受或手动输入，上限 `kd=2.0`。

**输出：** `robot.yaml` 中 `control.kp`、`control.kd` 更新完毕。

---

### Phase 4 — 地面验证 + 微调

**前提：机器狗放置在平坦地面，旁边有安全员。**

启动完整控制栈（使用刚写入的 kp/kd），实时打印观测量：

```
cmd_vel   : vx=+0.00 vy=+0.00 yaw=+0.00
gravity   : x=+0.01 y=-0.02 z=-0.99  (flat→~0,0,-1)
ang_vel   : x=+0.00 y=+0.01 z=+0.00
joint_pos : FR_hip=+0.01  FR_thigh=+0.79  FR_calf=-1.50  FL_hip=+0.00
joint_vel : FR_hip=+0.00  FR_thigh=+0.00  FR_calf=+0.00  FL_hip=+0.00
```

按 **Enter** 停止实时显示，进入增益微调：

| 输入 | 效果 |
|------|------|
| `+` | kp 或 kd 增加一档（`+2` / `+0.1`） |
| `-` | kp 或 kd 减少一档 |
| Enter | 跳过，保持当前值 |

每次调整后自动监测 2 秒，若检测到震荡则自动回退到上一组值。

**输出：** `robot.yaml` 中 `control.kp`、`control.kd` 更新为最终值，脚本退出。

---

## 急停机制

| 方式 | 触发条件 | 效果 |
|------|----------|------|
| 手柄按键 | Phase 0 绑定的按键（上升沿） | 所有电机立即零力矩，暂停执行 |
| Ctrl+C | 任意时刻 | 所有电机零力矩，脚本退出 |

暂停后显示提示：
```
Paused. Continue? [y/n]:
```
- `y` → 恢复执行
- `n` → 退出脚本

---

## 增益参数安全限制

| 参数 | 硬上限 | 超限处理 |
|------|--------|----------|
| `kp` | 40 | 拒绝继续升高 / 输入自动截断 |
| `kd` | 2.0 | 输入自动截断 |
| 震荡检测 | 速度符号反转 >3次/秒 | 自动退回上一档 |

---

## 无硬件演示模式

在没有 ROS 或电机的机器上，可以用 stub 脚本体验完整交互 UI：

```bash
cd src/legged_control

# Phase 2：default_q 采样流程
python3 demo_calibration.py --phase 2

# Phase 3：kp sweep + kd step response
python3 demo_calibration.py --phase 3

# Phase 4：live obs + 增益微调
python3 demo_calibration.py --phase 4
```

所有电机命令被 stub 拦截，仅打印 `[stub]` 提示，其余 UI、提示、配置读写完全真实运行。

---

## 输出文件

脚本在 `robot.yaml` 中原地更新以下字段（注释保留）：

| 字段 | 阶段 |
|------|------|
| `joints[*].default_q` | Phase 2 |
| `control.kp` | Phase 3 / Phase 4 |
| `control.kd` | Phase 3 / Phase 4 |
| `teleop.btn_emergency_stop` | Phase 0 |
