---
title: teleop_node design spec
date: 2026-04-14
branch: dev/policy-node
---

# teleop_node 设计规范

## 背景

`policy_node` 以零速 `/cmd_vel` 兜底运行（从不收到消息时默认零速）。本规范描述 `teleop_node`：读取手柄输入，转换为速度指令发布到 `/cmd_vel`，供 `policy_node` 使用。

LiDAR IMU 集成（`/odin1/imu`、`/odin1/odometry`）已在 `policy_node` 中完成，不在本规范范围内。

---

## 数据流

```
手柄 (/dev/input/js*)
    ↓ joy 包 (joy_node)
    ↓ sensor_msgs/Joy → /joy
teleop_node
    ↓ geometry_msgs/Twist → /cmd_vel
policy_node (obs[6:9]: vx, vy, yaw_rate)
```

---

## 新增节点：`teleop_node`

### 职责

订阅 `/joy`，每次收到消息时：
1. 对各轴做死区处理（线性重映射）
2. 缩放到最大速度
3. 检查紧急停止按钮
4. 发布 `geometry_msgs/Twist` 到 `/cmd_vel`

### 配置来源

全部从 `robot.yaml` 的 `teleop` 段读取，无硬编码：

```yaml
teleop:
  max_vx:  1.0     # m/s，前后最大速度
  max_vy:  0.5     # m/s，左右最大速度
  max_yaw: 1.0     # rad/s，偏航最大角速度
  deadzone: 0.05   # 死区（占满量程的比例）
  axis_vx:  1      # 左摇杆垂直轴
  axis_vy:  0      # 左摇杆水平轴
  axis_yaw: 3      # 右摇杆水平轴
  invert_vx:  true
  invert_vy:  false
  invert_yaw: true
  btn_emergency_stop: -1   # -1 = 禁用；填按钮编号启用
```

### 死区 + 缩放公式

```
raw = joy.axes[axis]
if |raw| < deadzone:
    output = 0.0
else:
    output = sign(raw) × (|raw| - deadzone) / (1 - deadzone) × max_vel
if invert:
    output = -output
```

死区内线性重映射，确保死区边界处连续（无跳变）。

### 紧急停止

- `btn_emergency_stop == -1`：禁用，正常发布速度指令
- `btn_emergency_stop >= 0`：检查 `joy.buttons[btn_emergency_stop]`
  - 按住（值为 1）→ 发布零速 `Twist`
  - 松开（值为 0）→ 恢复正常速度指令

e-stop 期间每帧都发零速（不停止发布），`policy_node` 持续收到指令保持站立。

### 纯函数（可独立测试，无 ROS2 依赖）

```python
def _apply_deadzone(value: float, deadzone: float) -> float:
    """死区处理，返回重映射后的 [-1, 1] 值。"""

def _scale_axis(raw: float, deadzone: float, max_vel: float, invert: bool) -> float:
    """死区 + 缩放 + 反转，返回物理单位速度值。"""
```

---

## Launch 集成

### `mode:=policy` 自动启动

在 `robot.launch.py` 的 `policy` 分支，在现有 `joint_aggregator` 和 `policy_node` 之后追加：

```python
Node(package='joy',            executable='joy_node',    name='joy_node',    output='screen'),
Node(package='legged_control', executable='teleop_node', name='teleop_node', output='screen'),
```

### 单独运行

```bash
ros2 run joy joy_node
ros2 run legged_control teleop_node
```

---

## 新增文件

```
src/legged_control/legged_control/teleop_node.py
src/legged_control/tests/test_teleop_node.py
```

## 修改文件

| 文件 | 变更 |
|------|------|
| `src/legged_control/setup.py` | 新增 `teleop_node = legged_control.teleop_node:main` |
| `src/legged_control/package.xml` | 新增 `<exec_depend>joy</exec_depend>` |
| `src/legged_control/launch/robot.launch.py` | `policy` 分支追加 `joy_node` + `teleop_node` |

---

## 依赖

```bash
sudo apt install ros-humble-joy
```

`joy` 仅作为运行时依赖，加入 `package.xml` 的 `<exec_depend>`，不加入 `install_requires`（Python 包）。

---

## 不在本规范范围内

- 手柄标定脚本（`calibrate.py`，用于自动设置 `btn_emergency_stop`）
- 速度斜率限制（ramping）
- 多手柄支持
