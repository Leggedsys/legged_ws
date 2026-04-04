# 首次上机操作手册

本文档覆盖从零开始到机器人能够站立并响应手柄指令的完整流程，适用于首次在真机上运行本系统的情况。

---

## 目录

1. [硬件清单与连接](#1-硬件清单与连接)
2. [系统环境安装](#2-系统环境安装)
3. [拉取代码并编译](#3-拉取代码并编译)
4. [硬件连接验证](#4-硬件连接验证)
5. [校准（首次必须执行）](#5-校准首次必须执行)
6. [验证电机驱动（不带策略）](#6-验证电机驱动不带策略)
7. [启动完整控制栈（stand-still 模式）](#7-启动完整控制栈stand-still-模式)
8. [手柄轴映射验证](#8-手柄轴映射验证)
9. [接入策略模型](#9-接入策略模型)
10. [常见问题排查](#10-常见问题排查)

---

## 1. 硬件清单与连接

### 必须

| 部件 | 说明 |
|------|------|
| 板载计算机 | Ubuntu 22.04，已安装 ROS2 Humble |
| Unitree GO-M8010-6 × 12 | 所有电机通过一条 RS-485 总线菊花链连接 |
| RS-485 转 USB 转接器 | 接到板载计算机，对应 `/dev/ttyUSB0` |
| USB 手柄 | PS4 / Xbox 均可，对应 `/dev/input/js0` |

### 可选（推荐）

| 部件 | 说明 |
|------|------|
| Odin1 LiDAR | 提供 IMU（角速度）和里程计（姿态四元数），策略观测量依赖这两路信号 |

### 连接顺序

1. 机器人**悬挂**在维修架上，四肢离地，准备好随时急停
2. 插入 RS-485 转 USB 线，检查 `/dev/ttyUSB0` 出现
3. 插入手柄接收器（或蓝牙配对），检查 `/dev/input/js0` 出现
4. 插入 LiDAR USB 线（如有）
5. 接通电机电源

---

## 2. 系统环境安装

### 2.1 ROS2 Humble

```bash
# 添加 ROS2 apt 源（若未安装）
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install ros-humble-desktop ros-humble-joy -y
```

### 2.2 Python 依赖

```bash
pip install ruamel.yaml colorama
```

### 2.3 设备访问权限

```bash
# 串口（重新登录后生效）
sudo usermod -a -G dialout $USER

# 手柄（重新登录后生效）
sudo usermod -a -G input $USER
```

> 添加完权限后**注销并重新登录**，再继续后续步骤。

### 2.4 LiDAR USB 访问（如有 Odin1）

```bash
sudo tee /etc/udev/rules.d/99-odin-usb.rules <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload && sudo udevadm trigger
```

---

## 3. 拉取代码并编译

```bash
cd ~/rc/legged_ws

# 安装 ROS 包依赖
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 编译全部包
colcon build

# source 工作空间
source install/setup.bash
```

> **注意：** 每次开新终端都需要重新 source，或将以下两行加入 `~/.bashrc`：
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/rc/legged_ws/install/setup.bash
> ```

---

## 4. 硬件连接验证

在执行任何运动之前，确认以下各项：

```bash
# 串口
ls -la /dev/ttyUSB0
# 预期：crw-rw---- 1 root dialout ...

# 手柄
ls -la /dev/input/js0
# 预期：crw-rw---- 1 root input ...

# 测试手柄是否有数据（摇杆时应看到数字变化，Ctrl+C 退出）
ros2 run joy joy_node &
ros2 topic echo /joy --once

# LiDAR（如有）
lsusb | grep 2207
# 预期：Bus ... ID 2207:0019 ...
```

如果串口或手柄没有权限，确认 2.3 节的组已生效（重新登录）。

---

## 5. 校准（首次必须执行）

**首次上机必须完成校准，才能进行后续任何运动测试。** 校准脚本会将电机 ID 映射、站立姿态、PD 增益写入 `robot.yaml`。

详细说明见 [calibration.md](calibration.md)，以下是最简流程：

```bash
source /opt/ros/humble/setup.bash
source ~/rc/legged_ws/install/setup.bash

python3 src/legged_control/scripts/calibrate.py
```

### 校准前准备

- 机器人**悬挂**在空中，四肢不着地
- 手柄已连接，准备好指定急停键
- 两人协作：一人操作电脑，一人在机器旁边随时准备断电

### 各阶段结果确认

| 阶段 | 完成后检查 |
|------|------------|
| Phase 0 | `robot.yaml` 中 `teleop.btn_emergency_stop` 不再是 -1 |
| Phase 1 | `robot.yaml` 中所有 12 个关节的 `motor_id` 已赋值 |
| Phase 2 | `robot.yaml` 中 `default_q` 已更新为实测站立角度 |
| Phase 3 | `robot.yaml` 中 `control.kp` 和 `kd` 已更新 |
| Phase 4 | 机器人放地面后能站稳，手动微调 kp/kd 完成 |

校准完成后，查看 `robot.yaml` 确认数值合理：

```bash
cat src/legged_control/config/robot.yaml
```

---

## 6. 验证电机驱动（不带策略）

在启动完整控制栈之前，先单独验证电机驱动是否正常工作。

**机器人仍悬挂在空中。**

```bash
# 启动 12 个电机节点（使用 robot.yaml 中的 kp/kd 保持站立姿态）
ros2 launch unitree_actuator_sdk all_motors.launch.py
```

另开一个终端检查关节状态：

```bash
source ~/rc/legged_ws/install/setup.bash

# 查看某个电机发布的关节角（应与 default_q 接近）
ros2 topic echo /fr/hip/joint_states --once

# 检查 12 个电机都在发布
ros2 topic list | grep joint_states
# 预期出现 12 条 /*/joint_states 话题
```

手动推拉关节，应感受到 PD 弹簧力（不应自由转动）。

确认后 Ctrl+C 停止电机节点，**等电机断力后再放下机器人**。

---

## 7. 启动完整控制栈（stand-still 模式）

**没有策略模型时**，policy_node 会发出零动作（所有关节保持 default_q），机器人只会站立不会行走。这是验证整个数据流的安全方式。

### 7.1 将机器人放置在平坦地面

确认四肢摆放大致对称，旁边有人看护。

### 7.2 启动全栈

```bash
# 不带 LiDAR
ros2 launch legged_control locomotion.launch.py with_lidar:=false

# 带 LiDAR（推荐，策略需要 IMU 和姿态）
ros2 launch legged_control locomotion.launch.py
```

启动后应看到以下节点全部正常运行：

```
[motor]        × 12   (/fr/hip/motor, /fr/thigh/motor, ...)
[joint_aggregator]
[joy_node]
[teleop_node]
[policy_node]   — 日志: "Policy node ready at 50.0 Hz, placeholder (stand-still)"
[watchdog_node] — 日志: "Watchdog ready. Timeout = 0.5 s."
```

### 7.3 验证数据流

另开终端：

```bash
# 聚合后的 12 关节状态（应以 ~1000 Hz 刷新）
ros2 topic hz /joint_states_aggregated

# 策略输出（应以 50 Hz 刷新，位置等于 default_q）
ros2 topic echo /joint_commands --once

# 速度指令（摇杆不动时应全为 0.0）
ros2 topic echo /cmd_vel --once
```

### 7.4 看门狗验证

在 locomotion.launch.py 运行期间，手动 kill policy_node：

```bash
ros2 node kill /policy_node
```

约 0.5 秒后，watchdog_node 终端应打印：

```
[ERROR] Policy silent for X.XX s — entering fallback (holding default_q). Restart node to clear.
```

机器人应继续保持站立（watchdog 接管 /joint_commands）。

验证完成后重启整个 launch 文件恢复正常。

---

## 8. 手柄轴映射验证

在完整栈运行期间，确认手柄轴映射与 `robot.yaml` 中的 `teleop` 配置一致。

```bash
# 实时查看速度指令
ros2 topic echo /cmd_vel
```

推动左摇杆（前后）时，`linear.x` 应变化；推动左摇杆（左右）时，`linear.y` 应变化；推动右摇杆（左右）时，`angular.z` 应变化。

如果方向相反，修改 `robot.yaml` 中对应的 `invert_*` 字段：

```yaml
teleop:
  invert_vx:  true   # 左摇杆前推 → 正值（前进）
  invert_vy:  false
  invert_yaw: true
```

如果轴序号对不上，修改 `axis_vx / axis_vy / axis_yaw`。可以用以下命令查看原始轴数据：

```bash
ros2 topic echo /joy
```

`axes` 数组的下标即为轴序号。修改完后重新 `colcon build` 使配置生效（或直接修改已安装的 share 目录下的 robot.yaml 临时验证）。

---

## 9. 接入策略模型

当你拿到训练好的 TorchScript 模型（`.pt` 文件）后，按以下步骤接入：

### 9.1 确认模型格式

策略模型应为 TorchScript 格式：

```python
import torch
model = torch.load('policy.pt', map_location='cpu')
# 输入: [1, 45] float32
# 输出: [1, 12] float32 (per-joint delta actions)
```

### 9.2 填写 robot.yaml

```yaml
policy:
  model_path: '/home/<user>/models/policy.pt'   # 绝对路径
  obs_mean: [0.0, 0.0, ...]   # 45 个值，从训练环境导出
  obs_std:  [1.0, 1.0, ...]   # 45 个值，从训练环境导出
```

观测量顺序（共 45 维）：

| 索引 | 含义 | 来源 |
|------|------|------|
| 0–2  | 速度指令 (vx, vy, yaw_rate) | 手柄 → /cmd_vel |
| 3–5  | 重力向量在机体系下 (gx, gy, gz) | LiDAR 里程计四元数 |
| 6–8  | 机体角速度 (wx, wy, wz) | LiDAR IMU |
| 9–20 | 关节角度 − default_q（12 个） | /joint_states_aggregated |
| 21–32 | 关节速度（12 个） | /joint_states_aggregated |
| 33–44 | 上一帧动作（12 个） | 策略节点内部 |

如果训练时没有做归一化，`obs_mean` 和 `obs_std` 留空即可：

```yaml
  obs_mean: []
  obs_std:  []
```

### 9.3 重新编译并启动

```bash
colcon build --packages-select legged_control
source install/setup.bash

ros2 launch legged_control locomotion.launch.py \
    model_path:=/home/<user>/models/policy.pt
```

policy_node 日志应显示：

```
[INFO] Loaded policy model: /home/<user>/models/policy.pt
[INFO] Policy node ready at 50.0 Hz, model=/home/<user>/models/policy.pt
```

### 9.4 首次带策略测试注意事项

- 机器人仍应先在悬挂状态下启动，观察肢体是否有异常抖动
- 手指放在急停键上（校准时绑定的按键）
- 确认 watchdog_node 在线（`ros2 node list` 能看到 `/watchdog_node`）
- 确认 LiDAR 在发布姿态（`ros2 topic hz /odin1/odometry`），否则重力向量为初始值 `[0,0,-1]`，策略输入不准确
- 放地面前先观察悬挂状态下的 `/joint_commands`，幅度应在合理范围内（`action_scale=0.25`，最大偏转 0.25 rad 约 14°）

---

## 10. 常见问题排查

### 电机不响应 / 串口报错

```bash
# 确认串口存在且有权限
ls -la /dev/ttyUSB0
groups | grep dialout

# 确认波特率：Unitree GO-M8010-6 默认 4Mbaud，驱动已写死，无需手动设置

# 查看电机节点日志
ros2 node list | grep motor
ros2 node info /fr/hip/motor
```

### 手柄不发布 /joy

```bash
# 确认设备存在
ls /dev/input/js0

# 手动启动 joy_node 测试
ros2 run joy joy_node
ros2 topic echo /joy
```

如果 joy_node 启动后无数据，尝试指定设备：

```bash
ros2 run joy joy_node --ros-args -p device:=/dev/input/js1
```

### 关节聚合不完整（/joint_states_aggregated 缺关节）

检查哪个电机节点没有发布：

```bash
ros2 topic list | grep joint_states
ros2 topic hz /fr/hip/joint_states   # 应 ~1000 Hz
```

缺少的电机通常是串口总线上 ID 冲突或物理连接问题。

### policy_node 启动后立即看门狗触发

策略节点崩溃时日志会打印 traceback。常见原因：

- 模型维度不匹配（输入不是 [1, 45] 或输出不是 [1, 12]）
- `torch` 未安装：`pip install torch --index-url https://download.pytorch.org/whl/cpu`
- `model_path` 路径不存在（policy_node 会打印 error 并以 stand-still 模式运行，不会崩溃）

### 机器人站立时抖动 / 振荡

kp 过高。重新运行校准脚本的 Phase 3 重新扫描，或直接编辑 `robot.yaml` 降低 `kp`，然后重启 locomotion.launch.py。

### LiDAR 不发布里程计

```bash
ros2 topic hz /odin1/odometry
ros2 topic hz /odin1/imu
```

如果话题不存在，检查 LiDAR 的 USB 连接和 udev 规则（见 2.4 节）。策略节点在没有里程计时会使用初始重力向量 `[0,0,-1]`（机器人水平时近似正确），可以临时测试，但姿态倾斜时观测量会有误差。
