# 位控使用说明

本文档对应当前分支的 `position_control` 开发版，以及两条仿真链：

- `position_control_sim.launch.py`：RViz 运动学仿真
- `gazebo_position_control.launch.py`：Gazebo 物理仿真骨架

## 1. 特性范围

当前位控链路：

- 输入：`/cmd_vel`（`linear.x/y`=平移速度，`angular.z`=偏航速率，`linear.z`=机身高度变化速率）
- 控制：解析步态 + 解析 IK
- 输出：`/joint_commands`

当前版本：

- 不依赖 IMU
- 不依赖里程计
- 不依赖雷达
- 支持前进、侧移、转弯的基础步态
- **支持实时机身高度调节**：手柄 RT 升高，LT 降低，范围 0.20～0.35 m
- 已带安全限幅、关节目标变化率限制、急停和 fault 安全回退

这意味着即使你的 IMU 挂在雷达上，也可以先不接 IMU 做位控开发。

## 2. 配置文件

实机位控默认读：

- `src/legged_control/config/robot.yaml`

RViz / Gazebo 仿真默认读：

- `src/legged_control/config/robot_sim.yaml`

`robot_sim.yaml` 是仿真专用配置，已经单独调整了：

- 默认站姿
- gait 对接姿态
- 更保守的速度限制

不要直接把仿真专用默认姿态当成实机姿态覆盖回 `robot.yaml`。

## 3. 构建

```bash
source /opt/ros/humble/setup.zsh
cd /home/grayerd/Desktop/Projects/rc/legged_ws
colcon build --packages-select legged_control
source install/setup.zsh
```

## 4. RViz 运动学仿真

启动：

```bash
ros2 launch legged_control position_control_sim.launch.py
```

用途：

- 检查默认站姿
- 检查关节方向
- 检查步态相位
- 检查前进/转弯脚端轨迹

这条链没有真实物理、地面接触或碰撞反力。

## 5. Gazebo 物理仿真

前提：需要先安装 Gazebo Classic 相关包。

建议安装：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-broadcaster \
  ros-humble-position-controllers \
  ros-humble-controller-manager
```

启动：

```bash
ros2 launch legged_control gazebo_position_control.launch.py
```

当前 Gazebo 链路：

- `gait_node` 产生 `/joint_commands`
- `gazebo_control_bridge` 转成 `gait_position_controller/commands`
- Gazebo `/joint_states` 再回桥到 `/joint_states_aggregated`

仿真模型文件：

- `/home/grayerd/Desktop/Projects/rc/塞北箭4urdf/urdf/塞北箭4_sim.urdf`

这个仿真模型已经额外补了：

- 有效关节限位
- 基础阻尼/摩擦
- 脚端简化碰撞体
- `gazebo_ros2_control` 接口骨架

## 6. 速度指令

### 6.1 手柄

`teleop_node` 会把 `/joy` 转成 `/cmd_vel`。

默认映射（北通鲲鹏 20）：

| 控件 | 话题字段 | 说明 |
|------|---------|------|
| 左摇杆前后 | `linear.x` | 前进/后退，最大 0.05 m/s |
| 左摇杆左右 | `linear.y` | 侧移，最大 0.02 m/s |
| 右摇杆左右 | `angular.z` | 偏航，最大 0.1 rad/s |
| RT（右扳机） | `linear.z` > 0 | 升高机身，最大 0.03 m/s |
| LT（左扳机） | `linear.z` < 0 | 降低机身，最大 −0.03 m/s |

机身高度变化只在 WAIT / TROT 阶段生效，起身（STANDUP）期间忽略。

### 6.2 手工发命令

升高机身（持续发送约 3s 可从 0.28 升至 0.35）：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.03}, angular: {z: 0.0}}" -r 10
```

降低机身：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: -0.03}, angular: {z: 0.0}}" -r 10
```

停止高度变化（发一次）：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -1
```

小速前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

小速左转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10
```

原地转向：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10
```

## 7. 安全机制

当前位控链内建这些保护：

- `cmd_vel` 二次限幅
- 每关节目标变化率限幅
- 连续 IK 失败 / 连续撞限位进入 `FAULT`
- `FAULT` 时回到安全默认站姿，并用较低增益保持
- 独立急停话题 `/emergency_stop`

### 7.1 急停

触发急停：

```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" -1
```

清除急停信号：

```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: false}" -1
```

注意：

- 清除急停后，节点仍停留在 `FAULT`
- 当前版本需要重新 launch 才恢复

## 8. 当前保守参数

`gait` 里当前默认安全限幅是：

```yaml
max_cmd_vx: 0.15
max_cmd_vy: 0.08
max_cmd_yaw: 0.4
max_joint_speed: 2.0
fault_hold_kp: 0.8
fault_hold_kd: 0.2
```

首次实机建议再保守一点：

- `max_cmd_vx: 0.08`
- `max_cmd_yaw: 0.2`

## 9. 实机试验顺序

建议顺序：

1. 先跑 RViz 仿真，确认关节方向和默认站姿
2. 再跑 Gazebo，确认地面接触和基础稳定性
3. 实机只先做：站立 -> 原地踏步 -> 极低速直行
4. 最后再尝试转弯

第一次实机测试时建议：

- 急停终端常驻
- 两人配合，一人盯机器人，一人盯终端
- 先不用雷达 IMU

## 10. 当前未做完的部分

当前位控还不是完整闭环稳定控制，暂时没有：

- IMU 姿态稳定
- 跌倒检测恢复
- 地形适应
- 基于机身姿态的 body compensation
- 自动 fault 恢复

现阶段目标是：

- 安全地把基础位控步态跑起来
- 先完成平地低速行走
