# 位控使用说明

本文档对应当前分支的 `position_control` 实现，覆盖：

- RViz 运动学仿真：`position_control_sim.launch.py`
- 真机位控：`robot.launch.py mode:=position_control`
- 上机验证与 `PD` 调参流程

## 1. 当前控制流程

当前 `position_control` 模式是显式姿态命令流程：

1. 启动后保持 `PASSIVE`，电机 `kp=kd=0`
2. 收到 `/posture_command=true` 后，平滑从零位起身到 `default_q`
3. 实际关节反馈接近 `default_q` 且关节速度接近 0 后，进入站立待机 `WAIT`
4. 只有收到非零 `/cmd_vel` 后，才进入步态 `TROT`
5. 收到 `/posture_command=false` 后，平滑趴下回零位
6. 实际关节反馈接近零位且关节速度接近 0 后，回到 `PASSIVE`

当前版本：

- 不依赖 IMU
- 不依赖里程计
- 不依赖雷达
- 支持前进和转弯基础步态
- 支持实时机身高度调节：`linear.z`
- 带关节目标变化率限制、IK/限位保护、跟踪误差保护和 `FAULT` 安全回退

## 2. 配置文件

默认统一使用：

- `src/legged_control/config/robot.yaml`

当前 RViz 仿真与真机共享这份配置，因此：

- 零位一致
- `default_q` 一致
- gait 参数一致
- teleop 配置一致

关键配置项：

- `joints[*].default_q`
- `joints[*].q_min / q_max`
- `control.kp / kd`
- `standup.ramp_duration / lie_down_duration`
- `gait.max_cmd_vx / max_cmd_yaw`
- `gait.motion_blend_duration`
- `gait.yaw_stride_scale`
- `gait.tracking_error_*`

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

- 检查零位和 `default_q`
- 检查关节方向
- 检查起身/趴下流程
- 检查步态相位和转弯轨迹
- 检查安全状态机是否按预期触发

这条链没有真实动力学、地面接触或电机通信延迟，只适合验证：

- 控制流程
- 话题契约
- 参数趋势

## 5. 手工控制命令

没有手柄时，可直接用 topic 驱动。

起身：

```bash
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: true}"
```

趴下：

```bash
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: false}"
```

前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

原地转向：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.08}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

升高机身：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.03}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

降低机身：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: -0.03}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果使用手柄，`A` 键等价于在 `true/false` 间切换 `/posture_command`。

## 6. 上机验证与 PD 调参手册

### 6.1 目标

按“先安全、再站稳、最后再走”的顺序完成：

1. 确认零位与默认站姿
2. 用 `stand` 模式调静态 `PD`
3. 用 `position_control` 模式验证起身/趴下
4. 架空验证前进与转弯
5. 再落地小速度验证

### 6.2 上机前准备

1. 确认编译的是最新代码
2. 确认串口权限正常
3. 确认 `robot.yaml` 已经是这台机器的实测参数

强烈建议：

1. 第一轮架空测试，四脚离地
2. 人站在可立即断电或急停的位置
3. 第一轮 `kp` 偏低、`kd` 保守
4. 任何异常先停节点，再断电

### 6.3 第一步：确认零位和默认站姿

启动 `passive`：

```bash
ros2 launch legged_control robot.launch.py
```

检查：

1. 关节方向是否正确
2. `q_min / q_max` 是否合理
3. `default_q` 是否真的是期望站姿
4. 四条腿左右是否对称

如果这一步不对，先修 `robot.yaml`，不要急着调 `PD`。

### 6.4 第二步：用 stand 模式调静态 PD

启动：

```bash
ros2 launch legged_control robot.launch.py mode:=stand
```

推荐先从当前配置附近开始：

- `kp = 1.3`
- `kd = 0.3`

如果你怀疑偏硬，第一轮可以先降到：

- `kp = 0.8 ~ 1.0`
- `kd = 0.2 ~ 0.3`

在线调参：

```bash
ros2 param set /stand_node kp 1.0
ros2 param set /stand_node kd 0.25
```

观察：

1. `kp` 太小：站不住、软、容易塌
2. `kp` 太大：关节发紧、抖、嗡嗡震
3. `kd` 太小：回正后晃很多次
4. `kd` 太大：动作发闷、拖拽感强

推荐调法：

1. 固定 `kd`
2. 逐步加 `kp`，直到“能稳稳站住但不明显抖”
3. 再逐步加 `kd`，直到“回正利落但不过分发闷”
4. 满意后写回 `robot.yaml`

### 6.5 第三步：验证起身/趴下流程

启动：

```bash
ros2 launch legged_control robot.launch.py mode:=position_control
```

然后手工发：

```bash
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /posture_command std_msgs/msg/Bool "{data: false}"
```

只看三件事：

1. 起身是否平滑
2. 起身后是否能稳定停在站姿
3. 趴下是否能完整回零位

如果这一步都不稳，不要继续走路，优先回去调：

- `control.kp / kd`
- `standup.ramp_duration`
- `standup.lie_down_duration`

### 6.6 第四步：架空验证运动命令

先起身，再发小速度命令。

前进：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

原地转：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.04}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

检查：

1. 四腿节拍是否对
2. 是否有单腿乱甩
3. 是否有某个关节反向
4. 转弯时是否触发 `FAULT`
5. 停止后是否能回到站立待机

### 6.7 第五步：落地小幅验证

顺序建议：

1. 起身站稳 5~10 秒
2. 小前进
3. 小转弯
4. 停止
5. 趴下

建议命令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.04}}"
```

### 6.8 出现这些现象就先停

1. 明显高频抖动
2. 某条腿突然抽动
3. 起身过程中姿态严重偏斜
4. 转弯时两步内明显失稳
5. 进入 `FAULT`
6. 趴下回零不完整且重复出现

## 7. 当前安全机制

当前位控链内建这些保护：

1. `/cmd_vel` 二次限幅
2. 每关节目标变化率限幅
3. 连续 IK 失败 / 连续撞限位进入 `FAULT`
4. 连续关节跟踪误差过大进入 `FAULT`
5. `FAULT` 时回到安全默认站姿，并用较低增益保持
6. 独立急停话题 `/emergency_stop`

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

## 8. 当前关键 gait 参数

当前默认参数以 `robot.yaml` 为准，重点关注：

```yaml
gait:
  max_cmd_vx: 0.03
  max_cmd_vy: 0.00
  max_cmd_yaw: 0.08
  max_joint_speed: 0.5
  motion_blend_duration: 0.6
  yaw_stride_scale: 1.8
  tracking_error_threshold: 0.45
  tracking_error_window: 0.4
  tracking_error_ratio: 0.9
  tracking_error_grace_period: 2.0
```

首次真机建议：

1. 保持当前保守上限
2. 不要一开始就提高 `max_cmd_yaw`
3. 不要一开始就缩短 `motion_blend_duration`

## 9. 当前未做的部分

当前位控还不是完整闭环稳定控制，暂时没有：

- IMU 姿态稳定
- 跌倒检测恢复
- 地形适应
- 基于机身姿态的 body compensation
- 自动 `FAULT` 恢复

现阶段目标是：

- 安全地把基础位控步态跑起来
- 完成平地低速前进与转弯
