# Legged Robot Position Demo

本文档只描述当前用于远端实机验证的演示版本：`mode:=position`。

这个版本的目标不是闭环行走，也不是策略部署，而是：

- 不依赖 IMU / LiDAR / policy model
- 用手柄驱动一个开环位控步态
- 在挂空条件下验证手柄链路、关节方向、基本位控和急停逻辑

不相关的旧模式说明不在这里展开。

---

## 1. 快速使用

### 1.1 安装与编译

首次使用前：

```bash
# 串口权限（只需一次，之后重新登录生效）
sudo usermod -a -G dialout $USER

# joy 驱动
sudo apt install ros-humble-joy

# 编译
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
```

每次修改 `robot.yaml` 或 Python 节点后，建议重新：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
```

### 1.2 启动命令

标准启动：

```bash
ros2 launch legged_control robot.launch.py mode:=position
```

现在 `robot.launch.py` 的默认模式也已经改成 `position`，因此下面这条命令等价：

```bash
ros2 launch legged_control robot.launch.py
```

如需覆盖串口：

```bash
ros2 launch legged_control robot.launch.py mode:=position serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1
```

查看 launch 参数：

```bash
ros2 launch legged_control robot.launch.py --show-args
```

远端验证时统一要求：

- 使用 `legs:=all`
- 不要用 `legs:=FR`、`legs:=FR,FL` 之类子集腿启动方式做演示

### 1.3 手柄映射

当前 `robot.yaml` 中的手柄映射是：

```yaml
teleop:
  axis_vx:  1
  axis_vy:  0
  axis_yaw: 3
  invert_vx:  true
  invert_vy:  false
  invert_yaw: true
  btn_emergency_stop: -1
```

对应操作：

- 左摇杆上下 -> 前进 / 后退命令
- 左摇杆左右 -> 当前模式未使用
- 右摇杆左右 -> 转向命令

注意：

- `position_gait_node` 当前只使用 `vx` 和 `yaw`
- `vy` 虽然会被 `teleop_node` 生成，但当前模式不使用

### 1.4 急停按钮设置

默认未配置急停按钮：

```yaml
btn_emergency_stop: -1
```

建议远端验证前务必配置一个按钮作为急停。

先查看手柄消息：

```bash
ros2 topic echo /joy
```

按一下你想作为急停的按钮，观察 `buttons` 数组里哪个索引变成 `1`，然后把该索引写入：

```yaml
teleop:
  btn_emergency_stop: 0
```

急停行为说明：

- 第一次按下：进入急停锁存
- 第二次按下：解除急停锁存

建议远端验证人员在正式演示前，先单独测试一次急停按钮是否生效。

### 1.5 推荐验证流程

建议远端人员严格按下面顺序测试。

#### 步骤 1：挂空

- 机器人四足离地
- 周围无障碍物
- 手边有硬件断电手段

#### 步骤 2：确认上电姿态

- 机器人以上电即零位的通用中立姿态启动
- 不要在趴姿或随机姿态启动这个模式

#### 步骤 3：先测试急停

- 启动后不推动摇杆
- 按急停按钮一次，确认系统进入零命令锁定
- 再按一次，确认可解除

#### 步骤 4：小命令测试前进

- 轻推左摇杆前后
- 只观察小幅摆腿方向是否符合预期
- 不要一开始就推到最大

#### 步骤 5：小命令测试转向

- 轻推右摇杆左右
- 观察 hip 转向摆动是否符合预期

#### 步骤 6：逐步增大命令

- 若前两步都正常，再逐步增大摇杆输入
- 如发现方向异常或幅度异常，立即急停

### 1.6 终端现象

启动后你会看到：

- `motor_bus_node` 输出上电零位标定日志
- `teleop_node` 输出手柄配置
- `position_gait_node` 输出当前步态参数
- `passive_monitor_node` 持续刷新 12 个关节位置

其中 `passive_monitor_node` 的输出可用来观察：

- 零命令时是否保持在 0 附近
- 轻推摇杆后哪些关节在动
- 是否有异常单腿大幅偏移

---

## 2. 适用范围

当前演示版本适用于：

- 机器人挂空，不落地承重
- 机器人在“通用中立姿态”上电
- 需要快速展示电机位控动作和手柄控制

当前演示版本不保证：

- 落地稳定行走
- 闭环姿态控制
- 地面接触安全性

如果机器人落地，请谨慎，不要直接把这版当作可稳定行走控制器使用。

---

## 2. 核心假设

这个版本依赖下面这个关键假设：

- 机器人上电时已经处于你希望的通用默认姿态
- `motor_bus_node` 启动时会把上电姿态标定为每个关节的 `q=0`
- `position_gait_node` 后续围绕这个零位做小幅位控摆腿

换句话说：

- 这版不是围绕 `robot.yaml.default_q` 做动作
- 这版是围绕“上电零位”做动作

因此，上电姿态必须一致。

---

## 3. 已知安全边界

当前实现使用了比实测更保守的软件限幅。

已知条件：

- `thigh` 和 `calf` 在零位附近至少有 `+-30 deg` 活动范围
- `hip` 的旧限位中只有髋关节部分相对可信

因此在 `position_demo` 中额外加了软件硬限幅：

```yaml
position_demo:
  hip_min: -0.20
  hip_max: 0.20
  thigh_min: -0.52
  thigh_max: 0.52
  calf_min: -0.52
  calf_max: 0.52
```

这组限制是围绕上电零位的关节目标角限制，不依赖旧的趴姿标定限位。

---

## 4. 当前模式做什么

启动 `mode:=position` 后，系统链路是：

```text
joy_node -> /joy -> teleop_node -> /cmd_vel -> position_gait_node -> /joint_commands -> motor_bus_node
```

会启动这些节点：

- `motor_bus_front`
- `motor_bus_rear`
- `joy_node`
- `teleop_node`
- `position_gait_node`
- `passive_monitor_node`

其中：

- `teleop_node` 把手柄映射成 `/cmd_vel`
- `position_gait_node` 根据 `/cmd_vel` 生成开环步态目标角
- `motor_bus_node` 以 PD 方式跟踪这些目标角
- `passive_monitor_node` 在终端持续显示 12 个关节当前位置

---

## 5. 当前保护逻辑

当前版本已经加上的保护逻辑：

1. `/cmd_vel` 超时回零

- 如果超过 `0.5 s` 没收到新的 `/cmd_vel`
- `position_gait_node` 自动回到零命令
- 不会继续沿用最后一帧手柄指令摆腿

2. 手柄急停锁存

- 急停按钮按一下：锁定急停
- 再按一下：解除急停
- 锁定期间 `teleop_node` 持续发布全零 `/cmd_vel`

3. 关节目标硬限幅

- `position_gait_node` 在发布前对 `hip/thigh/calf` 做软件裁剪
- 即使轨迹生成公式有偏差，也不会超出当前配置的保守范围

---

## 6. 关节正方向约定

当前步态生成已经按实机提供的正方向信息写死。

约定为：`q_motor` 增大时，关节运动方向如下。

### hip

- `FR_hip`: 向内
- `FL_hip`: 向外
- `RR_hip`: 向外
- `RL_hip`: 向内

### thigh

- `FR_thigh`: 向前
- `FL_thigh`: 向后
- `RR_thigh`: 向前
- `RL_thigh`: 向后

### calf

- `FR_calf`: 向后
- `FL_calf`: 向前
- `RR_calf`: 向后
- `RL_calf`: 向前

这部分已经被用于步态符号映射，不再是默认猜测。

---

## 7. 演示步态参数

当前演示参数在 `src/legged_control/config/robot.yaml`：

```yaml
position_demo:
  loop_hz: 50.0
  min_command: 0.05
  base_frequency_hz: 0.8
  frequency_scale_hz: 1.2
  hip_amplitude: 0.10
  thigh_amplitude: 0.16
  calf_amplitude: 0.22
  hip_min: -0.20
  hip_max: 0.20
  thigh_min: -0.52
  thigh_max: 0.52
  calf_min: -0.52
  calf_max: 0.52
```

含义：

- `loop_hz`: 关节目标发布频率
- `min_command`: 小于该输入幅值时视为零命令
- `base_frequency_hz`: 最低步态频率
- `frequency_scale_hz`: 命令越大，频率越高
- `hip/thigh/calf_amplitude`: 各关节基础摆动幅度
- `*_min / *_max`: 围绕上电零位的软件安全裁剪范围

如果远端验证人员觉得动作太大或太激进，优先减小：

- `thigh_amplitude`
- `calf_amplitude`
- `hip_amplitude`

而不是先去改限幅。

---

## 8. 当前已知限制

当前版本的已知限制：

- 这是开环位控演示，不是闭环稳定步行控制器
- 不使用 IMU / odom，不具备姿态反馈
- 只适合挂空演示，不建议直接落地跑
- `vy` 侧向命令当前未用于步态生成
- 文档中描述的安全边界基于当前已知零位附近活动范围，不等同于完整机械极限

---

## 9. 远端验证人员最重要的三条

1. 必须挂空验证，不要直接落地跑。
2. 必须先测试急停按钮是否生效。
3. 必须从很小的摇杆输入开始，不要一开始打满。
