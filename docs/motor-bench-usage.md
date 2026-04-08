# 单腿电机测试台使用说明

用于在装机前对单条腿的三个关节进行通信验证和运动测试。

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control unitree_actuator_sdk
source install/setup.bash

ros2 launch legged_control motor_bench.launch.py leg:=FR
```

`leg` 可选值：`FR` / `FL` / `RR` / `RL`（默认 `FR`）

手柄设备非默认时：
```bash
ros2 launch legged_control motor_bench.launch.py leg:=FR joy_device:=/dev/input/js1
```

## 手柄操作

| 输入 | 动作 |
|------|------|
| 左摇杆 上/下 | 控制当前关节位置（映射到该关节的 q_min / q_max） |
| **B** | 切换到下一个关节（hip → thigh → calf → hip） |
| **X** | 切换到上一个关节 |

摇杆松开时（位于死区内），关节保持当前位置不动。切换关节时，被离开的关节锁定在当前反馈位置。

## 启动顺序

节点启动后终端会打印：

```
[motor_bench_node] Motor bench ready  leg=FR  active=FR_hip  (B=next joint  X=prev joint)
```

每次切换关节时打印：

```
[motor_bench_node] Active joint → FR_thigh
```

## 配置

关节限位和 PD 增益在 `src/legged_control/config/robot.yaml` 中修改：

```yaml
joints:
  - name: FR_hip
    q_min: -1.047   # rad，约 -60°
    q_max:  1.047   # rad，约 +60°

control:
  kp: 20.0
  kd: 0.5
```

修改后需重新 `colcon build`。
