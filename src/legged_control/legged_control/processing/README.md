# processing 层

数据处理层。消费标准 ROS2 topic，产出策略 (policy) 所需的观测向量分量。**虚实共用**——不区分数据来自真实硬件还是 Gazebo 仿真。

---

## 节点列表

### state_estimator_node

**职责：** 融合 IMU 姿态与关节运动学，估计机身线速度、角速度和重力方向。

速度估计：互补滤波（运动学雅可比 0.8 + IMU 积分 0.2）。关节数据直接使用 URDF 坐标系（`joint_aggregator` 或 `gazebo_control_bridge` 已完成转换）。

输出 9 维向量 `[vx, vy, vz, wx, wy, wz, gx, gy, gz]`。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/state_estimate` | `std_msgs/Float32MultiArray` (9 floats) |
| 订阅 | `odin1/imu/filtered` | `sensor_msgs/Imu` |
| 订阅 | `/joint_states_aggregated` | `sensor_msgs/JointState` (URDF frame) |

> **注意：** 当前为盲策略（49 维观测，无高程扫描）。原 `height_scan_node` 已移除；若将来训练感知策略需要重新引入。

### teleop_node

**职责：** 手柄 (`/joy`) → 速度指令 (`/cmd_vel`) + 姿态指令 (`/posture_command`) + 目标高度 (`/height_command`)。

轴映射、死区、反转均由 `robot.yaml` 的 `teleop` 段配置。LT/RT 触发积分成目标站立高度（0.15–0.28 m）。

| | Topic | 类型 |
|---|---|---|
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/posture_command` | `std_msgs/Bool` |
| 发布 | `/height_command` | `std_msgs/Float32` |
| 订阅 | `/joy` | `sensor_msgs/Joy` |

---

## 层接口

```
数据源 (real/ 或 sim/)          processing 处理               对外输出 (消费方)
─────────────────────────────────────────────────────────────────────────
/joint_states_aggregated  →  state_estimator_node   →  /state_estimate (policy)
odin1/imu/filtered        →  state_estimator_node
/joy                      →  teleop_node            →  /cmd_vel + /height_command (policy)
```

## 数据源来源

| 真机模式 | 仿真模式 |
|---------|---------|
| `real/joint_aggregator`（motor→URDF） | `sim/gazebo_control_bridge`（重排 Gazebo→YAML） |
| `odin_ros_driver/host_sdk_sample` | URDF IMU 插件 |
| `realsense2_camera_node` | URDF depth 插件 |
