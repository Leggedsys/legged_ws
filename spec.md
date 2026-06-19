# legged_deploy

## 工作模式

1. policy_mode：运行策略，连接IMU、深度相机与电机角度反馈输入，接受遥控手柄command，输出关节目标位置至电机。
2. passive_mode：电机不输出力矩，只读取各传感器数据、电机角度反馈和遥控手柄command，用于测试数据链路、电机offset对齐等。
3. sim_mode：在仿真环境中运行policy，使用仿真环境生成的数据作为输入，同时接受遥控器手柄command。

policy_mode、passive_mode也需要可视化的仿真环境。

## 各工作模式详解

### policy_mode

观测空间（v1.0）

| 项                | 维度          | 说明                                        |
| ----------------- | ------------- | ------------------------------------------- |
| base_lin_vel      | 3             | 机身线速度（yaw frame）                     |
| base_ang_vel      | 3             | 机身角速度                                  |
| projected_gravity | 3             | 重力投影向量                                |
| velocity_commands | 3             | vx, vy, ω_z 指令                           |
| joint_pos_rel     | 12            | 关节角度（相对默认姿态）                    |
| joint_vel_rel     | 12            | 关节速度                                    |
| last_action       | 12            | 上一时刻动作                                |
| height_scan       | 187           | 前向高度扫描（1.2×0.6m grid, 0.05m分辨率） |
| **合计**    | **235** |                                             |

动作空间：12维关节位置残差（position control offset）。

工作流程：趴姿启动 --> 检查传感器数据完备性 --> 受到站立指令 --> 缓慢站立至默认位置 --> 接入策略并运行 --> 受到趴下指令 --> 从当前位置缓慢切换至趴姿 --> 电机停止输出力矩

需对齐项（在passive_mode中检验)：

1. 电机offset、direction、传动比、限位
2. IMU方向
3. 深度相机转高程图
4. 手柄键位

### passive_mode

观测空间同policy_mode，在终端显示这些数据的度数（以合适的方式)。

#### 关节角度的校验方式与sim2real

1. 上电后读取电机角度 `angle_0 = offset`
2. 将电机角度转为 `angle_1 = angle_0 - offset`，因此电机上电位置为零点位置
3. 然后对齐实机和urdf的电机方向，`angle_2 = direction * (angle_0 - offset)`，direction可先随便给，在后面步骤中对齐
4. 将 `angle_2`作为urdf中的关节角度，在仿真环境中显示狗的动作，与实机对比以确定direction、传动比是否正确

#### IMU正方向确定

利用仿真环境[待定]

#### 深度相机转高程图

在仿真环境中显示读取的真实高程图点云

#### 手柄键位

在终端显示由手柄信号解析的command

### sim_mode

在动力学仿真环境中测试policy运行效果
