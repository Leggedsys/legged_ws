# policy_node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 `policy` 运行模式：新增 `joint_aggregator` 和 `policy_node` 两个节点，完成 TorchScript 策略模型的 50 Hz 部署推理循环。

**Architecture:** `joint_aggregator` 订阅 12 个独立关节 topic，合并成 `/joint_states_aggregated`（motor frame）；`policy_node` 以 50 Hz 定时器订阅该聚合 topic 及 LiDAR IMU/odometry，构建 45 维观测向量，推理后将 12 维动作解码并发布到 `/joint_commands`（motor frame）；`robot.launch.py` 的 `policy` 分支启动两个节点。

**Tech Stack:** Python 3.10, ROS2 Humble, `torch`（TorchScript CPU 推理）, `numpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`

---

## 文件结构

| 动作 | 文件 | 职责 |
|------|------|------|
| 修改 | `src/legged_control/config/robot.yaml` | 替换 `action_scale` 为嵌套结构 |
| 新建 | `src/legged_control/legged_control/joint_aggregator.py` | 12→1 topic 合并节点 |
| 新建 | `src/legged_control/legged_control/policy_node.py` | 推理主节点（含纯函数 helpers） |
| 新建 | `src/legged_control/tests/test_joint_aggregator.py` | joint_aggregator 纯函数测试 |
| 新建 | `src/legged_control/tests/test_policy_node.py` | policy_node 纯函数测试 |
| 修改 | `src/legged_control/setup.py` | 注册两个新 entry_points |
| 修改 | `src/legged_control/launch/robot.launch.py` | 实现 `mode == 'policy'` 分支 |

---

## Task 1: 更新 robot.yaml 的 action_scale

**Files:**
- Modify: `src/legged_control/config/robot.yaml`

- [ ] **Step 1: 替换 action_scale 字段**

找到 `control` 段中的 `action_scale: 0.25` 这一行，将其替换为：

```yaml
  action_scale:
    front:
      hip:   0.04
      thigh: 0.05
      calf:  0.03
    rear:
      hip:   0.05
      thigh: 0.07
      calf:  0.04
```

- [ ] **Step 2: 验证 YAML 格式正确**

```bash
cd src/legged_control
python3 -c "
import yaml
with open('config/robot.yaml') as f:
    cfg = yaml.safe_load(f)
sc = cfg['control']['action_scale']
assert sc['front']['hip'] == 0.04
assert sc['rear']['calf'] == 0.04
print('OK:', sc)
"
```

期望输出：`OK: {'front': {'hip': 0.04, 'thigh': 0.05, 'calf': 0.03}, 'rear': {'hip': 0.05, 'thigh': 0.07, 'calf': 0.04}}`

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/config/robot.yaml
git commit -m "config: replace scalar action_scale with per-joint nested structure"
```

---

## Task 2: joint_aggregator — 纯函数测试

**Files:**
- Create: `src/legged_control/tests/test_joint_aggregator.py`

- [ ] **Step 1: 写测试文件**

```python
# src/legged_control/tests/test_joint_aggregator.py
"""Tests for joint_aggregator pure helper functions (no ROS2 required)."""
import pytest


def test_ns_from_joint_name_hip():
    from legged_control.joint_aggregator import _ns_from_joint_name
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_from_joint_name_calf():
    from legged_control.joint_aggregator import _ns_from_joint_name
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_from_joint_name_thigh():
    from legged_control.joint_aggregator import _ns_from_joint_name
    assert _ns_from_joint_name('FL_thigh') == 'fl/thigh'
```

- [ ] **Step 2: 运行测试，确认失败（模块不存在）**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/legged_control/tests/test_joint_aggregator.py -v
```

期望：`ImportError: cannot import name '_ns_from_joint_name' from 'legged_control.joint_aggregator'`

---

## Task 3: joint_aggregator — 实现

**Files:**
- Create: `src/legged_control/legged_control/joint_aggregator.py`

- [ ] **Step 1: 创建 joint_aggregator.py**

```python
# src/legged_control/legged_control/joint_aggregator.py
"""
joint_aggregator

Subscribes to 12 individual /<ns>/joint_states topics published by motor_bus_node
and merges them into a single /joint_states_aggregated message.

Published immediately whenever any joint receives a new message.
Joint order matches robot.yaml joints list (FR_hip ... RL_calf, index 0-11).
All values are in motor frame (no direction/zero_offset conversion).
"""
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' -> 'fr/hip'"""
    leg, joint = name.split('_', 1)
    return f'{leg.lower()}/{joint.lower()}'


class JointAggregatorNode(Node):
    _TIMEOUT = 0.5  # seconds before a joint is considered stale

    def __init__(self) -> None:
        super().__init__('joint_aggregator')

        cfg = self._load_config()
        self._names = [j['name'] for j in cfg['joints']]
        self._latest: dict = {
            name: {'position': 0.0, 'velocity': 0.0, 'stamp': None}
            for name in self._names
        }

        self._pub = self.create_publisher(JointState, '/joint_states_aggregated', 10)

        for name in self._names:
            ns = _ns_from_joint_name(name)
            self.create_subscription(
                JointState,
                f'/{ns}/joint_states',
                lambda msg, n=name: self._on_joint_state(msg, n),
                10,
            )

        self.get_logger().info(
            f'Joint aggregator ready — tracking {len(self._names)} joints')

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _on_joint_state(self, msg: JointState, name: str) -> None:
        if msg.name and msg.name[0] == name:
            self._latest[name] = {
                'position': msg.position[0] if msg.position else 0.0,
                'velocity': msg.velocity[0] if msg.velocity else 0.0,
                'stamp': time.monotonic(),
            }
        self._publish()

    def _publish(self) -> None:
        now_mono = time.monotonic()
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name     = list(self._names)
        out.position = []
        out.velocity = []

        for name in self._names:
            entry = self._latest[name]
            if (entry['stamp'] is not None
                    and now_mono - entry['stamp'] > self._TIMEOUT):
                self.get_logger().warn(
                    f'joint_aggregator: {name} stale (>{self._TIMEOUT}s)',
                    throttle_duration_sec=5.0,
                )
            out.position.append(float(entry['position']))
            out.velocity.append(float(entry['velocity']))

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JointAggregatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: 运行测试，确认通过**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/legged_control/tests/test_joint_aggregator.py -v
```

期望：`3 passed`

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/legged_control/joint_aggregator.py \
        src/legged_control/tests/test_joint_aggregator.py
git commit -m "feat: add joint_aggregator node — merges 12 joint topics into one"
```

---

## Task 4: policy_node — 纯函数测试

**Files:**
- Create: `src/legged_control/tests/test_policy_node.py`

- [ ] **Step 1: 写测试文件**

```python
# src/legged_control/tests/test_policy_node.py
"""Tests for policy_node pure helper functions (no ROS2 or torch required)."""
import numpy as np
import pytest


# ─── _quat_rotate_inverse ───────────────────────────────────────────────────

def test_quat_rotate_inverse_identity():
    """Identity quaternion: vector passes through unchanged."""
    from legged_control.policy_node import _quat_rotate_inverse
    q = np.array([1.0, 0.0, 0.0, 0.0])   # [w, x, y, z]
    v = np.array([0.0, 0.0, -1.0])
    np.testing.assert_allclose(_quat_rotate_inverse(q, v), v, atol=1e-6)


def test_quat_rotate_inverse_upright_gravity():
    """Upright robot (identity quat): g_body should be [0, 0, -1]."""
    from legged_control.policy_node import _quat_rotate_inverse
    q = np.array([1.0, 0.0, 0.0, 0.0])
    g_body = _quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0]))
    np.testing.assert_allclose(g_body, [0.0, 0.0, -1.0], atol=1e-6)


def test_quat_rotate_inverse_90deg_pitch():
    """90° forward pitch: gravity should shift to +x body axis."""
    from legged_control.policy_node import _quat_rotate_inverse
    # 90° pitch around Y (nose down): quat = [cos45, 0, sin45, 0]
    import math
    c, s = math.cos(math.pi / 4), math.sin(math.pi / 4)
    q = np.array([c, 0.0, s, 0.0])
    g_body = _quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0]))
    # After 90° nose-down rotation, gravity in body frame points along +x
    np.testing.assert_allclose(g_body[0],  1.0, atol=1e-6)
    np.testing.assert_allclose(g_body[2],  0.0, atol=1e-6)


# ─── _motor_to_urdf ─────────────────────────────────────────────────────────

def test_motor_to_urdf_positive_direction_with_offset():
    from legged_control.policy_node import _motor_to_urdf
    directions   = np.array([1.0, 1.0])
    zero_offsets = np.array([0.0, 1.254])
    q_motor      = np.array([0.1, 0.0])
    dq_motor     = np.array([0.5, 0.2])
    q_urdf, dq_urdf = _motor_to_urdf(q_motor, dq_motor, directions, zero_offsets)
    np.testing.assert_allclose(q_urdf,  [0.1,  1.254], atol=1e-6)
    np.testing.assert_allclose(dq_urdf, [0.5,  0.2],   atol=1e-6)


def test_motor_to_urdf_negative_direction():
    from legged_control.policy_node import _motor_to_urdf
    directions   = np.array([-1.0])
    zero_offsets = np.array([-1.221])
    q_motor      = np.array([0.2])
    dq_motor     = np.array([1.0])
    q_urdf, dq_urdf = _motor_to_urdf(q_motor, dq_motor, directions, zero_offsets)
    np.testing.assert_allclose(q_urdf,  [-0.2 - 1.221], atol=1e-6)
    np.testing.assert_allclose(dq_urdf, [-1.0],          atol=1e-6)


# ─── _decode_action ─────────────────────────────────────────────────────────

def test_decode_action_zero_returns_default_motor():
    """Zero action → target_q_motor should equal default_q in motor frame (0.0)."""
    from legged_control.policy_node import _decode_action
    directions     = np.array([1.0, -1.0])
    zero_offsets   = np.array([0.0, -1.221])
    default_q_urdf = np.array([0.0, -1.221])   # = zero_offset (since default_q_motor=0)
    scales         = np.array([0.04, 0.03])
    # q_min/q_max are motor-frame values from robot.yaml
    q_mins_motor   = np.array([-1.0, 0.0])
    q_maxs_motor   = np.array([ 1.0, 5.554])
    action         = np.array([0.0, 0.0])

    result = _decode_action(action, default_q_urdf, scales, q_mins_motor, q_maxs_motor,
                            directions, zero_offsets)
    # target_q_urdf = [0, -1.221]
    # target_q_motor = ([0,-1.221] - [0,-1.221]) / [1,-1] = [0, 0]
    # clip([0,0], motor limits) = [0, 0]
    np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-6)


def test_decode_action_clips_in_motor_frame():
    """Large action clipped AFTER converting to motor frame (direction=+1 case)."""
    from legged_control.policy_node import _decode_action
    directions     = np.array([1.0])
    zero_offsets   = np.array([0.0])
    default_q_urdf = np.array([0.0])
    scales         = np.array([0.5])
    q_mins_motor   = np.array([-0.1])   # motor-frame limits
    q_maxs_motor   = np.array([ 0.1])
    action         = np.array([100.0])   # huge

    result = _decode_action(action, default_q_urdf, scales, q_mins_motor, q_maxs_motor,
                            directions, zero_offsets)
    # target_q_urdf = 0 + 0.5×100 = 50
    # target_q_motor = (50 - 0) / 1 = 50
    # clip(50, -0.1, 0.1) = 0.1
    np.testing.assert_allclose(result, [0.1], atol=1e-6)


def test_decode_action_negative_direction_clips_motor_frame():
    """Clip in motor frame for negative-direction joint."""
    from legged_control.policy_node import _decode_action
    directions     = np.array([-1.0])
    zero_offsets   = np.array([1.221])
    default_q_urdf = np.array([1.221])   # direction×0 + 1.221
    scales         = np.array([0.03])
    q_mins_motor   = np.array([0.0])    # motor-frame limits (from robot.yaml)
    q_maxs_motor   = np.array([5.554])
    action         = np.array([-100.0]) # large negative action

    result = _decode_action(action, default_q_urdf, scales, q_mins_motor, q_maxs_motor,
                            directions, zero_offsets)
    # target_q_urdf = 1.221 + 0.03×(-100) = -1.779
    # target_q_motor = (-1.779 - 1.221) / (-1) = 3.0
    # clip(3.0, 0.0, 5.554) = 3.0
    np.testing.assert_allclose(result, [3.0], atol=1e-6)


# ─── _build_joint_params ────────────────────────────────────────────────────

def test_build_joint_params_shape():
    """_build_joint_params should return six arrays of shape (12,)."""
    from legged_control.policy_node import _build_joint_params, JOINT_ORDER
    import os, yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    result = _build_joint_params(cfg)
    assert len(result) == 6
    for arr in result:
        assert arr.shape == (12,), f"Expected (12,), got {arr.shape}"


def test_build_joint_params_fr_hip_scale():
    """FR_hip (front/hip) should have scale 0.04."""
    from legged_control.policy_node import _build_joint_params, JOINT_ORDER
    import os, yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    _, _, _, _, _, scales = _build_joint_params(cfg)
    fr_hip_idx = JOINT_ORDER.index('FR_hip')
    assert scales[fr_hip_idx] == pytest.approx(0.04)


def test_build_joint_params_rr_calf_scale():
    """RR_calf (rear/calf) should have scale 0.04."""
    from legged_control.policy_node import _build_joint_params, JOINT_ORDER
    import os, yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        cfg = yaml.safe_load(f)
    _, _, _, _, _, scales = _build_joint_params(cfg)
    rr_calf_idx = JOINT_ORDER.index('RR_calf')
    assert scales[rr_calf_idx] == pytest.approx(0.04)
```

- [ ] **Step 2: 运行测试，确认失败**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/legged_control/tests/test_policy_node.py -v
```

期望：`ImportError: cannot import name '_quat_rotate_inverse' from 'legged_control.policy_node'`

---

## Task 5: policy_node — 实现

**Files:**
- Create: `src/legged_control/legged_control/policy_node.py`

- [ ] **Step 1: 创建 policy_node.py**

```python
# src/legged_control/legged_control/policy_node.py
"""
policy_node

Runs the learned locomotion policy at 50 Hz.

Observation vector (45 dims, fixed order):
  obs[0:3]   base angular velocity (rad/s)       from /odin1/imu
  obs[3:6]   projected gravity (body frame)       from /odin1/odometry quaternion
  obs[6:9]   velocity commands (vx, vy, yaw_rate) from /cmd_vel (zeros if absent)
  obs[9:21]  joint pos relative to default_q_urdf (URDF frame, rad)
  obs[21:33] joint velocity (URDF frame, rad/s)
  obs[33:45] last action (raw model output, dimensionless)

Action vector (12 dims):
  Per-joint position delta from default_q_urdf, scaled by action_scale per joint.
  Decoded to motor-frame target_q and published on /joint_commands.

Sensors: LiDAR (/odin1/imu, /odin1/odometry) must be live; if either is silent
for >0.5 s the tick loop stops publishing and logs an error.
"""
import os
import time

import numpy as np
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


# Canonical joint order — must match robot.yaml and training environment
JOINT_ORDER = [
    'FR_hip', 'FR_thigh', 'FR_calf',
    'FL_hip', 'FL_thigh', 'FL_calf',
    'RR_hip', 'RR_thigh', 'RR_calf',
    'RL_hip', 'RL_thigh', 'RL_calf',
]

_LEG_GROUP = {
    'FR': 'front', 'FL': 'front',
    'RR': 'rear',  'RL': 'rear',
}


def _quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q = [w, x, y, z].

    Equivalent to: rotate v from world frame into body frame.
    Formula matches Isaac Lab mdp.projected_gravity.
    """
    qw   = q[0]
    qvec = q[1:]
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(qvec, v) * (2.0 * qw)
    c = qvec * np.dot(qvec, v) * 2.0
    return a - b + c


def _build_joint_params(cfg: dict) -> tuple:
    """Parse robot.yaml and return six (12,) arrays aligned to JOINT_ORDER.

    Returns:
        directions    — +1 or -1 per joint
        zero_offsets  — static URDF frame offset (rad)
        default_q_urdf — standing pose in URDF frame (direction×default_q_motor + zero_offset)
        q_mins        — joint lower limit (motor frame, rad)
        q_maxs        — joint upper limit (motor frame, rad)
        scales        — action scale (rad/unit) per joint
    """
    joint_map        = {j['name']: j for j in cfg['joints']}
    action_scale_cfg = cfg['control']['action_scale']

    directions     = np.zeros(12)
    zero_offsets   = np.zeros(12)
    default_q_urdf = np.zeros(12)
    q_mins         = np.zeros(12)
    q_maxs         = np.zeros(12)
    scales         = np.zeros(12)

    for i, name in enumerate(JOINT_ORDER):
        j     = joint_map[name]
        leg   = name.split('_')[0]          # 'FR', 'FL', 'RR', 'RL'
        jtype = name.split('_', 1)[1].lower()  # 'hip', 'thigh', 'calf'
        group = _LEG_GROUP[leg]             # 'front' or 'rear'

        d        = float(j['direction'])
        zo       = float(j['zero_offset'])
        dq_motor = float(j['default_q'])

        directions[i]     = d
        zero_offsets[i]   = zo
        default_q_urdf[i] = d * dq_motor + zo
        q_mins[i]         = float(j['q_min'])
        q_maxs[i]         = float(j['q_max'])
        scales[i]         = float(action_scale_cfg[group][jtype])

    return directions, zero_offsets, default_q_urdf, q_mins, q_maxs, scales


def _motor_to_urdf(q_motor: np.ndarray, dq_motor: np.ndarray,
                   directions: np.ndarray, zero_offsets: np.ndarray) -> tuple:
    """Convert motor-frame position/velocity to URDF frame.

    q_urdf  = direction × q_motor  + zero_offset
    dq_urdf = direction × dq_motor
    """
    q_urdf  = directions * q_motor  + zero_offsets
    dq_urdf = directions * dq_motor
    return q_urdf, dq_urdf


def _decode_action(action: np.ndarray,
                   default_q_urdf: np.ndarray,
                   scales: np.ndarray,
                   q_mins_motor: np.ndarray,
                   q_maxs_motor: np.ndarray,
                   directions: np.ndarray,
                   zero_offsets: np.ndarray) -> np.ndarray:
    """Decode raw model output to motor-frame target positions.

    q_min/q_max in robot.yaml are motor-frame values, so clipping is done
    AFTER converting from URDF frame to motor frame.

    target_q_urdf  = default_q_urdf + scale × action
    target_q_motor = clip((target_q_urdf - zero_offset) / direction, q_min, q_max)
    """
    target_q_urdf  = default_q_urdf + scales * action
    target_q_motor = (target_q_urdf - zero_offsets) / directions
    return np.clip(target_q_motor, q_mins_motor, q_maxs_motor)


class PolicyNode(Node):
    _SENSOR_TIMEOUT = 0.5  # seconds

    def __init__(self) -> None:
        super().__init__('policy_node')

        cfg         = self._load_config()
        policy_cfg  = cfg.get('policy', {})
        control_cfg = cfg['control']

        (self._directions, self._zero_offsets, self._default_q_urdf,
         self._q_mins, self._q_maxs, self._scales) = _build_joint_params(cfg)

        # Observation normalisation (optional)
        obs_mean = policy_cfg.get('obs_mean', [])
        obs_std  = policy_cfg.get('obs_std',  [])
        if obs_mean and obs_std:
            self._obs_mean = np.array(obs_mean, dtype=np.float32)
            self._obs_std  = np.array(obs_std,  dtype=np.float32)
        else:
            self._obs_mean = None
            self._obs_std  = None

        # Load TorchScript model
        model_path = policy_cfg.get('model_path', '')
        if not model_path:
            raise RuntimeError(
                'policy_node: policy.model_path is not set in robot.yaml. '
                'Set it to the path of your exported .pt file.')
        self._model = torch.jit.load(model_path, map_location='cpu')
        self._model.eval()
        with torch.no_grad():                        # warm-up
            self._model(torch.zeros(1, 45))

        self._last_action = np.zeros(12, dtype=np.float32)

        # Sensor state buffers
        self._ang_vel    = np.zeros(3, dtype=np.float32)
        self._gravity    = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._cmd_vel    = np.zeros(3, dtype=np.float32)
        self._q_motor    = np.zeros(12, dtype=np.float32)
        self._dq_motor   = np.zeros(12, dtype=np.float32)
        self._last_imu_t  = None
        self._last_odom_t = None

        self._joint_idx = {name: i for i, name in enumerate(JOINT_ORDER)}

        # Subscriptions
        self.create_subscription(
            JointState, '/joint_states_aggregated', self._on_joint_states, 10)
        self.create_subscription(Imu, '/odin1/imu', self._on_imu, 10)
        self.create_subscription(
            Odometry, '/odin1/odometry', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)

        policy_hz = float(control_cfg.get('policy_hz', 50.0))
        self.create_timer(1.0 / policy_hz, self._tick)

        self.get_logger().info(
            f'Policy node ready — {policy_hz} Hz  '
            f'model={model_path}  '
            f'normalise={self._obs_mean is not None}')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        av = msg.angular_velocity
        self._ang_vel    = np.array([av.x, av.y, av.z], dtype=np.float32)
        self._last_imu_t = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        ori = msg.pose.pose.orientation
        q   = np.array([ori.w, ori.x, ori.y, ori.z])
        self._gravity     = _quat_rotate_inverse(
            q, np.array([0.0, 0.0, -1.0])).astype(np.float32)
        self._last_odom_t = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            idx = self._joint_idx.get(name)
            if idx is not None:
                self._q_motor[idx]  = float(pos)
                self._dq_motor[idx] = float(vel)

    # ── tick ─────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()

        # LiDAR timeout guard
        if (self._last_imu_t  is None or
                now - self._last_imu_t  > self._SENSOR_TIMEOUT or
                self._last_odom_t is None or
                now - self._last_odom_t > self._SENSOR_TIMEOUT):
            self.get_logger().error(
                'policy_node: LiDAR sensor timeout — /joint_commands suspended',
                throttle_duration_sec=2.0,
            )
            return

        # Convert joint readings to URDF frame
        q_urdf, dq_urdf = _motor_to_urdf(
            self._q_motor, self._dq_motor,
            self._directions, self._zero_offsets,
        )

        # Build 45-dim observation
        obs = np.concatenate([
            self._ang_vel,                    # [0:3]
            self._gravity,                    # [3:6]
            self._cmd_vel,                    # [6:9]
            (q_urdf - self._default_q_urdf),  # [9:21]
            dq_urdf,                          # [21:33]
            self._last_action,                # [33:45]
        ]).astype(np.float32)

        # Optional normalisation
        if self._obs_mean is not None:
            obs = (obs - self._obs_mean) / (self._obs_std + 1e-8)

        # Inference
        with torch.no_grad():
            action = self._model(
                torch.from_numpy(obs).unsqueeze(0)
            ).squeeze(0).numpy().astype(np.float32)

        self._last_action = action.copy()

        # Decode to motor-frame targets (clip in motor frame)
        target_q_motor = _decode_action(
            action, self._default_q_urdf, self._scales,
            self._q_mins, self._q_maxs,   # motor-frame limits from robot.yaml
            self._directions, self._zero_offsets,
        )

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name     = list(JOINT_ORDER)
        out.position = target_q_motor.tolist()
        self._pub.publish(out)

    # ── config ───────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)


def main() -> None:
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Build 使测试可以 import 到新模块**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
```

期望：`Finished <<< legged_control`（无错误）

- [ ] **Step 3: 运行纯函数测试，确认通过**

```bash
python3 -m pytest src/legged_control/tests/test_policy_node.py -v
```

期望：`10 passed`（跳过需要 ROS2 运行时的测试，纯函数全部通过）

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/legged_control/policy_node.py \
        src/legged_control/tests/test_policy_node.py
git commit -m "feat: add policy_node — 50 Hz TorchScript inference loop"
```

---

## Task 6: 注册 entry_points 并接通 launch

**Files:**
- Modify: `src/legged_control/setup.py`
- Modify: `src/legged_control/launch/robot.launch.py`

- [ ] **Step 1: 更新 setup.py**

将 `entry_points` 的 `console_scripts` 列表改为：

```python
        'console_scripts': [
            'passive_monitor_node = legged_control.passive_monitor_node:main',
            'stand_node           = legged_control.stand_node:main',
            'motor_bus_node       = legged_control.motor_bus_node:main',
            'joint_aggregator     = legged_control.joint_aggregator:main',
            'policy_node          = legged_control.policy_node:main',
        ],
```

- [ ] **Step 2: 更新 robot.launch.py — 实现 policy 分支**

将现有的：

```python
    if mode == 'policy':
        raise RuntimeError(
            "mode:=policy is not yet implemented. "
            "Available modes: passive, stand"
        )
```

替换为：

```python
    if mode == 'policy':
        kp     = float(control['kp'])
        kd     = float(control['kd'])
        motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
        return motors + [
            Node(
                package='legged_control',
                executable='joint_aggregator',
                name='joint_aggregator',
                output='screen',
            ),
            Node(
                package='legged_control',
                executable='policy_node',
                name='policy_node',
                output='screen',
            ),
        ]
```

- [ ] **Step 3: Build**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
```

期望：`Finished <<< legged_control`

- [ ] **Step 4: 验证 entry_points 已注册**

```bash
ros2 pkg executables legged_control
```

期望输出包含：
```
legged_control joint_aggregator
legged_control motor_bus_node
legged_control passive_monitor_node
legged_control policy_node
legged_control stand_node
```

- [ ] **Step 5: 运行全部测试**

```bash
python3 -m pytest src/legged_control/tests/ -v
```

期望：所有已有测试 + 新增测试全部通过。

- [ ] **Step 6: Commit**

```bash
git add src/legged_control/setup.py \
        src/legged_control/launch/robot.launch.py
git commit -m "feat: wire policy mode in launch — joint_aggregator + policy_node"
```

---

## Task 7: 更新 CLAUDE.md

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: 在「Launch parameters」表中补充 policy 说明**

找到 CLAUDE.md 中 `| \`mode\` | \`passive\` | ... |` 这行，确认 `policy` 已列在 Options 列；若描述仍为 `(not yet implemented)` 则去掉括号注释。

找到「Mode descriptions」中的 `**\`policy\`**` 段落，将其更新为：

```markdown
**`policy`** — 以 50 Hz 运行 TorchScript 策略模型（从 `robot.yaml` 的 `policy.model_path` 加载）。
启动 `joint_aggregator`（聚合 12 路关节 topic）和 `policy_node`（构建 45 维观测 → 推理 → 发布 `/joint_commands`）。
使用前须在 `robot.yaml` 中填写 `policy.model_path`。
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md for policy mode"
```

---

## 冒烟测试说明（无真机时）

以下步骤可在无 LiDAR / 无电机的情况下验证节点能正常启动和通信：

```bash
# Terminal 1 — 启动 joint_aggregator（需要先 build）
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run legged_control joint_aggregator

# Terminal 2 — 模拟一个关节发布
ros2 topic pub /fr/hip/joint_states sensor_msgs/JointState \
  "{name: ['FR_hip'], position: [0.1], velocity: [0.0]}" -r 10

# Terminal 3 — 确认聚合 topic 有数据
ros2 topic echo /joint_states_aggregated --once
```

`policy_node` 需要真实模型文件才能启动；若只测试节点结构，可在 `robot.yaml` 的 `policy.model_path` 填一个用 `torch.jit.save` 导出的 dummy 模型：

```python
import torch
m = torch.jit.script(torch.nn.Linear(45, 12))
torch.jit.save(m, '/tmp/dummy_policy.pt')
```
