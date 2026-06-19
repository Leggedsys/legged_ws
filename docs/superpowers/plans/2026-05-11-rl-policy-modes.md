# RL Policy Modes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `policy`, `passive` (extended), and `simulation` modes to `legged_control` while removing redundant nodes, per the design spec at `docs/superpowers/specs/2026-05-11-rl-policy-modes-design.md`.

**Architecture:** Four operating modes share the same motor/teleop infrastructure. `passive` runs all sensors with full obs display. `policy` adds NN inference via `policy_node`. `simulation` replaces hardware with Gazebo using the identical software stack. `position_control` is untouched.

**Tech Stack:** Python 3.10, ROS2 Humble, `torch` (TorchScript inference), `numpy`, `tf2_ros`, `imu_filter_madgwick`, `realsense2_camera`.

---

## File Map

| Action | Path | Responsibility |
|--------|------|----------------|
| Delete | `legged_control/stand_node.py` | superseded by gait_node WAIT |
| Delete | `legged_control/standup_node.py` | superseded by gait_node standup |
| Delete | `legged_control/go_m8010_6_node.py` | unregistered dead code |
| Delete | `launch/gazebo_stand.launch.py` | stand mode removed |
| Delete | `tests/test_stand_node.py` | node deleted |
| Delete | `tests/test_standup_node.py` | node deleted |
| Delete | `tests/test_stand_config_semantics.py` | stand mode deleted |
| Modify | `legged_control/kinematics.py` | add `_smoothstep`, `_numerical_jacobian`, `leg_kinematic_velocity` |
| Modify | `legged_control/gait_node.py` | update `_smoothstep` import |
| Modify | `legged_control/passive_monitor_node.py` | full obs display |
| Modify | `launch/robot.launch.py` | 4-mode support |
| Modify | `setup.py` | entry_points + data_files |
| Create | `legged_control/state_estimator_node.py` | attitude + velocity → `/state_estimate` |
| Create | `legged_control/height_scan_node.py` | depth → 325-dim grid → `/height_scan` |
| Create | `legged_control/policy_node.py` | state machine + NN inference |
| Create | `launch/gazebo_policy.launch.py` | simulation mode entry point |
| Create | `config/policy.yaml` | model path, q_default, scale, sign flips |
| Create | `tests/test_state_estimator.py` | unit tests for pure functions |
| Create | `tests/test_height_scan.py` | unit tests for pure functions |
| Create | `tests/test_policy_node.py` | unit tests for joint mapping, obs, action decode |

---

## Joint Order Reference

**robot.yaml (canonical, indices 0–11):**
`FR_hip(0) FR_thigh(1) FR_calf(2) FL_hip(3) FL_thigh(4) FL_calf(5) RR_hip(6) RR_thigh(7) RR_calf(8) RL_hip(9) RL_thigh(10) RL_calf(11)`

**Policy (training order, indices 0–11):**
`FL_hip(0) FR_hip(1) FL_thigh(2) FR_thigh(3) FL_calf(4) FR_calf(5) RL_hip(6) RR_hip(7) RL_thigh(8) RR_thigh(9) RL_calf(10) RR_calf(11)`

**Index maps (constant arrays):**
```python
# policy_vec[i] = yaml_vec[_YAML_TO_POLICY[i]]
_YAML_TO_POLICY = [3, 0, 4, 1, 5, 2, 9, 6, 10, 7, 11, 8]

# yaml_vec[j] = policy_vec[_POLICY_TO_YAML[j]]
_POLICY_TO_YAML = [1, 3, 5, 0, 2, 4, 7, 9, 11, 6, 8, 10]

# Policy indices of FR_hip and RL_hip (need sign flip on real hardware)
_HIP_SIGN_FLIP_POLICY_IDX = [1, 6]
```

---

## Task 1: Cleanup

**Files:**
- Delete: `legged_control/stand_node.py`, `standup_node.py`, `go_m8010_6_node.py`
- Delete: `launch/gazebo_stand.launch.py`
- Delete: `tests/test_stand_node.py`, `tests/test_standup_node.py`, `tests/test_stand_config_semantics.py`
- Modify: `legged_control/kinematics.py`
- Modify: `legged_control/gait_node.py`
- Modify: `setup.py`

- [ ] **Step 1: Delete dead files**

```bash
git rm src/legged_control/legged_control/stand_node.py \
       src/legged_control/legged_control/standup_node.py \
       src/legged_control/legged_control/go_m8010_6_node.py \
       src/legged_control/launch/gazebo_stand.launch.py \
       src/legged_control/tests/test_stand_node.py \
       src/legged_control/tests/test_standup_node.py \
       src/legged_control/tests/test_stand_config_semantics.py
```

- [ ] **Step 2: Move `_smoothstep` into `kinematics.py`**

Append to `src/legged_control/legged_control/kinematics.py`:

```python
def _smoothstep(t: float) -> float:
    """Smooth clamped cubic interpolation in [0, 1]."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)
```

- [ ] **Step 3: Fix import in `gait_node.py`**

In `src/legged_control/legged_control/gait_node.py`, replace:
```python
from legged_control.standup_node import _smoothstep
```
with:
```python
from legged_control.kinematics import _smoothstep
```

- [ ] **Step 4: Update `setup.py` entry_points — remove stand/standup, add new nodes**

In `src/legged_control/setup.py`, replace the `console_scripts` block with:

```python
"console_scripts": [
    "passive_monitor_node  = legged_control.passive_monitor_node:main",
    "motor_bus_node        = legged_control.motor_bus_node:main",
    "joint_aggregator      = legged_control.joint_aggregator:main",
    "teleop_node           = legged_control.teleop_node:main",
    "gait_node             = legged_control.gait_node:main",
    "fake_motor_bus_node   = legged_control.fake_motor_bus_node:main",
    "urdf_joint_state_bridge = legged_control.urdf_joint_state_bridge:main",
    "gazebo_control_bridge = legged_control.gazebo_control_bridge:main",
    "state_estimator_node  = legged_control.state_estimator_node:main",
    "height_scan_node      = legged_control.height_scan_node:main",
    "policy_node           = legged_control.policy_node:main",
],
```

- [ ] **Step 5: Build and verify existing tests still pass**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
colcon build --packages-select legged_control
source install/setup.bash
python3 -m pytest src/legged_control/tests/ -v --ignore=src/legged_control/tests/test_gait_node.py -x
```

Expected: all remaining tests pass. `test_gait_node.py` excluded only if it imports from deleted modules — check and fix if needed.

- [ ] **Step 6: Fix any test imports that referenced deleted modules, then commit**

```bash
git add src/legged_control/
git commit -m "refactor: remove stand/standup/legacy nodes, migrate _smoothstep to kinematics"
```

---

## Task 2: `state_estimator_node` — pure functions

**Files:**
- Modify: `legged_control/kinematics.py` (add kinematic velocity helpers)
- Create: `tests/test_state_estimator.py`
- Create: `legged_control/state_estimator_node.py` (pure functions only in this task)

### Background

`state_estimator_node` subscribes to:
- `odin1/imu/filtered` (`sensor_msgs/Imu`, orientation quaternion + angular_velocity from `imu_filter_madgwick`)
- `/joint_states_aggregated` (`sensor_msgs/JointState`)

Publishes `/state_estimate` as `std_msgs/Float32MultiArray` with layout:
- `data[0:3]` = `base_lin_vel` in yaw frame (m/s)
- `data[3:6]` = `base_ang_vel` in body frame (rad/s), from IMU directly
- `data[6:9]` = `projected_gravity` in body frame (unit vector)

**Velocity estimation** uses a complementary filter: kinematic leg velocity (assuming contact) is blended with IMU integration. All four legs contribute equally.

- [ ] **Step 1: Write failing tests**

Create `src/legged_control/tests/test_state_estimator.py` (implementation not yet written, all imports will fail):

> *(full test file listed in Step 2 below — write it first)*

- [ ] **Step 1b: Run to confirm they fail**

```bash
python3 -m pytest src/legged_control/tests/test_state_estimator.py -v
```

Expected: **FAIL** — `projected_gravity_from_quat` not defined in `kinematics`.

- [ ] **Step 2: Add kinematic velocity helpers to `kinematics.py`**

Append to `src/legged_control/legged_control/kinematics.py`:

```python
import numpy as np


def _numerical_jacobian(
    leg: str, q_urdf: tuple[float, float, float], eps: float = 1e-4
) -> np.ndarray:
    """Compute 3×3 numerical Jacobian dp_foot/dq for one leg.

    Returns J where J[j, i] = d(foot_pos[j]) / d(q[i]).
    """
    p0 = np.array(forward_kinematics(leg, q_urdf))
    J = np.zeros((3, 3))
    for i in range(3):
        q_plus = list(q_urdf)
        q_plus[i] += eps
        p_plus = np.array(forward_kinematics(leg, tuple(q_plus)))
        J[:, i] = (p_plus - p0) / eps
    return J


def leg_kinematic_velocity(
    leg: str,
    q_urdf: tuple[float, float, float],
    dq_urdf: tuple[float, float, float],
) -> np.ndarray:
    """Estimate body linear velocity from one leg assuming foot is in contact.

    Returns v_body in leg's base frame (3-vector). When foot is stationary
    in world frame, body velocity = -J @ dq.
    """
    J = _numerical_jacobian(leg, q_urdf)
    dq = np.array(dq_urdf)
    return -J @ dq


def projected_gravity_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Compute gravity projection in body frame from orientation quaternion.

    World gravity = [0, 0, -1]. Rotate into body frame using R^T (= R_world_to_body).
    Returns unit vector (3,).
    """
    # Rotation matrix from world to body: R^T where R is body-to-world
    # Using quaternion: R_body_to_world then transpose
    x, y, z, w = qx, qy, qz, qw
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),        1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])
    g_world = np.array([0.0, 0.0, -1.0])
    return R.T @ g_world  # body frame gravity direction


def yaw_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Extract yaw-only rotation matrix (3×3) from quaternion for velocity frame."""
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy, -sy, 0.0],
        [sy,  cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
```

- [ ] **Step 3: Write test file (now functions exist — tests should pass)**

Create `src/legged_control/tests/test_state_estimator.py`:

```python
import math
import numpy as np
import pytest
from legged_control.kinematics import (
    projected_gravity_from_quat,
    yaw_rotation_matrix,
    leg_kinematic_velocity,
)


def test_projected_gravity_identity():
    """Identity quaternion → gravity is [0, 0, -1] in body frame."""
    g = projected_gravity_from_quat(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(g, [0.0, 0.0, -1.0], atol=1e-6)


def test_projected_gravity_pitch_90():
    """90° pitch (nose up) → gravity has +x component in body frame."""
    # Quaternion for 90° pitch: qy = sin(45°), qw = cos(45°)
    s = math.sin(math.pi / 4)
    g = projected_gravity_from_quat(0.0, s, 0.0, s)
    assert g[0] > 0.9, f"Expected x>0.9, got {g}"
    assert abs(g[2]) < 0.1, f"Expected z≈0, got {g}"


def test_projected_gravity_unit_length():
    """Output must always be unit vector."""
    g = projected_gravity_from_quat(0.1, 0.2, 0.3, 0.9)
    np.testing.assert_allclose(np.linalg.norm(g), 1.0, atol=1e-5)


def test_yaw_rotation_identity():
    """Identity quaternion → identity rotation in xy plane."""
    R = yaw_rotation_matrix(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-6)


def test_yaw_rotation_90():
    """90° yaw → x maps to y."""
    s = math.sin(math.pi / 4)
    R = yaw_rotation_matrix(0.0, 0.0, s, s)
    v = R @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(v, [0.0, 1.0, 0.0], atol=1e-5)


def test_kinematic_velocity_zero_dq():
    """Zero joint velocities → zero body velocity."""
    v = leg_kinematic_velocity("FL", (0.0, 0.7, -1.2), (0.0, 0.0, 0.0))
    np.testing.assert_allclose(v, [0.0, 0.0, 0.0], atol=1e-6)


def test_kinematic_velocity_shape():
    """Returns a 3-vector."""
    v = leg_kinematic_velocity("FR", (0.0, 0.5, -1.0), (0.1, 0.2, -0.1))
    assert v.shape == (3,)
```

- [ ] **Step 3: Run tests to confirm they fail**

```bash
python3 -m pytest src/legged_control/tests/test_state_estimator.py -v
```

Expected: **FAIL** — `projected_gravity_from_quat` not yet defined.

- [ ] **Step 4: Run tests to confirm they pass**

```bash
python3 -m pytest src/legged_control/tests/test_state_estimator.py -v
```

Expected: all 7 tests **PASS**.

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/kinematics.py \
        src/legged_control/tests/test_state_estimator.py
git commit -m "feat: add kinematic velocity and attitude helpers to kinematics"
```

---

## Task 3: `state_estimator_node` — ROS node

**Files:**
- Create: `legged_control/state_estimator_node.py`

- [ ] **Step 1: Create the node**

Create `src/legged_control/legged_control/state_estimator_node.py`:

```python
"""state_estimator_node

Subscribes:
  odin1/imu/filtered  (sensor_msgs/Imu)   — orientation quaternion + angular_velocity
  /joint_states_aggregated (sensor_msgs/JointState) — motor-frame positions + velocities

Publishes:
  /state_estimate (std_msgs/Float32MultiArray, 9 floats)
    data[0:3] = base_lin_vel in yaw frame (m/s)
    data[3:6] = base_ang_vel in body frame (rad/s)
    data[6:9] = projected_gravity in body frame (unit vector)

Velocity estimation: complementary filter blending kinematic velocity
(assuming all four feet in contact) with IMU-integrated velocity.
Alpha = 0.8 (high trust in kinematics; adjust if drift is observed).
"""

from __future__ import annotations

import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from legged_control.kinematics import (
    leg_kinematic_velocity,
    projected_gravity_from_quat,
    yaw_rotation_matrix,
)

_LEG_ORDER = ("FL", "FR", "RL", "RR")

# robot.yaml canonical joint order
_YAML_JOINT_ORDER = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


def _load_joint_cfg(config_path: str) -> dict[str, dict]:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return {j["name"]: j for j in cfg["joints"]}


def _motor_to_urdf(q_motor: float, direction: float, zero_offset: float) -> float:
    return direction * q_motor + zero_offset


def _leg_q_urdf(
    leg: str, joint_pos: dict[str, float], joint_cfg: dict[str, dict]
) -> tuple[float, float, float] | None:
    names = [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]
    if any(n not in joint_pos for n in names):
        return None
    return tuple(
        _motor_to_urdf(
            joint_pos[n],
            float(joint_cfg[n]["direction"]),
            float(joint_cfg[n]["zero_offset"]),
        )
        for n in names
    )


def _leg_dq_urdf(
    leg: str, joint_vel: dict[str, float], joint_cfg: dict[str, dict]
) -> tuple[float, float, float] | None:
    names = [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]
    if any(n not in joint_vel for n in names):
        return None
    return tuple(
        float(joint_cfg[n]["direction"]) * joint_vel[n] for n in names
    )


class StateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_estimator_node")

        self.declare_parameter("config_path", "")
        config_path = str(self.get_parameter("config_path").value or "").strip()
        if not config_path:
            share = get_package_share_directory("legged_control")
            config_path = os.path.join(share, "config", "robot.yaml")
        self._joint_cfg = _load_joint_cfg(config_path)

        self.declare_parameter("velocity_alpha", 0.8)

        self._quat = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
        self._ang_vel = (0.0, 0.0, 0.0)
        self._lin_vel = np.zeros(3)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._imu_ready = False

        self._pub = self.create_publisher(Float32MultiArray, "/state_estimate", 10)
        self.create_subscription(Imu, "odin1/imu/filtered", self._on_imu, 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joints, 10
        )
        self.get_logger().info("state_estimator_node ready")

    def _on_imu(self, msg: Imu) -> None:
        o = msg.orientation
        self._quat = (o.x, o.y, o.z, o.w)
        av = msg.angular_velocity
        self._ang_vel = (av.x, av.y, av.z)
        self._imu_ready = True
        self._publish()

    def _on_joints(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _estimate_velocity(self) -> np.ndarray:
        alpha = float(self.get_parameter("velocity_alpha").value)
        R_yaw = yaw_rotation_matrix(*self._quat)
        kin_velocities = []
        for leg in _LEG_ORDER:
            q = _leg_q_urdf(leg, self._joint_pos, self._joint_cfg)
            dq = _leg_dq_urdf(leg, self._joint_vel, self._joint_cfg)
            if q is None or dq is None:
                continue
            v_body = leg_kinematic_velocity(leg, q, dq)
            kin_velocities.append(R_yaw @ v_body)
        if not kin_velocities:
            return self._lin_vel
        v_kin = np.mean(kin_velocities, axis=0)
        self._lin_vel = alpha * v_kin + (1.0 - alpha) * self._lin_vel
        return self._lin_vel

    def _publish(self) -> None:
        if not self._imu_ready:
            return
        lin_vel = self._estimate_velocity()
        ang_vel = np.array(self._ang_vel)
        proj_grav = projected_gravity_from_quat(*self._quat)

        msg = Float32MultiArray()
        msg.data = [
            float(lin_vel[0]), float(lin_vel[1]), float(lin_vel[2]),
            float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2]),
            float(proj_grav[0]), float(proj_grav[1]), float(proj_grav[2]),
        ]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

- [ ] **Step 2: Build and smoke-check**

```bash
colcon build --packages-select legged_control
source install/setup.bash
ros2 run legged_control state_estimator_node &
sleep 2 && ros2 topic list | grep state_estimate
kill %1
```

Expected: `/state_estimate` appears in topic list.

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/legged_control/state_estimator_node.py
git commit -m "feat: add state_estimator_node (attitude + contact velocity)"
```

---

## Task 4: `height_scan_node` — pure functions + node

**Files:**
- Create: `legged_control/height_scan_node.py`
- Create: `tests/test_height_scan.py`

### Background

Grid: x ∈ [0.10, 1.30], y ∈ [−0.30, 0.30], resolution 0.05 m → 25 × 13 = 325 cells.
Indexing: `idx = y_idx * 25 + x_idx` (x-major, x changes fastest).
Value: `height_scan[i] = -terrain_z_in_base_link` at grid cell i, clipped to [−1, 1].
- Flat ground at stance height (z ≈ −0.28 m in base_link) → value ≈ +0.28.
- Obstacle above ground → smaller positive or negative value.
- NaN cells (no depth hit) → default to 0.

The node requires a static TF `base_link → camera_link` to be published (set in launch file once camera mounting position is determined during hardware integration). Until then, use an identity transform (camera co-located with base_link) as placeholder.

- [ ] **Step 1: Write failing tests**

Create `src/legged_control/tests/test_height_scan.py`:

```python
import numpy as np
import pytest


def _build_height_scan(points_base_link: np.ndarray) -> np.ndarray:
    """Import target: legged_control.height_scan_node._build_height_scan"""
    from legged_control.height_scan_node import _build_height_scan as _f
    return _f(points_base_link)


def test_empty_point_cloud_returns_zeros():
    points = np.zeros((0, 3), dtype=np.float32)
    hs = _build_height_scan(points)
    assert hs.shape == (325,)
    np.testing.assert_array_equal(hs, np.zeros(325, dtype=np.float32))


def test_flat_ground_at_stance_height():
    """Ground plane at z=-0.28 → height_scan ≈ 0.28 everywhere."""
    xs = np.linspace(0.10, 1.30, 25)
    ys = np.linspace(-0.30, 0.30, 13)
    xv, yv = np.meshgrid(xs, ys)
    pts = np.stack([xv.ravel(), yv.ravel(), np.full(325, -0.28)], axis=1).astype(np.float32)
    hs = _build_height_scan(pts)
    np.testing.assert_allclose(hs, np.full(325, 0.28, dtype=np.float32), atol=1e-4)


def test_obstacle_clips_to_negative():
    """Point at z=+0.80 → height_scan = -0.80, clips to -1.0."""
    pts = np.array([[0.70, 0.0, 0.80]], dtype=np.float32)
    hs = _build_height_scan(pts)
    # Cell for x=0.70, y=0.00: x_idx=12, y_idx=6, idx=6*25+12=162
    assert hs[162] == pytest.approx(-1.0, abs=1e-4)


def test_out_of_range_points_ignored():
    """Points outside the grid footprint do not affect any cell."""
    pts = np.array([[0.0, 0.0, -0.3], [2.0, 0.0, -0.3]], dtype=np.float32)
    hs = _build_height_scan(pts)
    np.testing.assert_array_equal(hs, np.zeros(325, dtype=np.float32))


def test_multiple_hits_use_highest_point():
    """Multiple points in same cell → the one with largest z (highest) wins."""
    x, y = 0.50, 0.00
    pts = np.array([[x, y, -0.30], [x, y, -0.10]], dtype=np.float32)
    hs = _build_height_scan(pts)
    # x_idx=8, y_idx=6, idx=158
    assert hs[158] == pytest.approx(0.10, abs=1e-4)


def test_output_dtype_float32():
    hs = _build_height_scan(np.zeros((0, 3), dtype=np.float32))
    assert hs.dtype == np.float32
```

- [ ] **Step 2: Run to confirm they fail**

```bash
python3 -m pytest src/legged_control/tests/test_height_scan.py -v
```

Expected: **FAIL** — `height_scan_node` not yet created.

- [ ] **Step 3: Create `height_scan_node.py` with pure function + ROS node**

Create `src/legged_control/legged_control/height_scan_node.py`:

```python
"""height_scan_node

Subscribes:
  /camera/depth/image_rect_raw  (sensor_msgs/Image, 16UC1, mm units)
  /camera/depth/camera_info     (sensor_msgs/CameraInfo)

Publishes:
  /height_scan  (std_msgs/Float32MultiArray, 325 floats)

Height scan grid (in base_link frame):
  x: [0.10, 1.30] m (25 cols, step 0.05 m, positive = forward)
  y: [−0.30, 0.30] m (13 rows, step 0.05 m, positive = left)
  index = y_idx * 25 + x_idx  (x-major)
  value = -terrain_z_in_base_link, clipped to [−1, 1]
    positive ≈ ground (normal stance), negative ≈ raised obstacle

Requires a static TF base_link → camera_link.  During initial hardware
integration, confirm the TF is published by the launch file.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 — needed to register PointStamped transforms

_X_MIN, _X_MAX = 0.10, 1.30
_Y_MIN, _Y_MAX = -0.30, 0.30
_N_X, _N_Y = 25, 13
_RES = 0.05
_N_CELLS = _N_X * _N_Y  # 325


def _build_height_scan(points_base_link: np.ndarray) -> np.ndarray:
    """Compute 325-dim height scan from point cloud in base_link frame.

    Args:
        points_base_link: (N, 3) float32 array, columns = (x, y, z).

    Returns:
        (325,) float32 array, value = -z_terrain, clipped to [-1, 1].
        Empty cells default to 0.
    """
    grid = np.full(_N_CELLS, np.nan, dtype=np.float32)

    if len(points_base_link) == 0:
        return np.zeros(_N_CELLS, dtype=np.float32)

    xs, ys, zs = points_base_link[:, 0], points_base_link[:, 1], points_base_link[:, 2]

    # Filter to grid footprint
    mask = (xs >= _X_MIN) & (xs <= _X_MAX) & (ys >= _Y_MIN) & (ys <= _Y_MAX)
    xs, ys, zs = xs[mask], ys[mask], zs[mask]

    if len(xs) == 0:
        return np.zeros(_N_CELLS, dtype=np.float32)

    xi = np.clip(np.round((xs - _X_MIN) / _RES).astype(int), 0, _N_X - 1)
    yi = np.clip(np.round((ys - _Y_MIN) / _RES).astype(int), 0, _N_Y - 1)
    idx = yi * _N_X + xi

    # For each cell, keep the point with the highest z (closest to robot = most salient)
    for i, z in zip(idx, zs):
        if np.isnan(grid[i]) or z > grid[i]:
            grid[i] = z

    # NaN → 0 (unknown terrain, neutral)
    grid = np.where(np.isnan(grid), 0.0, grid)

    return np.clip(-grid, -1.0, 1.0).astype(np.float32)


def _deproject_pixel(
    u: np.ndarray, v: np.ndarray, z_m: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
) -> np.ndarray:
    """Backproject pixel (u, v) with depth z_m (metres) into camera-frame XYZ.

    Returns (N, 3) array.
    """
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return np.stack([x, y, z_m], axis=1)


class HeightScanNode(Node):
    def __init__(self) -> None:
        super().__init__("height_scan_node")

        self._fx = self._fy = self._cx = self._cy = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Float32MultiArray, "/height_scan", 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._on_info, 1)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._on_depth, 10)
        self.get_logger().info("height_scan_node ready — waiting for camera_info")

    def _on_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            K = msg.k
            self._fx, self._fy, self._cx, self._cy = K[0], K[4], K[2], K[5]
            self.get_logger().info(
                f"camera_info received: fx={self._fx:.1f} fy={self._fy:.1f}"
            )

    def _on_depth(self, msg: Image) -> None:
        if self._fx is None:
            return

        # Decode 16UC1 depth image (mm → m)
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        z_mm = depth.astype(np.float32)
        valid = z_mm > 0
        vs, us = np.where(valid)
        z_m = z_mm[valid] / 1000.0

        # Backproject to camera frame
        pts_cam = _deproject_pixel(
            us.astype(np.float32), vs.astype(np.float32), z_m,
            self._fx, self._fy, self._cx, self._cy,
        )

        # Transform to base_link frame via TF
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, rclpy.time.Time()
            )
        except Exception:
            return  # TF not yet available

        t = tf.transform.translation
        q = tf.transform.rotation
        translation = np.array([t.x, t.y, t.z])
        # Quaternion rotation matrix (camera → base_link)
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
        ])
        pts_base = (R @ pts_cam.T).T + translation

        hs = _build_height_scan(pts_base.astype(np.float32))
        out = Float32MultiArray()
        out.data = hs.tolist()
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = HeightScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

- [ ] **Step 4: Run tests**

```bash
colcon build --packages-select legged_control
source install/setup.bash
python3 -m pytest src/legged_control/tests/test_height_scan.py -v
```

Expected: all 6 tests **PASS**.

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/height_scan_node.py \
        src/legged_control/tests/test_height_scan.py
git commit -m "feat: add height_scan_node (D435 depth → 325-dim elevation grid)"
```

---

## Task 5: `policy_node` — joint mapping, obs assembly, action decode

**Files:**
- Create: `legged_control/policy_node.py` (data layer only — no ROS, no state machine yet)
- Create: `tests/test_policy_node.py`

- [ ] **Step 1: Write failing tests**

Create `src/legged_control/tests/test_policy_node.py`:

```python
import numpy as np
import pytest


# ── helpers that import from the module under test ────────────────────────────

def reorder_yaml_to_policy(yaml_vec):
    from legged_control.policy_node import _reorder_yaml_to_policy
    return _reorder_yaml_to_policy(np.array(yaml_vec, dtype=np.float32))


def reorder_policy_to_yaml(policy_vec):
    from legged_control.policy_node import _reorder_policy_to_yaml
    return _reorder_policy_to_yaml(np.array(policy_vec, dtype=np.float32))


def decode_action(action, q_default_urdf, action_scale, sign_flip_policy_idx, joint_cfg):
    from legged_control.policy_node import _decode_action
    return _decode_action(
        np.array(action, dtype=np.float32),
        np.array(q_default_urdf, dtype=np.float32),
        np.array(action_scale, dtype=np.float32),
        sign_flip_policy_idx,
        joint_cfg,
    )


# ── joint order ───────────────────────────────────────────────────────────────

_YAML_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]
_POLICY_NAMES = [
    "FL_hip", "FR_hip", "FL_thigh", "FR_thigh", "FL_calf", "FR_calf",
    "RL_hip", "RR_hip", "RL_thigh", "RR_thigh", "RL_calf", "RR_calf",
]


def test_reorder_yaml_to_policy_identity_check():
    """Each joint ends up at the correct policy index."""
    yaml_vec = np.arange(12, dtype=np.float32)
    policy_vec = reorder_yaml_to_policy(yaml_vec)
    for pi, pname in enumerate(_POLICY_NAMES):
        yi = _YAML_NAMES.index(pname)
        assert policy_vec[pi] == yaml_vec[yi], (
            f"policy[{pi}]={pname}: expected yaml[{yi}]={yaml_vec[yi]}, got {policy_vec[pi]}"
        )


def test_reorder_policy_to_yaml_identity_check():
    """policy → yaml → policy is identity."""
    original = np.random.rand(12).astype(np.float32)
    roundtrip = reorder_yaml_to_policy(reorder_policy_to_yaml(original))
    np.testing.assert_allclose(roundtrip, original, atol=1e-6)


def test_reorder_yaml_to_policy_shape():
    assert reorder_yaml_to_policy(np.zeros(12)).shape == (12,)


# ── action decode ──────────────────────────────────────────────────────────────

def _make_joint_cfg():
    """Minimal joint_cfg matching robot.yaml for FR leg (yaml indices 0-2)."""
    return {
        "FR_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "FR_thigh": {"direction":  1, "zero_offset": -1.254, "q_min": -2.0, "q_max": 2.0},
        "FR_calf":  {"direction": -1, "zero_offset":  2.791, "q_min": -4.5, "q_max": 4.5},
        "FL_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "FL_thigh": {"direction":  1, "zero_offset":  1.254, "q_min": -2.0, "q_max": 2.0},
        "FL_calf":  {"direction": -1, "zero_offset": -2.791, "q_min": -4.5, "q_max": 4.5},
        "RR_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "RR_thigh": {"direction":  1, "zero_offset": -1.254, "q_min": -2.0, "q_max": 2.0},
        "RR_calf":  {"direction": -1, "zero_offset":  2.791, "q_min": -4.5, "q_max": 4.5},
        "RL_hip":   {"direction":  1, "zero_offset":  0.000, "q_min": -0.5, "q_max": 0.5},
        "RL_thigh": {"direction":  1, "zero_offset":  1.254, "q_min": -2.0, "q_max": 2.0},
        "RL_calf":  {"direction": -1, "zero_offset": -2.791, "q_min": -4.5, "q_max": 4.5},
    }


def test_zero_action_gives_default_pose():
    """Zero action → motor targets equal q_default_motor for all joints."""
    cfg = _make_joint_cfg()
    # q_default in URDF frame, yaml order
    q_def_urdf = np.array([0.0, 0.7, -1.2] * 4, dtype=np.float32)
    scale = np.array([0.15, 0.20, 0.15] * 4, dtype=np.float32)
    action_policy = np.zeros(12, dtype=np.float32)  # policy order
    sign_flip = [1, 6]  # FR_hip and RL_hip in policy order

    q_motor = decode_action(action_policy, q_def_urdf, scale, sign_flip, cfg)
    assert q_motor.shape == (12,)

    # FR_hip (yaml[0]): direction=1, zero_offset=0, q_urdf=0.0 → q_motor=0.0
    assert q_motor[0] == pytest.approx(0.0, abs=1e-5)


def test_sign_flip_applied_to_fr_hip():
    """FR_hip action sign is flipped before decode."""
    cfg = _make_joint_cfg()
    q_def_urdf = np.zeros(12, dtype=np.float32)
    scale = np.ones(12, dtype=np.float32)
    action_policy = np.zeros(12, dtype=np.float32)
    action_policy[1] = 0.5  # FR_hip in policy order = 1

    q_motor_flipped = decode_action(action_policy, q_def_urdf, scale, [1, 6], cfg)
    q_motor_plain = decode_action(action_policy, q_def_urdf, scale, [], cfg)

    # FR_hip is yaml[0], direction=1, zero_offset=0
    # flipped: q_urdf = 0 + (-0.5)*1 = -0.5 → q_motor = -0.5
    # plain:   q_urdf = 0 + 0.5*1  = 0.5  → q_motor = 0.5
    assert q_motor_flipped[0] == pytest.approx(-0.5, abs=1e-5)
    assert q_motor_plain[0] == pytest.approx(0.5, abs=1e-5)


def test_output_clamped_to_q_limits():
    """Extreme action is clamped to q_min/q_max."""
    cfg = _make_joint_cfg()
    q_def_urdf = np.zeros(12, dtype=np.float32)
    scale = np.ones(12, dtype=np.float32) * 100.0  # huge scale forces clamping
    action = np.zeros(12, dtype=np.float32)
    q_motor = decode_action(action, q_def_urdf, scale, [], cfg)
    for i, name in enumerate(["FR_hip", "FR_thigh", "FR_calf",
                               "FL_hip", "FL_thigh", "FL_calf",
                               "RR_hip", "RR_thigh", "RR_calf",
                               "RL_hip", "RL_thigh", "RL_calf"]):
        assert cfg[name]["q_min"] <= q_motor[i] <= cfg[name]["q_max"]
```

- [ ] **Step 2: Run to confirm they fail**

```bash
python3 -m pytest src/legged_control/tests/test_policy_node.py -v
```

Expected: **FAIL** — `policy_node` not yet created.

- [ ] **Step 3: Create `policy_node.py` — data layer only**

Create `src/legged_control/legged_control/policy_node.py` with just the pure functions (full node added in Task 6):

```python
"""policy_node — RL policy deployment node.

Runs a TorchScript policy at 50 Hz with the same PASSIVE→STANDUP→WAIT→POLICY→LIE_DOWN
state machine as gait_node. The POLICY phase replaces IK trot with neural-network inference.

Observation vector (373 dims):
  [0:3]   base_lin_vel     from /state_estimate[0:3]
  [3:6]   base_ang_vel     from /state_estimate[3:6]
  [6:9]   projected_gravity from /state_estimate[6:9]
  [9:12]  velocity_commands [vx, vy, omega_z] from /cmd_vel
  [12:24] joint_pos_rel    q_motor - q_default_motor (yaml order → policy order)
  [24:36] joint_vel        dq_motor (yaml order → policy order)
  [36:48] last_action      previous raw policy output (policy order)
  [48:373] height_scan     from /height_scan (325 floats)

Action: 12-dim (policy order) joint position residuals.
  q_target_urdf = q_default_urdf + sign_flip * action * scale
  q_target_motor = direction * (q_target_urdf - zero_offset)
"""

from __future__ import annotations

import numpy as np

# ── Joint order constants ──────────────────────────────────────────────────────

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

_POLICY_JOINT_NAMES = [
    "FL_hip", "FR_hip", "FL_thigh", "FR_thigh", "FL_calf", "FR_calf",
    "RL_hip", "RR_hip", "RL_thigh", "RR_thigh", "RL_calf", "RR_calf",
]

# policy_vec[i] = yaml_vec[_YAML_TO_POLICY[i]]
_YAML_TO_POLICY = [
    _YAML_JOINT_NAMES.index(name) for name in _POLICY_JOINT_NAMES
]

# yaml_vec[j] = policy_vec[_POLICY_TO_YAML[j]]
_POLICY_TO_YAML = [
    _POLICY_JOINT_NAMES.index(name) for name in _YAML_JOINT_NAMES
]

# Policy indices for FR_hip and RL_hip (sign flip required on real hardware)
_DEFAULT_HIP_SIGN_FLIP_POLICY_IDX = [
    _POLICY_JOINT_NAMES.index("FR_hip"),
    _POLICY_JOINT_NAMES.index("RL_hip"),
]


def _reorder_yaml_to_policy(yaml_vec: np.ndarray) -> np.ndarray:
    """Reorder a 12-vector from yaml (robot.yaml) order to policy order."""
    return yaml_vec[_YAML_TO_POLICY]


def _reorder_policy_to_yaml(policy_vec: np.ndarray) -> np.ndarray:
    """Reorder a 12-vector from policy order to yaml (robot.yaml) order."""
    return policy_vec[_POLICY_TO_YAML]


def _decode_action(
    action_policy: np.ndarray,
    q_default_urdf_yaml: np.ndarray,
    action_scale_yaml: np.ndarray,
    sign_flip_policy_idx: list[int],
    joint_cfg: dict[str, dict],
) -> np.ndarray:
    """Decode policy action to motor-frame joint targets (yaml order).

    Args:
        action_policy:       (12,) raw action in policy joint order.
        q_default_urdf_yaml: (12,) policy default pose in URDF frame, yaml order.
        action_scale_yaml:   (12,) per-joint action scale, yaml order.
        sign_flip_policy_idx: policy-order indices where action must be negated
                              before decode (e.g. [FR_hip=1, RL_hip=6]).
        joint_cfg:           dict of joint configs from robot.yaml.

    Returns:
        (12,) float32 motor-frame target positions, yaml order, clamped to limits.
    """
    action = action_policy.copy()
    for idx in sign_flip_policy_idx:
        action[idx] *= -1.0

    # Reorder from policy to yaml order
    action_yaml = _reorder_policy_to_yaml(action)

    # Decode: q_urdf = q_default + action * scale
    q_target_urdf = q_default_urdf_yaml + action_yaml * action_scale_yaml

    # Convert to motor frame and clamp
    q_motor = np.empty(12, dtype=np.float32)
    for i, name in enumerate(_YAML_JOINT_NAMES):
        cfg = joint_cfg[name]
        direction = float(cfg["direction"])
        zero_offset = float(cfg["zero_offset"])
        q_m = direction * (float(q_target_urdf[i]) - zero_offset)
        q_motor[i] = float(np.clip(q_m, float(cfg["q_min"]), float(cfg["q_max"])))

    return q_motor


def _assemble_obs(
    state_estimate: np.ndarray,
    cmd_vel: tuple[float, float, float],
    joint_pos_motor_yaml: np.ndarray,
    joint_vel_motor_yaml: np.ndarray,
    q_default_motor_yaml: np.ndarray,
    last_action_policy: np.ndarray,
    height_scan: np.ndarray,
) -> np.ndarray:
    """Assemble 373-dim observation vector.

    All inputs in yaml order where joint-indexed; output is raw float32 numpy array.
    """
    joint_pos_rel_policy = _reorder_yaml_to_policy(
        joint_pos_motor_yaml - q_default_motor_yaml
    )
    joint_vel_policy = _reorder_yaml_to_policy(joint_vel_motor_yaml)

    obs = np.concatenate([
        state_estimate[:9],           # lin_vel, ang_vel, proj_grav
        np.array(cmd_vel, dtype=np.float32),
        joint_pos_rel_policy,
        joint_vel_policy,
        last_action_policy,
        height_scan,
    ])
    return obs.astype(np.float32)
```

- [ ] **Step 4: Run tests**

```bash
colcon build --packages-select legged_control
source install/setup.bash
python3 -m pytest src/legged_control/tests/test_policy_node.py -v
```

Expected: all tests **PASS**.

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/policy_node.py \
        src/legged_control/tests/test_policy_node.py
git commit -m "feat: add policy_node data layer (joint mapping, obs assembly, action decode)"
```

---

## Task 6: `policy_node` — state machine + ROS node

**Files:**
- Modify: `legged_control/policy_node.py` (append full `PolicyNode` class + `main`)
- Create: `config/policy.yaml`

- [ ] **Step 1: Create `config/policy.yaml`**

Create `src/legged_control/config/policy.yaml`:

```yaml
policy:
  # Absolute path to TorchScript .pt file. Override at launch with:
  #   ros2 launch legged_control robot.launch.py mode:=policy model_path:=/path/to/policy.pt
  model_path: ""

  # Default stance pose in URDF frame (training convention), yaml joint order.
  # These values come from the training config (hip=0.0, thigh=0.7, calf=-1.2).
  # IMPORTANT: verify against hardware in passive mode before first policy run.
  joint_default_q_urdf:
    FR_hip:    0.0
    FR_thigh:  0.7
    FR_calf:  -1.2
    FL_hip:    0.0
    FL_thigh:  0.7
    FL_calf:  -1.2
    RR_hip:    0.0
    RR_thigh:  0.7
    RR_calf:  -1.2
    RL_hip:    0.0
    RL_thigh:  0.7
    RL_calf:  -1.2

  # Per-joint action scale, yaml order
  action_scale:
    FR_hip:   0.15
    FR_thigh: 0.20
    FR_calf:  0.15
    FL_hip:   0.15
    FL_thigh: 0.20
    FL_calf:  0.15
    RR_hip:   0.15
    RR_thigh: 0.20
    RR_calf:  0.15
    RL_hip:   0.15
    RL_thigh: 0.20
    RL_calf:  0.15

  # Joints where the training URDF sign differs from deployment URDF.
  # These are negated before action decode.
  hip_sign_flip: [FR_hip, RL_hip]
```

- [ ] **Step 2: Append full ROS node to `policy_node.py`**

Append to the end of `src/legged_control/legged_control/policy_node.py`:

```python
# ── ROS node ──────────────────────────────────────────────────────────────────

import math
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
import rclpy.parameter
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from legged_control.kinematics import _smoothstep

_PHASE_PASSIVE = "PASSIVE"
_PHASE_STANDUP = "STANDUP"
_PHASE_WAIT    = "WAIT"
_PHASE_POLICY  = "POLICY"
_PHASE_LIEDOWN = "LIEDOWN"
_PHASE_FAULT   = "FAULT"

_STANDUP_TOL = 0.05   # rad — near-target threshold after standup ramp
_LIEDOWN_TOL = 0.03   # rad — near-zero threshold before returning to passive
_VEL_SETTLED = 0.05   # rad/s — settled velocity threshold


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_node")

        self.declare_parameter("config_path", "")
        self.declare_parameter("policy_config_path", "")
        self.declare_parameter("model_path", "")

        cfg = self._load_robot_cfg()
        policy_cfg = self._load_policy_cfg()

        self._joint_cfg = {j["name"]: j for j in cfg["joints"]}
        self._joint_names_yaml = [j["name"] for j in cfg["joints"]]

        # Build per-joint arrays (yaml order)
        self._q_default_urdf = self._build_q_default_urdf(policy_cfg)
        self._action_scale = self._build_action_scale(policy_cfg)
        self._sign_flip_policy_idx = self._build_sign_flip(policy_cfg)
        self._q_default_motor = self._urdf_to_motor(self._q_default_urdf)

        control_cfg = cfg["control"]
        standup_cfg = cfg.get("standup", {})
        kp = float(control_cfg["kp"])
        kd = float(control_cfg["kd"])
        self.declare_parameter("kp", kp)
        self.declare_parameter("kd", kd)
        self.declare_parameter("ramp_duration", float(standup_cfg.get("ramp_duration", 8.0)))
        self.declare_parameter("lie_down_duration", float(standup_cfg.get("lie_down_duration", 2.0)))

        loop_hz = float(control_cfg.get("gait_hz", 50.0))
        self._dt = 1.0 / loop_hz

        # Load policy model
        model_path = str(self.get_parameter("model_path").value or "").strip()
        if not model_path:
            model_path = str(policy_cfg.get("model_path", "") or "")
        self._policy = None
        if model_path:
            try:
                import torch
                self._policy = torch.jit.load(model_path)
                self._policy.eval()
                self.get_logger().info(f"[policy] loaded model: {model_path}")
            except Exception as e:
                self.get_logger().error(f"[policy] failed to load model {model_path}: {e}")

        # State
        self._phase = _PHASE_PASSIVE
        self._phase_start: float | None = None
        self._stand_requested = False
        self._lie_down_start: list[float] | None = None
        self._last_published: list[float] | None = None
        self._last_action = np.zeros(12, dtype=np.float32)
        self._passive_broadcast = False
        self._fault_broadcast = False

        # Sensor buffers
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._state_estimate = np.zeros(9, dtype=np.float32)
        self._height_scan = np.zeros(325, dtype=np.float32)
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._joint_state_seen = False

        self._gain_clients = [
            self.create_client(SetParameters, "/motor_bus_front/set_parameters"),
            self.create_client(SetParameters, "/motor_bus_rear/set_parameters"),
        ]

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32MultiArray, "/height_scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/posture_command", self._on_posture, 10)
        self.add_on_set_parameters_callback(self._on_gains_changed)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"policy_node ready — {loop_hz:.0f} Hz  "
            f"model={'loaded' if self._policy else 'NOT LOADED'}  "
            f"kp={kp}  kd={kd}"
        )

    # ── Config loading ─────────────────────────────────────────────────────────

    def _load_robot_cfg(self) -> dict:
        share = get_package_share_directory("legged_control")
        path = str(self.get_parameter("config_path").value or "").strip()
        if not path:
            path = os.path.join(share, "config", "robot.yaml")
        with open(path) as f:
            return yaml.safe_load(f)

    def _load_policy_cfg(self) -> dict:
        share = get_package_share_directory("legged_control")
        path = str(self.get_parameter("policy_config_path").value or "").strip()
        if not path:
            path = os.path.join(share, "config", "policy.yaml")
        with open(path) as f:
            return yaml.safe_load(f).get("policy", {})

    def _build_q_default_urdf(self, pcfg: dict) -> np.ndarray:
        m = pcfg.get("joint_default_q_urdf", {})
        return np.array([float(m.get(n, 0.0)) for n in _YAML_JOINT_NAMES], dtype=np.float32)

    def _build_action_scale(self, pcfg: dict) -> np.ndarray:
        m = pcfg.get("action_scale", {})
        return np.array([float(m.get(n, 0.1)) for n in _YAML_JOINT_NAMES], dtype=np.float32)

    def _build_sign_flip(self, pcfg: dict) -> list[int]:
        flip_names = pcfg.get("hip_sign_flip", [])
        result = []
        for name in flip_names:
            if name in _POLICY_JOINT_NAMES:
                result.append(_POLICY_JOINT_NAMES.index(name))
        return result

    def _urdf_to_motor(self, q_urdf_yaml: np.ndarray) -> np.ndarray:
        out = np.empty(12, dtype=np.float32)
        for i, name in enumerate(_YAML_JOINT_NAMES):
            cfg = self._joint_cfg[name]
            out[i] = float(cfg["direction"]) * (float(q_urdf_yaml[i]) - float(cfg["zero_offset"]))
        return out

    # ── Subscribers ────────────────────────────────────────────────────────────

    def _on_joints(self, msg: JointState) -> None:
        self._joint_state_seen = True
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joint_pos[name] = float(pos)
            self._joint_vel[name] = float(vel)

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_estimate = np.array(msg.data[:9], dtype=np.float32)

    def _on_scan(self, msg: Float32MultiArray) -> None:
        self._height_scan = np.array(msg.data[:325], dtype=np.float32)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _on_posture(self, msg: Bool) -> None:
        if self._phase == _PHASE_FAULT:
            return
        if bool(msg.data):
            if self._phase == _PHASE_PASSIVE:
                self._stand_requested = True
        else:
            if self._phase in (_PHASE_WAIT, _PHASE_POLICY, _PHASE_STANDUP):
                self._phase = _PHASE_LIEDOWN
                self._phase_start = time.monotonic()
                self._lie_down_start = list(self._last_published or self._q_default_motor.tolist())

    # ── Gain broadcast ─────────────────────────────────────────────────────────

    def _broadcast_gains(self, kp: float, kd: float) -> None:
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name="kp", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp))),
            Parameter(name="kd", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd))),
        ]
        for client in self._gain_clients:
            if client.service_is_ready():
                client.call_async(req)

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        new_kp = next((p.value for p in params if p.name == "kp"), None)
        new_kd = next((p.value for p in params if p.name == "kd"), None)
        if new_kp is not None or new_kd is not None:
            kp = new_kp or self.get_parameter("kp").value
            kd = new_kd or self.get_parameter("kd").value
            self._broadcast_gains(float(kp), float(kd))
        return SetParametersResult(successful=True)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _current_pos(self) -> list[float] | None:
        vals = [self._joint_pos.get(n) for n in self._joint_names_yaml]
        if any(v is None for v in vals):
            return None
        return [float(v) for v in vals]

    def _current_vel(self) -> list[float] | None:
        vals = [self._joint_vel.get(n) for n in self._joint_names_yaml]
        if any(v is None for v in vals):
            return None
        return [float(v) for v in vals]

    def _is_near(self, targets: list[float], tol: float) -> bool:
        pos = self._current_pos()
        if pos is None:
            return False
        return all(abs(p - t) <= tol for p, t in zip(pos, targets))

    def _is_settled(self) -> bool:
        vel = self._current_vel()
        if vel is None:
            return False
        return all(abs(v) <= _VEL_SETTLED for v in vel)

    def _publish(self, positions: list[float]) -> None:
        self._last_published = list(positions)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names_yaml)
        msg.position = positions
        self._pub.publish(msg)

    # ── Phase implementations ──────────────────────────────────────────────────

    def _standup_targets(self, elapsed: float) -> tuple[list[float], bool]:
        ramp = max(float(self.get_parameter("ramp_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / ramp)
        done = elapsed >= ramp
        targets = [alpha * q for q in self._q_default_motor.tolist()]
        return targets, done

    def _liedown_targets(self, elapsed: float) -> tuple[list[float], bool]:
        dur = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / dur)
        start = self._lie_down_start or self._q_default_motor.tolist()
        targets = [(1.0 - alpha) * s for s in start]
        return targets, elapsed >= dur

    def _run_inference(self) -> list[float]:
        pos = self._current_pos()
        vel = self._current_vel()
        if pos is None or vel is None or self._policy is None:
            return self._q_default_motor.tolist()

        pos_arr = np.array(pos, dtype=np.float32)
        vel_arr = np.array(vel, dtype=np.float32)
        obs = _assemble_obs(
            self._state_estimate,
            self._cmd_vel,
            pos_arr,
            vel_arr,
            self._q_default_motor,
            self._last_action,
            self._height_scan,
        )
        try:
            import torch
            with torch.inference_mode():
                obs_t = torch.from_numpy(obs).unsqueeze(0)
                action = self._policy(obs_t).squeeze(0).numpy()
        except Exception as e:
            self.get_logger().error(f"[policy] inference error: {e}", throttle_duration_sec=1.0)
            return self._q_default_motor.tolist()

        self._last_action = action.copy()
        q_motor = _decode_action(
            action,
            self._q_default_urdf,
            self._action_scale,
            self._sign_flip_policy_idx,
            self._joint_cfg,
        )
        return q_motor.tolist()

    # ── Main tick ──────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()

        if self._phase == _PHASE_PASSIVE:
            if self._stand_requested:
                self._broadcast_gains(
                    float(self.get_parameter("kp").value),
                    float(self.get_parameter("kd").value),
                )
                self._phase = _PHASE_STANDUP
                self._phase_start = now
                self._last_published = None
                self._last_action = np.zeros(12, dtype=np.float32)
                self._stand_requested = False
                self._passive_broadcast = False
                self.get_logger().info("[policy] posture=true → STANDUP")
            else:
                if not self._passive_broadcast:
                    self._broadcast_gains(0.0, 0.0)
                    self._passive_broadcast = True
            return

        if self._phase_start is None:
            self._phase_start = now
        elapsed = now - self._phase_start

        if self._phase == _PHASE_STANDUP:
            targets, done = self._standup_targets(elapsed)
            self._publish(targets)
            if done and self._is_near(self._q_default_motor.tolist(), _STANDUP_TOL) and self._is_settled():
                self._phase = _PHASE_WAIT
                self._phase_start = now
                self.get_logger().info("[policy] standup complete → WAIT")
            elif done and elapsed > float(self.get_parameter("ramp_duration").value) + 5.0:
                self._phase = _PHASE_WAIT
                self._phase_start = now
                self.get_logger().warn("[policy] standup timeout → WAIT")
            return

        if self._phase == _PHASE_WAIT:
            self._publish(self._q_default_motor.tolist())
            if self._joint_state_seen and any(abs(v) > 1e-4 for v in self._cmd_vel):
                self._phase = _PHASE_POLICY
                self._phase_start = now
                self.get_logger().info("[policy] cmd_vel received → POLICY")
            return

        if self._phase == _PHASE_POLICY:
            if all(abs(v) <= 1e-4 for v in self._cmd_vel):
                self._phase = _PHASE_WAIT
                self._phase_start = now
                return
            targets = self._run_inference()
            self._publish(targets)
            return

        if self._phase == _PHASE_LIEDOWN:
            targets, done = self._liedown_targets(elapsed)
            self._publish(targets)
            if done and self._is_near([0.0] * 12, _LIEDOWN_TOL) and self._is_settled():
                self._phase = _PHASE_PASSIVE
                self._phase_start = None
                self._passive_broadcast = False
                self._last_action = np.zeros(12, dtype=np.float32)
                self.get_logger().info("[policy] liedown complete → PASSIVE")
            return

        if self._phase == _PHASE_FAULT:
            if not self._fault_broadcast:
                self._broadcast_gains(0.5, 0.1)
                self._fault_broadcast = True
            self._publish(self._last_published or self._q_default_motor.tolist())


def main() -> None:
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

- [ ] **Step 3: Build and verify unit tests still pass**

```bash
colcon build --packages-select legged_control
source install/setup.bash
python3 -m pytest src/legged_control/tests/test_policy_node.py -v
```

Expected: all tests **PASS** (the pure functions are unchanged).

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/legged_control/policy_node.py \
        src/legged_control/config/policy.yaml
git commit -m "feat: add policy_node state machine and ROS node with TorchScript inference"
```

---

## Task 7: Extended `passive_monitor_node` + launch integration

**Files:**
- Modify: `legged_control/passive_monitor_node.py`
- Modify: `launch/robot.launch.py`
- Create: `launch/gazebo_policy.launch.py`
- Modify: `setup.py` (data_files)

### Part A — Extended passive monitor

- [ ] **Step 1: Rewrite `passive_monitor_node.py`**

Replace the contents of `src/legged_control/legged_control/passive_monitor_node.py`:

```python
"""passive_monitor_node

Subscribes to aggregated data and displays the full RL observation space
in the terminal at 2 Hz. Used for sensor calibration and data-link verification.

Display sections:
  JOINTS   — pos_rel (vs policy q_default) and velocity for all 12 joints
  IMU      — base_lin_vel, base_ang_vel, projected_gravity
  COMMAND  — vx, vy, omega_z from /cmd_vel
  HEIGHT   — height_scan mean and stddev (full 325-dim values not printed)
"""

from __future__ import annotations

import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]


class PassiveMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("passive_monitor_node")

        self.declare_parameter("policy_config_path", "")
        share = get_package_share_directory("legged_control")

        # Load policy q_default for joint_pos_rel display
        pcfg_path = str(self.get_parameter("policy_config_path").value or "").strip()
        if not pcfg_path:
            pcfg_path = os.path.join(share, "config", "policy.yaml")
        try:
            with open(pcfg_path) as f:
                pcfg = yaml.safe_load(f).get("policy", {})
            qd = pcfg.get("joint_default_q_urdf", {})
            self._q_default = np.array([float(qd.get(n, 0.0)) for n in _YAML_JOINT_NAMES])
        except Exception:
            self._q_default = np.zeros(12)

        self._joint_pos = np.full(12, float("nan"))
        self._joint_vel = np.full(12, float("nan"))
        self._state_estimate = np.zeros(9)
        self._height_scan = np.zeros(325)
        self._cmd_vel = (0.0, 0.0, 0.0)

        self.create_subscription(JointState, "/joint_states_aggregated", self._on_joints, 10)
        self.create_subscription(Float32MultiArray, "/state_estimate", self._on_state, 10)
        self.create_subscription(Float32MultiArray, "/height_scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        try:
            self._tty = open("/dev/tty", "w")
        except OSError:
            self._tty = None

        self.create_timer(0.5, self._display)
        self.get_logger().info("passive_monitor_node ready")

    def _on_joints(self, msg: JointState) -> None:
        pos_map = dict(zip(msg.name, msg.position))
        vel_map = dict(zip(msg.name, msg.velocity))
        for i, n in enumerate(_YAML_JOINT_NAMES):
            if n in pos_map:
                self._joint_pos[i] = float(pos_map[n])
            if n in vel_map:
                self._joint_vel[i] = float(vel_map[n])

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._state_estimate = np.array(msg.data[:9])

    def _on_scan(self, msg: Float32MultiArray) -> None:
        self._height_scan = np.array(msg.data[:325])

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _display(self) -> None:
        lv = self._state_estimate[0:3]
        av = self._state_estimate[3:6]
        pg = self._state_estimate[6:9]
        vx, vy, wz = self._cmd_vel
        pos_rel = self._joint_pos - self._q_default

        lines = [
            "┌─── PASSIVE MONITOR ──────────────────────────────────────┐",
            "│ JOINTS       pos_rel(rad)   vel(rad/s)                   │",
        ]
        for i, name in enumerate(_YAML_JOINT_NAMES):
            lines.append(f"│  {name:<12}  {pos_rel[i]:+7.3f}       {self._joint_vel[i]:+7.3f}            │")
        lines += [
            "│ IMU                                                      │",
            f"│  lin_vel    vx={lv[0]:+6.3f}  vy={lv[1]:+6.3f}  vz={lv[2]:+6.3f}     │",
            f"│  ang_vel    wx={av[0]:+6.3f}  wy={av[1]:+6.3f}  wz={av[2]:+6.3f}     │",
            f"│  proj_grav  gx={pg[0]:+6.3f}  gy={pg[1]:+6.3f}  gz={pg[2]:+6.3f}     │",
            "│ COMMAND                                                  │",
            f"│  vx={vx:+6.3f}  vy={vy:+6.3f}  wz={wz:+6.3f}                   │",
            "│ HEIGHT SCAN                                              │",
            f"│  mean={np.nanmean(self._height_scan):+6.3f}  std={np.nanstd(self._height_scan):5.3f}  "
            f"min={np.nanmin(self._height_scan):+6.3f}  max={np.nanmax(self._height_scan):+6.3f}  │",
            "└──────────────────────────────────────────────────────────┘",
        ]
        text = "\n".join(lines)
        if self._tty:
            self._tty.write(f"\033[2J\033[H{text}\n")
            self._tty.flush()
        else:
            print(text, flush=True)


def main() -> None:
    rclpy.init()
    node = PassiveMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

### Part B — Launch file updates

- [ ] **Step 2: Update `robot.launch.py` — replace stand/standup modes with passive/policy**

In `src/legged_control/launch/robot.launch.py`, replace the `_launch_setup` function body. The four mode branches should be:

```python
if mode == "passive":
    motors = _bus_nodes(joints, port_map, motor_hz, kp=0.0, kd=0.0)
    return motors + [
        Node(package="legged_control", executable="joint_aggregator",
             name="joint_aggregator", output="screen"),
        Node(package="odin_ros_driver", executable="odin1_node",
             name="odin1_node", output="log"),
        Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
             name="imu_filter_madgwick",
             parameters=[{"use_mag": False, "publish_tf": False,
                          "fixed_frame": "base_link", "world_frame": "enu"}],
             remappings=[("imu/data_raw", "odin1/imu"),
                         ("imu/data", "odin1/imu/filtered")],
             output="log"),
        Node(package="realsense2_camera", executable="realsense2_camera_node",
             name="camera", output="log"),
        Node(package="legged_control", executable="state_estimator_node",
             name="state_estimator_node", output="screen"),
        Node(package="legged_control", executable="height_scan_node",
             name="height_scan_node", output="screen"),
        Node(package="joy", executable="joy_node", name="joy_node", output="log"),
        Node(package="legged_control", executable="teleop_node",
             name="teleop_node", output="screen"),
        Node(package="legged_control", executable="passive_monitor_node",
             name="passive_monitor_node", output="screen"),
    ]

if mode == "position_control":
    # ... (existing code, unchanged)

if mode == "policy":
    kp = float(control["kp"])
    kd = float(control["kd"])
    motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
    return motors + [
        Node(package="legged_control", executable="joint_aggregator",
             name="joint_aggregator", output="screen"),
        Node(package="odin_ros_driver", executable="odin1_node",
             name="odin1_node", output="log"),
        Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
             name="imu_filter_madgwick",
             parameters=[{"use_mag": False, "publish_tf": False,
                          "fixed_frame": "base_link", "world_frame": "enu"}],
             remappings=[("imu/data_raw", "odin1/imu"),
                         ("imu/data", "odin1/imu/filtered")],
             output="log"),
        Node(package="realsense2_camera", executable="realsense2_camera_node",
             name="camera", output="log"),
        Node(package="legged_control", executable="state_estimator_node",
             name="state_estimator_node", output="screen"),
        Node(package="legged_control", executable="height_scan_node",
             name="height_scan_node", output="screen"),
        Node(package="joy", executable="joy_node", name="joy_node", output="log"),
        Node(package="legged_control", executable="teleop_node",
             name="teleop_node", output="screen"),
        Node(package="legged_control", executable="policy_node",
             name="policy_node",
             parameters=[{"model_path": LaunchConfiguration("model_path")}],
             output="screen"),
    ]
```

Also add a `DeclareLaunchArgument` for `model_path`:
```python
DeclareLaunchArgument("model_path", default_value="",
                      description="Path to TorchScript .pt policy file"),
```

And update the valid modes error message:
```python
raise RuntimeError(
    f"Unknown mode '{mode}'. Valid modes: passive, stand, standup, position_control, policy, simulation"
)
```

- [ ] **Step 3: Create `launch/gazebo_policy.launch.py`**

Create `src/legged_control/launch/gazebo_policy.launch.py`:

```python
"""Run RL policy against the Gazebo physics simulation.

The URDF must include:
  - IMU sensor plugin publishing sensor_msgs/Imu on 'odin1/imu'
  - Depth camera plugin publishing sensor_msgs/Image on /camera/depth/image_rect_raw
    and sensor_msgs/CameraInfo on /camera/depth/camera_info

Same software stack as real-hardware policy mode:
  state_estimator_node + height_scan_node + policy_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_SIM_URDF = "/home/grayerd/Desktop/Projects/rc/塞北箭4urdf/urdf/塞北箭4_sim.urdf"
_POSTURE_CMD_DELAY = 2.0


def _ros2_pub_once(topic, msg_type, data):
    return ["zsh", "-lc",
            f"source /opt/ros/humble/setup.zsh && "
            f"ros2 topic pub --once {topic} {msg_type} '{data}'"]


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    config_path = os.path.join(share, "config", "robot.yaml")
    physics_launch = os.path.join(share, "launch", "gazebo_physics.launch.py")
    model_path = LaunchConfiguration("model_path").perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physics_launch),
            launch_arguments={
                "urdf_path": LaunchConfiguration("urdf_path"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "start_paused": "false",
            }.items(),
        ),
        Node(package="legged_control", executable="gazebo_control_bridge",
             name="gazebo_control_bridge",
             parameters=[{"config_path": config_path}], output="screen"),
        Node(package="imu_filter_madgwick", executable="imu_filter_madgwick_node",
             name="imu_filter_madgwick",
             parameters=[{"use_mag": False, "publish_tf": False,
                          "fixed_frame": "base_link", "world_frame": "enu"}],
             remappings=[("imu/data_raw", "odin1/imu"),
                         ("imu/data", "odin1/imu/filtered")],
             output="log"),
        Node(package="legged_control", executable="state_estimator_node",
             name="state_estimator_node",
             parameters=[{"config_path": config_path}], output="screen"),
        Node(package="legged_control", executable="height_scan_node",
             name="height_scan_node", output="screen"),
        Node(package="joy", executable="joy_node", name="joy_node", output="log"),
        Node(package="legged_control", executable="teleop_node",
             name="teleop_node", output="screen"),
        Node(package="legged_control", executable="policy_node",
             name="policy_node",
             parameters=[{"config_path": config_path, "model_path": model_path}],
             output="screen"),
        TimerAction(period=_POSTURE_CMD_DELAY, actions=[
            ExecuteProcess(
                cmd=_ros2_pub_once("/posture_command", "std_msgs/msg/Bool", "{data: true}"),
                output="screen",
            )
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=_DEFAULT_SIM_URDF),
        DeclareLaunchArgument("spawn_z", default_value="1.80"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="Path to TorchScript .pt policy file"),
        OpaqueFunction(function=_launch_setup),
    ])
```

- [ ] **Step 4: Update `setup.py` data_files**

In `src/legged_control/setup.py`, update the config list:
```python
("share/" + package_name + "/config",
 ["config/robot.yaml", "config/robot_sim.yaml",
  "config/position_control_sim.rviz", "config/policy.yaml"]),
```

Update the launch list:
```python
("share/" + package_name + "/launch",
 ["launch/robot.launch.py",
  "launch/position_control_sim.launch.py",
  "launch/gazebo_physics.launch.py",
  "launch/gazebo_position_control.launch.py",
  "launch/gazebo_policy.launch.py"]),
```

- [ ] **Step 5: Build and run full test suite**

```bash
colcon build --packages-select legged_control
source install/setup.bash
python3 -m pytest src/legged_control/tests/ -v -x
```

Expected: all tests **PASS** (deleted-node tests are gone; remaining tests unaffected).

- [ ] **Step 6: Verify passive mode launches without errors (dry run)**

```bash
ros2 launch legged_control robot.launch.py mode:=passive --dry-run
```

Expected: launch description printed without errors (nodes listed; hardware not required for dry-run).

- [ ] **Step 7: Final commit**

```bash
git add src/legged_control/
git commit -m "feat: add passive/policy/simulation modes with full obs pipeline and launch integration"
```

---

## Post-Implementation Hardware Checklist

Run these in order on the physical robot before running `policy` mode:

1. `mode:=passive` — verify all 12 joint angles display correctly; compare with URDF in RViz
2. `mode:=passive` — tilt robot, verify `projected_gravity` direction matches tilt
3. `mode:=passive` — push robot forward, verify `base_lin_vel[0]` shows positive value
4. `mode:=passive` — display height scan mean; confirm D435 is seeing the ground ahead
5. `mode:=passive` — move gamepad sticks, verify `vx/vy/wz` respond correctly
6. Only then: `mode:=policy model_path:=/path/to/v0.1-flat-pretrain/policy.pt`
