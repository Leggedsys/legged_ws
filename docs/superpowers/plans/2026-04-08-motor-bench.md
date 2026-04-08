# Motor Bench Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a single-leg joystick position test node that lets the operator sweep each of the three joints through its configured range to verify motion.

**Architecture:** A new `motor_bench_node` subscribes to `/joy` and `/‹ns›/joint_states`, maps left-stick vertical to the active joint's `[q_min, q_max]`, and publishes `/joint_commands` at 50 Hz. B/X buttons cycle through the three joints; switching locks the departing joint at its last feedback position. A new launch file brings up 3 motor nodes + joy_node + motor_bench_node for the selected leg.

**Tech Stack:** Python 3, ROS2 Humble, `sensor_msgs/Joy`, `sensor_msgs/JointState`, `launch_ros`, pytest

---

## File Map

| Action | Path | Responsibility |
|--------|------|---------------|
| Create | `src/legged_control/legged_control/motor_bench_node.py` | Joystick → `/joint_commands` |
| Create | `src/legged_control/launch/motor_bench.launch.py` | Start 3 motor nodes + joy + bench node |
| Create | `src/legged_control/tests/test_motor_bench_node.py` | Unit tests for pure helper functions |
| Modify | `src/legged_control/setup.py` | Register new entry point and launch file |

---

## Task 1: Create git branch

**Files:** none

- [ ] **Step 1: Create and switch to the new branch**

```bash
git checkout -b dev/motor-bench
```

Expected output:
```
Switched to a new branch 'dev/motor-bench'
```

---

## Task 2: Write and test pure helper functions

These three functions contain all the logic worth unit-testing. They have no ROS2 dependency.

**Files:**
- Create: `src/legged_control/legged_control/motor_bench_node.py` (stub — pure functions only)
- Create: `src/legged_control/tests/test_motor_bench_node.py`

- [ ] **Step 1: Write the failing tests**

Create `src/legged_control/tests/test_motor_bench_node.py`:

```python
import math
import pytest
from legged_control.motor_bench_node import (
    _apply_deadzone,
    _map_to_range,
    _select_joints,
)

# --- _apply_deadzone ---

def test_deadzone_zero_input():
    assert _apply_deadzone(0.0) == 0.0

def test_deadzone_inside_zone():
    assert _apply_deadzone(0.04) == 0.0
    assert _apply_deadzone(-0.04) == 0.0

def test_deadzone_at_boundary():
    assert _apply_deadzone(0.05) == 0.0   # exactly at edge → still zero

def test_deadzone_full_positive():
    result = _apply_deadzone(1.0)
    assert math.isclose(result, 1.0, rel_tol=1e-6)

def test_deadzone_full_negative():
    result = _apply_deadzone(-1.0)
    assert math.isclose(result, -1.0, rel_tol=1e-6)

def test_deadzone_rescales_above_boundary():
    # Just above deadzone (0.05) should give a small positive value, not jump to 1
    result = _apply_deadzone(0.10)
    assert 0.0 < result < 0.1

def test_deadzone_custom_dz():
    assert _apply_deadzone(0.09, dz=0.10) == 0.0
    result = _apply_deadzone(0.20, dz=0.10)
    assert result > 0.0

# --- _map_to_range ---

def test_map_range_min():
    assert math.isclose(_map_to_range(-1.0, -1.047, 1.047), -1.047, rel_tol=1e-6)

def test_map_range_max():
    assert math.isclose(_map_to_range(1.0, -1.047, 1.047), 1.047, rel_tol=1e-6)

def test_map_range_midpoint():
    result = _map_to_range(0.0, -1.047, 1.047)
    assert math.isclose(result, 0.0, abs_tol=1e-6)

def test_map_range_asymmetric():
    # calf: q_min=-2.723, q_max=-0.837
    result = _map_to_range(-1.0, -2.723, -0.837)
    assert math.isclose(result, -2.723, rel_tol=1e-6)
    result = _map_to_range(1.0, -2.723, -0.837)
    assert math.isclose(result, -0.837, rel_tol=1e-6)

# --- _select_joints ---

JOINTS_CFG = [
    {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'FR_calf',  'motor_id': 2, 'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
    {'name': 'FL_hip',   'motor_id': 3, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'FL_thigh', 'motor_id': 4, 'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'FL_calf',  'motor_id': 5, 'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
    {'name': 'RR_hip',   'motor_id': 6, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'RR_thigh', 'motor_id': 7, 'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'RR_calf',  'motor_id': 8, 'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
    {'name': 'RL_hip',   'motor_id': 9, 'default_q': 0.0,  'q_min': -1.047, 'q_max': 1.047},
    {'name': 'RL_thigh', 'motor_id': 10,'default_q': 0.8,  'q_min': -1.571, 'q_max': 3.927},
    {'name': 'RL_calf',  'motor_id': 11,'default_q': -1.5, 'q_min': -2.723, 'q_max': -0.837},
]

def test_select_joints_fr_uppercase():
    result = _select_joints(JOINTS_CFG, 'FR')
    assert len(result) == 3
    assert [j['name'] for j in result] == ['FR_hip', 'FR_thigh', 'FR_calf']

def test_select_joints_fr_lowercase():
    result = _select_joints(JOINTS_CFG, 'fr')
    assert len(result) == 3
    assert result[0]['name'] == 'FR_hip'

def test_select_joints_rl():
    result = _select_joints(JOINTS_CFG, 'RL')
    assert [j['name'] for j in result] == ['RL_hip', 'RL_thigh', 'RL_calf']

def test_select_joints_unknown_leg():
    result = _select_joints(JOINTS_CFG, 'XX')
    assert result == []
```

- [ ] **Step 2: Create stub with only the pure functions (no ROS2 imports yet)**

Create `src/legged_control/legged_control/motor_bench_node.py`:

```python
"""
motor_bench_node

Single-leg joystick position test. Sweeps one of three joints through its
configured q_min/q_max range via left stick vertical. B/X buttons cycle
through joints; switching locks the departing joint at its last feedback
position.

Subscriptions:
  /joy                        sensor_msgs/Joy
  /<ns>/joint_states × 3      sensor_msgs/JointState

Publication:
  /joint_commands             sensor_msgs/JointState  (50 Hz)

Parameters:
  leg  string  default 'FR'   leg prefix, case-insensitive (FR/FL/RR/RL)
"""

_AXIS_POS = 1    # left stick vertical (inverted inside callback)
_BTN_NEXT = 1    # B button
_BTN_PREV = 2    # X button

_NS_MAP = {
    'FR': ['fr/hip', 'fr/thigh', 'fr/calf'],
    'FL': ['fl/hip', 'fl/thigh', 'fl/calf'],
    'RR': ['rr/hip', 'rr/thigh', 'rr/calf'],
    'RL': ['rl/hip', 'rl/thigh', 'rl/calf'],
}


def _apply_deadzone(v: float, dz: float = 0.05) -> float:
    """Return v rescaled so that |v| < dz maps to 0, full range stays ±1."""
    if abs(v) <= dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _map_to_range(v: float, q_min: float, q_max: float) -> float:
    """Map v ∈ [-1, 1] linearly to [q_min, q_max]."""
    return q_min + (v + 1.0) / 2.0 * (q_max - q_min)


def _select_joints(joints_cfg: list, leg: str) -> list:
    """Return the joints whose name starts with leg (case-insensitive)."""
    prefix = leg.upper()
    return [j for j in joints_cfg if j['name'].upper().startswith(prefix)]
```

- [ ] **Step 3: Run tests — expect them to pass**

```bash
cd /home/grayred/rc/legged_ws
python -m pytest src/legged_control/tests/test_motor_bench_node.py -v
```

Expected: all tests PASS. If `_apply_deadzone(0.05)` returns non-zero, the boundary is `<=` not `<` — adjust accordingly (already written as `<=` above).

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/legged_control/motor_bench_node.py \
        src/legged_control/tests/test_motor_bench_node.py
git commit -m "feat(motor-bench): add pure helper functions with tests"
```

---

## Task 3: Complete `motor_bench_node.py` with ROS2 node class

**Files:**
- Modify: `src/legged_control/legged_control/motor_bench_node.py`

- [ ] **Step 1: Replace the stub with the full node**

Replace the entire contents of `src/legged_control/legged_control/motor_bench_node.py` with:

```python
"""
motor_bench_node

Single-leg joystick position test. Sweeps one of three joints through its
configured q_min/q_max range via left stick vertical. B/X buttons cycle
through joints; switching locks the departing joint at its last feedback
position.

Subscriptions:
  /joy                        sensor_msgs/Joy
  /<ns>/joint_states × 3      sensor_msgs/JointState

Publication:
  /joint_commands             sensor_msgs/JointState  (50 Hz)

Parameters:
  leg  string  default 'FR'   leg prefix, case-insensitive (FR/FL/RR/RL)
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

_AXIS_POS = 1    # left stick vertical (inverted inside callback)
_BTN_NEXT = 1    # B button — next joint
_BTN_PREV = 2    # X button — previous joint

_NS_MAP = {
    'FR': ['fr/hip', 'fr/thigh', 'fr/calf'],
    'FL': ['fl/hip', 'fl/thigh', 'fl/calf'],
    'RR': ['rr/hip', 'rr/thigh', 'rr/calf'],
    'RL': ['rl/hip', 'rl/thigh', 'rl/calf'],
}


def _apply_deadzone(v: float, dz: float = 0.05) -> float:
    """Return v rescaled so that |v| <= dz maps to 0, full range stays ±1."""
    if abs(v) <= dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _map_to_range(v: float, q_min: float, q_max: float) -> float:
    """Map v ∈ [-1, 1] linearly to [q_min, q_max]."""
    return q_min + (v + 1.0) / 2.0 * (q_max - q_min)


def _select_joints(joints_cfg: list, leg: str) -> list:
    """Return the joints whose name starts with leg (case-insensitive)."""
    prefix = leg.upper()
    return [j for j in joints_cfg if j['name'].upper().startswith(prefix)]


class MotorBenchNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_bench_node')
        self.declare_parameter('leg', 'FR')
        leg = self.get_parameter('leg').get_parameter_value().string_value.upper()

        cfg = self._load_config()
        self._joints = _select_joints(cfg['joints'], leg)
        if len(self._joints) != 3:
            raise RuntimeError(
                f"Expected 3 joints for leg '{leg}', "
                f"found {len(self._joints)}: {[j['name'] for j in self._joints]}"
            )

        self._dz = float(cfg['teleop']['deadzone'])
        self._active_idx = 0
        self._locked_q   = [float(j['default_q']) for j in self._joints]
        self._feedback_q = [float(j['default_q']) for j in self._joints]
        self._prev_buttons: list[int] = []

        # Feedback subscriptions — one per joint namespace
        ns_list = _NS_MAP[leg]
        self._subs_fb = []
        for i, ns in enumerate(ns_list):
            sub = self.create_subscription(
                JointState, f'/{ns}/joint_states',
                self._make_fb_cb(i), 10)
            self._subs_fb.append(sub)

        self._sub_joy = self.create_subscription(
            Joy, '/joy', self._on_joy, 10)
        self._pub = self.create_publisher(
            JointState, '/joint_commands', 10)
        self._timer = self.create_timer(1.0 / 50.0, self._publish_cmd)

        self.get_logger().info(
            f'Motor bench ready  leg={leg}  '
            f'active={self._joints[0]["name"]}  '
            f'(B=next joint  X=prev joint)')

    # ------------------------------------------------------------------

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _make_fb_cb(self, idx: int):
        def cb(msg: JointState) -> None:
            if msg.position:
                self._feedback_q[idx] = float(msg.position[0])
        return cb

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        prev    = self._prev_buttons

        def fell(i: int) -> bool:
            """True on the first frame a button is pressed (falling edge)."""
            return (i < len(buttons) and buttons[i] == 1
                    and (i >= len(prev) or prev[i] == 0))

        # Left stick vertical → position of active joint
        # axis 1 is inverted: push up = negative raw value
        raw = -msg.axes[_AXIS_POS]
        v   = _apply_deadzone(raw, self._dz)
        if v != 0.0:
            j = self._joints[self._active_idx]
            self._locked_q[self._active_idx] = _map_to_range(
                v, j['q_min'], j['q_max'])

        # B → next joint
        if fell(_BTN_NEXT):
            self._locked_q[self._active_idx] = self._feedback_q[self._active_idx]
            self._active_idx = (self._active_idx + 1) % 3
            self.get_logger().info(
                f'Active joint → {self._joints[self._active_idx]["name"]}')

        # X → previous joint
        if fell(_BTN_PREV):
            self._locked_q[self._active_idx] = self._feedback_q[self._active_idx]
            self._active_idx = (self._active_idx - 1) % 3
            self.get_logger().info(
                f'Active joint → {self._joints[self._active_idx]["name"]}')

        self._prev_buttons = buttons

    def _publish_cmd(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = [j['name'] for j in self._joints]
        msg.position = list(self._locked_q)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotorBenchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Re-run the unit tests to confirm pure functions still pass**

```bash
python -m pytest src/legged_control/tests/test_motor_bench_node.py -v
```

Expected: all tests PASS (the class was added without touching the pure functions).

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/legged_control/motor_bench_node.py
git commit -m "feat(motor-bench): add MotorBenchNode ROS2 class"
```

---

## Task 4: Write `motor_bench.launch.py`

**Files:**
- Create: `src/legged_control/launch/motor_bench.launch.py`

- [ ] **Step 1: Create the launch file**

Create `src/legged_control/launch/motor_bench.launch.py`:

```python
"""
motor_bench.launch.py — single-leg motor bench test.

Launch args:
  leg          [FR]               Leg to test: FR / FL / RR / RL (case-insensitive)
  joy_device   [/dev/input/js0]   Joystick device path

Usage:
  ros2 launch legged_control motor_bench.launch.py
  ros2 launch legged_control motor_bench.launch.py leg:=RL
  ros2 launch legged_control motor_bench.launch.py leg:=FL joy_device:=/dev/input/js1
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NS_MAP = {
    'FR': ['fr/hip', 'fr/thigh', 'fr/calf'],
    'FL': ['fl/hip', 'fl/thigh', 'fl/calf'],
    'RR': ['rr/hip', 'rr/thigh', 'rr/calf'],
    'RL': ['rl/hip', 'rl/thigh', 'rl/calf'],
}


def _load_robot_config() -> dict:
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def _launch_setup(context, *args, **kwargs):
    leg        = LaunchConfiguration('leg').perform(context).upper()
    joy_device = LaunchConfiguration('joy_device').perform(context)

    cfg         = _load_robot_config()
    control_cfg = cfg['control']
    serial_port = control_cfg['serial_port']
    motor_hz    = control_cfg['motor_hz']
    kp          = control_cfg['kp']
    kd          = control_cfg['kd']

    # Pick the 3 joints for this leg (same prefix logic as motor_bench_node)
    leg_joints = [j for j in cfg['joints']
                  if j['name'].upper().startswith(leg)]
    if len(leg_joints) != 3:
        raise RuntimeError(
            f"motor_bench.launch: expected 3 joints for leg '{leg}', "
            f"found {len(leg_joints)}")

    ns_list = _NS_MAP[leg]

    motor_nodes = [
        Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=ns,
            name='motor',
            parameters=[{
                'serial_port': serial_port,
                'motor_id':    j['motor_id'],
                'loop_hz':     motor_hz,
                'joint_name':  j['name'],
                'target_q':    j['default_q'],
                'target_dq':   0.0,
                'kp':          kp,
                'kd':          kd,
                'tau':         0.0,
            }],
            output='screen',
        )
        for j, ns in zip(leg_joints, ns_list)
    ]

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': joy_device}],
        output='screen',
    )

    bench = Node(
        package='legged_control',
        executable='motor_bench_node',
        name='motor_bench_node',
        parameters=[{'leg': leg}],
        output='screen',
    )

    return motor_nodes + [joy, bench]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('leg',        default_value='FR',
                              description='Leg to test: FR/FL/RR/RL'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0',
                              description='Joystick device path'),
        OpaqueFunction(function=_launch_setup),
    ])
```

- [ ] **Step 2: Commit**

```bash
git add src/legged_control/launch/motor_bench.launch.py
git commit -m "feat(motor-bench): add motor_bench.launch.py"
```

---

## Task 5: Register new files in `setup.py`

**Files:**
- Modify: `src/legged_control/setup.py`

- [ ] **Step 1: Add entry point and launch file**

In `src/legged_control/setup.py`, make two additions:

In `data_files`, add `motor_bench.launch.py` to the launch share line:

```python
('share/' + package_name + '/launch', [
    'launch/locomotion.launch.py',
    'launch/motor_bench.launch.py',
]),
```

In `console_scripts`, add:

```python
'motor_bench_node = legged_control.motor_bench_node:main',
```

The full `setup.py` should look like:

```python
from setuptools import setup, find_packages

package_name = 'legged_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/locomotion.launch.py',
            'launch/motor_bench.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'joint_aggregator  = legged_control.joint_aggregator:main',
            'teleop_node       = legged_control.teleop_node:main',
            'policy_node       = legged_control.policy_node:main',
            'watchdog_node     = legged_control.watchdog_node:main',
            'motor_bench_node  = legged_control.motor_bench_node:main',
        ],
    },
)
```

- [ ] **Step 2: Commit**

```bash
git add src/legged_control/setup.py
git commit -m "feat(motor-bench): register motor_bench_node entry point and launch file"
```

---

## Task 6: Build and verify

- [ ] **Step 1: Build the package**

```bash
cd /home/grayred/rc/legged_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
```

Expected: build finishes with `Summary: 1 packages finished`.

- [ ] **Step 2: Verify the executable is registered**

```bash
ros2 run legged_control motor_bench_node --ros-args -p leg:=FR &
sleep 2
kill %1
```

Expected: log line `Motor bench ready  leg=FR  active=FR_hip  (B=next joint  X=prev joint)` then clean shutdown.

- [ ] **Step 3: Verify the launch file is found**

```bash
ros2 launch legged_control motor_bench.launch.py --show-args
```

Expected output includes `leg` and `joy_device` arguments with their defaults.

- [ ] **Step 4: Run full test suite to confirm nothing broken**

```bash
python -m pytest src/legged_control/tests/ -v
```

Expected: all tests PASS.

- [ ] **Step 5: Final commit**

```bash
git add -A
git status   # confirm only expected files are staged
git commit -m "feat(motor-bench): verify build and smoke test"
```
