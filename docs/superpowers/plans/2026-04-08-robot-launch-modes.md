# Robot Launch Modes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a clean `legged_control` ROS2 package from scratch on `dev/robot-launch-modes` with a single launch file and three modes: `passive` (zero-torque position monitor), `stand` (hold default pose for PID tuning), `policy` (future placeholder).

**Architecture:** One `robot.launch.py` with a `mode` parameter always starts all 12 motor nodes; mode selects which companion node runs alongside them. Pure helper functions are unit-tested; ROS2 node wiring is verified by a manual build check.

**Tech Stack:** Python 3, ROS2 Humble, `rclpy`, `sensor_msgs`, `ament_python`, `pytest`

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `src/legged_control/package.xml` | Create | ROS2 package manifest |
| `src/legged_control/setup.py` | Create | Python package install, entry points, data files |
| `src/legged_control/setup.cfg` | Create | ament_python install prefix |
| `src/legged_control/resource/legged_control` | Create | ament index marker (empty) |
| `src/legged_control/config/robot.yaml` | Create | Joint config, PD gains (copy from dev/motor-bench) |
| `src/legged_control/legged_control/__init__.py` | Create | Empty package init |
| `src/legged_control/legged_control/passive_monitor_node.py` | Create | Subscribe to 12 joint_states, print positions at 2 Hz |
| `src/legged_control/legged_control/stand_node.py` | Create | Publish default_q to all 12 joints at 50 Hz |
| `src/legged_control/launch/robot.launch.py` | Create | mode/serial_port args, motor nodes, mode node |
| `src/legged_control/tests/__init__.py` | Create | Empty |
| `src/legged_control/tests/test_passive_monitor_node.py` | Create | Unit tests for pure helper functions |
| `src/legged_control/tests/test_stand_node.py` | Create | Unit tests for pure helper functions |

---

## Task 1: Package Scaffold

**Files:**
- Create: `src/legged_control/package.xml`
- Create: `src/legged_control/setup.py`
- Create: `src/legged_control/setup.cfg`
- Create: `src/legged_control/resource/legged_control`
- Create: `src/legged_control/config/robot.yaml`
- Create: `src/legged_control/legged_control/__init__.py`
- Create: `src/legged_control/tests/__init__.py`

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/legged_control/{config,launch,legged_control,resource,tests}
touch src/legged_control/resource/legged_control
touch src/legged_control/legged_control/__init__.py
touch src/legged_control/tests/__init__.py
```

- [ ] **Step 2: Write `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>legged_control</name>
  <version>0.1.0</version>
  <description>Locomotion control stack for quadruped robot.</description>
  <maintainer email="todo@todo.com">todo</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/legged_control
[install]
install_scripts=$base/lib/legged_control
```

- [ ] **Step 4: Write `setup.py`** (entry points and data files will be filled in later tasks)

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
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [],
    },
)
```

- [ ] **Step 5: Copy `robot.yaml` from `dev/motor-bench`**

```bash
git show dev/motor-bench:src/legged_control/config/robot.yaml > src/legged_control/config/robot.yaml
```

Verify it has 12 joint entries and a `control` section:

```bash
grep -c "motor_id" src/legged_control/config/robot.yaml
# Expected output: 12
grep "serial_port\|kp\|kd" src/legged_control/config/robot.yaml
# Expected: serial_port: /dev/ttyUSB0, kp: 0.2, kd: 0.5
```

- [ ] **Step 6: Commit**

```bash
git add src/legged_control/
git commit -m "chore: scaffold legged_control package"
```

---

## Task 2: passive_monitor_node

**Files:**
- Create: `src/legged_control/legged_control/passive_monitor_node.py`
- Create: `src/legged_control/tests/test_passive_monitor_node.py`
- Modify: `src/legged_control/setup.py` (add entry point)

- [ ] **Step 1: Write the failing tests**

```python
# src/legged_control/tests/test_passive_monitor_node.py
import math
from legged_control.passive_monitor_node import _ns_from_joint_name, _format_display


def test_ns_from_joint_name_hip():
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_from_joint_name_calf():
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_from_joint_name_thigh():
    assert _ns_from_joint_name('FL_thigh') == 'fl/thigh'


def test_format_display_structure():
    positions = {
        'FR_hip': 0.312, 'FR_thigh': 0.841, 'FR_calf': -1.502,
        'FL_hip': 0.000, 'FL_thigh': 0.823, 'FL_calf': -1.499,
        'RR_hip': -0.015, 'RR_thigh': 0.810, 'RR_calf': -1.510,
        'RL_hip': 0.003, 'RL_thigh': 0.834, 'RL_calf': -1.495,
    }
    text = _format_display(positions)
    lines = text.splitlines()
    assert len(lines) == 4
    assert '[passive]' in lines[0]
    assert 'FR' in lines[0]
    assert 'FL' in lines[1]
    assert 'RR' in lines[2]
    assert 'RL' in lines[3]


def test_format_display_values():
    positions = {
        'FR_hip': 0.312, 'FR_thigh': 0.841, 'FR_calf': -1.502,
        'FL_hip': 0.0, 'FL_thigh': 0.823, 'FL_calf': -1.499,
        'RR_hip': -0.015, 'RR_thigh': 0.810, 'RR_calf': -1.510,
        'RL_hip': 0.003, 'RL_thigh': 0.834, 'RL_calf': -1.495,
    }
    text = _format_display(positions)
    assert ' 0.312' in text
    assert '-1.502' in text
    assert '-0.015' in text


def test_format_display_nan_for_missing():
    text = _format_display({})
    assert 'nan' in text
```

- [ ] **Step 2: Run tests — verify they fail**

```bash
cd src/legged_control && python -m pytest tests/test_passive_monitor_node.py -v
```

Expected: `ImportError` or `ModuleNotFoundError` (module not yet written).

- [ ] **Step 3: Implement `passive_monitor_node.py`**

```python
"""
passive_monitor_node

Subscribes to all 12 joint_states topics. Every 0.5 s overwrites the
terminal with a 4-line position readout (one line per leg). Motors are
expected to run at zero torque so the operator can move joints by hand
to observe limits and default positions.

No command publishing, no joystick, no file writes.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_LEGS = ['FR', 'FL', 'RR', 'RL']
_SLOTS = ['hip', 'thigh', 'calf']


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' → 'fr/hip'"""
    leg, joint = name.split('_', 1)
    return f'{leg.lower()}/{joint.lower()}'


def _format_display(positions: dict) -> str:
    """
    Format all 12 positions as a 4-line string.

    positions: mapping of joint_name → float (nan if not yet received).
    """
    lines = []
    for i, leg in enumerate(_LEGS):
        prefix = '[passive] ' if i == 0 else '          '
        values = '  '.join(
            f'{slot}={positions.get(f"{leg}_{slot}", float("nan")):7.3f}'
            for slot in _SLOTS
        )
        lines.append(f'{prefix}{leg}  {values}')
    return '\n'.join(lines)


class PassiveMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('passive_monitor_node')

        cfg = self._load_config()
        self._positions: dict = {j['name']: float('nan') for j in cfg['joints']}

        for j in cfg['joints']:
            ns = _ns_from_joint_name(j['name'])
            self.create_subscription(
                JointState, f'/{ns}/joint_states',
                self._make_cb(j['name']), 10,
            )

        self._first = True
        self.create_timer(0.5, self._display)
        self.get_logger().info('Passive monitor ready — move joints by hand')

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _make_cb(self, joint_name: str):
        def cb(msg: JointState) -> None:
            if msg.position:
                self._positions[joint_name] = float(msg.position[0])
        return cb

    def _display(self) -> None:
        text = _format_display(self._positions)
        if self._first:
            print(text, flush=True)
            self._first = False
        else:
            # Move cursor up 4 lines and overwrite
            print(f'\033[4A\033[J{text}', flush=True)


def main() -> None:
    rclpy.init()
    node = PassiveMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Run tests — verify they pass**

```bash
cd src/legged_control && python -m pytest tests/test_passive_monitor_node.py -v
```

Expected output:
```
test_passive_monitor_node.py::test_ns_from_joint_name_hip PASSED
test_passive_monitor_node.py::test_ns_from_joint_name_calf PASSED
test_passive_monitor_node.py::test_ns_from_joint_name_thigh PASSED
test_passive_monitor_node.py::test_format_display_structure PASSED
test_passive_monitor_node.py::test_format_display_values PASSED
test_passive_monitor_node.py::test_format_display_nan_for_missing PASSED
6 passed
```

- [ ] **Step 5: Add entry point to `setup.py`**

In `setup.py`, replace the empty `console_scripts` list:

```python
        'console_scripts': [
            'passive_monitor_node = legged_control.passive_monitor_node:main',
        ],
```

- [ ] **Step 6: Commit**

```bash
git add src/legged_control/legged_control/passive_monitor_node.py \
        src/legged_control/tests/test_passive_monitor_node.py \
        src/legged_control/setup.py
git commit -m "feat: add passive_monitor_node with tests"
```

---

## Task 3: stand_node

**Files:**
- Create: `src/legged_control/legged_control/stand_node.py`
- Create: `src/legged_control/tests/test_stand_node.py`
- Modify: `src/legged_control/setup.py` (add entry point)

- [ ] **Step 1: Write the failing tests**

```python
# src/legged_control/tests/test_stand_node.py
from legged_control.stand_node import _load_joint_defaults


def test_load_joint_defaults_names():
    joints_cfg = [
        {'name': 'FR_hip',   'default_q':  0.0},
        {'name': 'FR_thigh', 'default_q':  0.8},
        {'name': 'FR_calf',  'default_q': -1.5},
    ]
    names, _ = _load_joint_defaults(joints_cfg)
    assert names == ['FR_hip', 'FR_thigh', 'FR_calf']


def test_load_joint_defaults_values():
    joints_cfg = [
        {'name': 'FR_hip',   'default_q':  0.0},
        {'name': 'FR_thigh', 'default_q':  0.8},
        {'name': 'FR_calf',  'default_q': -1.5},
    ]
    _, defaults = _load_joint_defaults(joints_cfg)
    assert defaults == [0.0, 0.8, -1.5]


def test_load_joint_defaults_coerces_to_float():
    joints_cfg = [{'name': 'FR_hip', 'default_q': '0'}]
    _, defaults = _load_joint_defaults(joints_cfg)
    assert isinstance(defaults[0], float)


def test_load_joint_defaults_12_joints():
    joints_cfg = [
        {'name': f'J{i}', 'default_q': float(i)} for i in range(12)
    ]
    names, defaults = _load_joint_defaults(joints_cfg)
    assert len(names) == 12
    assert len(defaults) == 12
```

- [ ] **Step 2: Run tests — verify they fail**

```bash
cd src/legged_control && python -m pytest tests/test_stand_node.py -v
```

Expected: `ImportError` (module not yet written).

- [ ] **Step 3: Implement `stand_node.py`**

```python
"""
stand_node

Publishes default_q for all 12 joints at 50 Hz on /joint_commands.
Motors run with PD gains from robot.yaml, holding the robot in its
default standing pose. Used for manual kp/kd tuning: edit robot.yaml
and restart the launch to apply new gains.

No joystick input, no feedback subscription.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _load_joint_defaults(joints_cfg: list) -> tuple:
    """Return (names, default_q_values) from joints config list."""
    names    = [j['name'] for j in joints_cfg]
    defaults = [float(j['default_q']) for j in joints_cfg]
    return names, defaults


class StandNode(Node):
    def __init__(self) -> None:
        super().__init__('stand_node')

        cfg = self._load_config()
        self._names, self._default_q = _load_joint_defaults(cfg['joints'])

        self._pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.create_timer(1.0 / 50.0, self._publish)

        self.get_logger().info(
            f'Stand node ready — holding {len(self._names)} joints at default_q'
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self._names)
        msg.position = list(self._default_q)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Run tests — verify they pass**

```bash
cd src/legged_control && python -m pytest tests/test_stand_node.py -v
```

Expected output:
```
test_stand_node.py::test_load_joint_defaults_names PASSED
test_stand_node.py::test_load_joint_defaults_values PASSED
test_stand_node.py::test_load_joint_defaults_coerces_to_float PASSED
test_stand_node.py::test_load_joint_defaults_12_joints PASSED
4 passed
```

- [ ] **Step 5: Add entry point to `setup.py`**

In `setup.py`, add to `console_scripts`:

```python
        'console_scripts': [
            'passive_monitor_node = legged_control.passive_monitor_node:main',
            'stand_node           = legged_control.stand_node:main',
        ],
```

- [ ] **Step 6: Commit**

```bash
git add src/legged_control/legged_control/stand_node.py \
        src/legged_control/tests/test_stand_node.py \
        src/legged_control/setup.py
git commit -m "feat: add stand_node with tests"
```

---

## Task 4: robot.launch.py

**Files:**
- Create: `src/legged_control/launch/robot.launch.py`

The launch file reads `robot.yaml` at launch time (inside `OpaqueFunction`), builds 12 motor nodes with the correct gains for the selected mode, and starts the appropriate companion node.

Motor nodes use `output='log'` so their ROS2 logger output goes to `~/.ros/log/` instead of the terminal, keeping the terminal clean for the monitor display. Only the companion node uses `output='screen'`.

- [ ] **Step 1: Write `robot.launch.py`**

```python
"""
robot.launch.py — unified launch for all robot operating modes.

Launch args:
  mode          [passive]           passive | stand | policy
  serial_port   [from robot.yaml]   Override serial port for all motors

Usage:
  ros2 launch legged_control robot.launch.py
  ros2 launch legged_control robot.launch.py mode:=stand
  ros2 launch legged_control robot.launch.py mode:=passive serial_port:=/dev/ttyUSB1
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NS_MAP = {
    'FR_hip':   'fr/hip',   'FR_thigh': 'fr/thigh', 'FR_calf': 'fr/calf',
    'FL_hip':   'fl/hip',   'FL_thigh': 'fl/thigh', 'FL_calf': 'fl/calf',
    'RR_hip':   'rr/hip',   'RR_thigh': 'rr/thigh', 'RR_calf': 'rr/calf',
    'RL_hip':   'rl/hip',   'RL_thigh': 'rl/thigh', 'RL_calf': 'rl/calf',
}

_YAML_SENTINEL = '__from_yaml__'


def _load_config() -> dict:
    share = get_package_share_directory('legged_control')
    with open(os.path.join(share, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def _motor_nodes(joints: list, serial_port: str, motor_hz: float,
                 kp: float, kd: float) -> list:
    return [
        Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=_NS_MAP[j['name']],
            name='motor',
            parameters=[{
                'serial_port': serial_port,
                'motor_id':    j['motor_id'],
                'loop_hz':     motor_hz,
                'joint_name':  j['name'],
                'target_q':    float(j['default_q']),
                'target_dq':   0.0,
                'kp':          kp,
                'kd':          kd,
                'tau':         0.0,
            }],
            output='log',
        )
        for j in joints
    ]


def _launch_setup(context, *args, **kwargs):
    mode        = LaunchConfiguration('mode').perform(context).lower()
    serial_port = LaunchConfiguration('serial_port').perform(context)

    cfg     = _load_config()
    control = cfg['control']
    if serial_port == _YAML_SENTINEL:
        serial_port = control['serial_port']
    motor_hz = float(control['motor_hz'])
    joints   = cfg['joints']

    if mode == 'passive':
        motors = _motor_nodes(joints, serial_port, motor_hz, kp=0.0, kd=0.0)
        companion = Node(
            package='legged_control',
            executable='passive_monitor_node',
            name='passive_monitor_node',
            output='screen',
        )
        return motors + [companion]

    if mode == 'stand':
        kp     = float(control['kp'])
        kd     = float(control['kd'])
        motors = _motor_nodes(joints, serial_port, motor_hz, kp=kp, kd=kd)
        companion = Node(
            package='legged_control',
            executable='stand_node',
            name='stand_node',
            output='screen',
        )
        return motors + [companion]

    if mode == 'policy':
        raise RuntimeError(
            "mode:=policy is not yet implemented. "
            "Available modes: passive, stand"
        )

    raise RuntimeError(
        f"Unknown mode '{mode}'. Valid modes: passive, stand, policy"
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='passive',
            description='Operating mode: passive | stand | policy',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value=_YAML_SENTINEL,
            description='Serial port override (default: value from robot.yaml)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
```

- [ ] **Step 2: Build the package**

```bash
cd /path/to/legged_ws   # workspace root
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
```

Expected: build succeeds with no errors.

- [ ] **Step 3: Verify entry points are installed**

```bash
source install/setup.bash
ros2 run legged_control passive_monitor_node --help 2>&1 | head -5
ros2 run legged_control stand_node --help 2>&1 | head -5
```

Expected: both commands start (they will fail without a full ROS2 graph, but the executables must be found).

- [ ] **Step 4: Verify launch file is found**

```bash
ros2 launch legged_control robot.launch.py --show-args
```

Expected output includes:
```
Arguments (pass arguments as '<name>:=<value>'):
    'mode': ...  default: 'passive'
    'serial_port': ...
```

- [ ] **Step 5: Run all tests**

```bash
cd src/legged_control
python -m pytest tests/ -v
```

Expected: 10 tests, all PASSED.

- [ ] **Step 6: Commit**

```bash
git add src/legged_control/launch/robot.launch.py
git commit -m "feat: add robot.launch.py with passive/stand/policy modes"
```
