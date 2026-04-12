# Motor Bus Node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace 12 individual `go_m8010_6_node` instances with 2 `motor_bus_node` instances (one per serial port) to eliminate RS485 bus collisions caused by concurrent `sendRecv` calls from multiple processes.

**Architecture:** A new `motor_bus_node` owns one serial port exclusively and cycles through all assigned joints sequentially in each 1000 Hz tick using individual `sendRecv` calls. It validates each response (`data.correct` and `data.motor_id`) before publishing. `stand_node` is updated to broadcast kp/kd to the two bus nodes instead of 12 individual motor nodes. The launch file starts 2 bus node instances instead of 12 motor nodes.

**Tech Stack:** Python 3, ROS2 Humble, Unitree GO-M8010-6 SDK (`sendRecv`), `robot.yaml`

---

### Task 1: Write failing tests for `motor_bus_node` helpers

**Files:**
- Create: `src/legged_control/tests/test_motor_bus_node.py`

- [ ] **Step 1: Write the failing test file**

```python
# src/legged_control/tests/test_motor_bus_node.py
from legged_control.motor_bus_node import _ns_from_joint_name, _filter_joints


def test_ns_hip():
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_calf():
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_thigh():
    assert _ns_from_joint_name('RR_thigh') == 'rr/thigh'


def test_filter_joints_subset():
    joints = [
        {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0},
        {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8},
        {'name': 'RR_hip',   'motor_id': 6, 'default_q': 0.0},
    ]
    result = _filter_joints(joints, ['FR_hip', 'FR_thigh'])
    assert len(result) == 2
    assert result[0]['name'] == 'FR_hip'
    assert result[1]['name'] == 'FR_thigh'


def test_filter_joints_empty_names():
    joints = [{'name': 'FR_hip', 'motor_id': 0, 'default_q': 0.0}]
    assert _filter_joints(joints, []) == []


def test_filter_joints_unknown_name():
    joints = [{'name': 'FR_hip', 'motor_id': 0, 'default_q': 0.0}]
    assert _filter_joints(joints, ['XX_hip']) == []


def test_filter_joints_preserves_yaml_order():
    joints = [
        {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0},
        {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8},
        {'name': 'FR_calf',  'motor_id': 2, 'default_q': -1.5},
    ]
    # Request in reverse order — result follows yaml order, not request order
    result = _filter_joints(joints, ['FR_calf', 'FR_hip'])
    assert [j['name'] for j in result] == ['FR_hip', 'FR_calf']
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/opt/yaml_cpp_vendor/lib:$LD_LIBRARY_PATH" \
PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH" \
python3 -m pytest src/legged_control/tests/test_motor_bus_node.py -v
```

Expected: `ModuleNotFoundError` — `motor_bus_node` does not exist yet.

- [ ] **Step 3: Commit the failing test**

```bash
git add src/legged_control/tests/test_motor_bus_node.py
git commit -m "test: add failing tests for motor_bus_node helpers"
```

---

### Task 2: Implement `motor_bus_node.py` and register entry point

**Files:**
- Create: `src/legged_control/legged_control/motor_bus_node.py`
- Modify: `src/legged_control/setup.py`

- [ ] **Step 1: Create `motor_bus_node.py`**

```python
# src/legged_control/legged_control/motor_bus_node.py
"""
motor_bus_node

Manages all motors on a single RS485 serial bus. Cycles through every
assigned joint sequentially in each tick — only one process ever holds
the serial port, eliminating bus collisions.

Two instances are started: motor_bus_front (FR/FL) and motor_bus_rear (RR/RL).

Gain tuning at runtime without restart:
  ros2 param set /motor_bus_front kp 5.0
  ros2 param set /motor_bus_front kd 0.3
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState

from .sdk_loader import load_sdk


def _ns_from_joint_name(name: str) -> str:
    """'FR_hip' → 'fr/hip'"""
    leg, joint = name.split('_', 1)
    return f'{leg.lower()}/{joint.lower()}'


def _filter_joints(all_joints: list, joint_names: list) -> list:
    """Return joint dicts from all_joints whose 'name' is in joint_names,
    preserving the order they appear in all_joints."""
    name_set = set(joint_names)
    return [j for j in all_joints if j['name'] in name_set]


class MotorBusNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_bus_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('kp', 20.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('loop_hz', 1000.0)

        joint_names = self.get_parameter('joint_names').value
        cfg = self._load_config()
        joints = _filter_joints(cfg['joints'], joint_names)
        if not joints:
            raise RuntimeError(
                f'motor_bus_node: no joints found for names {joint_names}')

        sdk = load_sdk()
        self._sdk = sdk
        serial_port = self.get_parameter('serial_port').value
        self._serial = sdk.SerialPort(serial_port)

        self._names = [j['name'] for j in joints]
        self._targets = {j['name']: float(j['default_q']) for j in joints}

        self._cmds = []
        self._datas = []
        for j in joints:
            cmd = sdk.MotorCmd()
            cmd.motorType = sdk.MotorType.GO_M8010_6
            cmd.mode = sdk.queryMotorMode(
                sdk.MotorType.GO_M8010_6, sdk.MotorMode.FOC)
            cmd.id  = j['motor_id']
            cmd.q   = float(j['default_q'])
            cmd.dq  = 0.0
            cmd.tau = 0.0
            self._cmds.append(cmd)

            data = sdk.MotorData()
            data.motorType = sdk.MotorType.GO_M8010_6
            self._datas.append(data)

        self._pubs = [
            self.create_publisher(
                JointState,
                f'/{_ns_from_joint_name(name)}/joint_states',
                10,
            )
            for name in self._names
        ]

        self.create_subscription(
            JointState, '/joint_commands', self._on_joint_cmd, 10)
        self.add_on_set_parameters_callback(self._on_gains_changed)

        loop_hz = self.get_parameter('loop_hz').value
        self.create_timer(1.0 / loop_hz, self._tick)

        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        self.get_logger().info(
            f'Motor bus ready — {len(joints)} joints on {serial_port}  '
            f'kp={kp}  kd={kd}')

    def _load_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)

    def _on_joint_cmd(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._targets:
                self._targets[name] = float(pos)

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        for p in params:
            if p.name in ('kp', 'kd'):
                self.get_logger().info(f'Gain updated: {p.name}={p.value}')
        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value

        for cmd, data, pub, name in zip(
                self._cmds, self._datas, self._pubs, self._names):
            cmd.kp = kp
            cmd.kd = kd
            cmd.q  = self._targets[name]
            cmd.dq = 0.0
            self._serial.sendRecv(cmd, data)

            if data.correct and data.motor_id == cmd.id:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name     = [name]
                msg.position = [float(data.q)]
                msg.velocity = [float(data.dq)]
                msg.effort   = [float(data.tau)]
                pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotorBusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Register the entry point in `setup.py`**

In `src/legged_control/setup.py`, find the `console_scripts` list and add the new entry:

```python
        'console_scripts': [
            'passive_monitor_node = legged_control.passive_monitor_node:main',
            'stand_node           = legged_control.stand_node:main',
            'motor_bus_node       = legged_control.motor_bus_node:main',
        ],
```

- [ ] **Step 3: Run tests to verify helpers pass**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/opt/yaml_cpp_vendor/lib:$LD_LIBRARY_PATH" \
PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH" \
python3 -m pytest src/legged_control/tests/test_motor_bus_node.py -v
```

Expected: 6 tests PASS.

- [ ] **Step 4: Run full test suite**

```bash
LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/opt/yaml_cpp_vendor/lib:$LD_LIBRARY_PATH" \
PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH" \
python3 -m pytest src/legged_control/tests/ -v
```

Expected: all tests PASS (15 existing + 6 new = 21 total).

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/motor_bus_node.py \
        src/legged_control/setup.py
git commit -m "feat: add motor_bus_node — single-owner RS485 bus manager"
```

---

### Task 3: Update `stand_node.py` — 2 bus clients instead of 12 motor clients

**Files:**
- Modify: `src/legged_control/legged_control/stand_node.py`
- Modify: `src/legged_control/tests/test_stand_node.py`

- [ ] **Step 1: Remove `_motor_node_name` from `stand_node.py` and update `_gain_clients`**

In `src/legged_control/legged_control/stand_node.py`:

Remove this function entirely (lines 36–39):
```python
def _motor_node_name(joint_name: str) -> str:
    """'FR_hip' → '/fr/hip/motor'"""
    leg, joint = joint_name.split('_', 1)
    return f'/{leg.lower()}/{joint.lower()}/motor'
```

Replace the `_gain_clients` block (currently lines 58–63):
```python
        # One async parameter client per motor node.
        # set_parameters calls are fire-and-forget; absent nodes fail silently.
        self._gain_clients = [
            AsyncParametersClient(self, _motor_node_name(j['name']))
            for j in cfg['joints']
        ]
```

With:
```python
        # One async parameter client per bus node.
        # set_parameters calls are fire-and-forget; absent nodes fail silently.
        self._gain_clients = [
            AsyncParametersClient(self, '/motor_bus_front'),
            AsyncParametersClient(self, '/motor_bus_rear'),
        ]
```

Also update the module docstring at the top — replace:
```
The new values are immediately broadcast to all motor nodes via the
ROS2 parameter service. No rebuild or relaunch required.
```
With:
```
The new values are immediately broadcast to both motor bus nodes via the
ROS2 parameter service. No rebuild or relaunch required.
```

- [ ] **Step 2: Remove obsolete `_motor_node_name` tests from `test_stand_node.py`**

In `src/legged_control/tests/test_stand_node.py`, remove the import of `_motor_node_name` and the three test functions that use it:

Remove from the import line:
```python
from legged_control.stand_node import _load_joint_defaults, _motor_node_name
```
Replace with:
```python
from legged_control.stand_node import _load_joint_defaults
```

Remove these three test functions:
```python
def test_motor_node_name_hip():
    assert _motor_node_name('FR_hip') == '/fr/hip/motor'


def test_motor_node_name_calf():
    assert _motor_node_name('RL_calf') == '/rl/calf/motor'


def test_motor_node_name_thigh():
    assert _motor_node_name('RR_thigh') == '/rr/thigh/motor'
```

- [ ] **Step 3: Run full test suite**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/opt/yaml_cpp_vendor/lib:$LD_LIBRARY_PATH" \
PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH" \
python3 -m pytest src/legged_control/tests/ -v
```

Expected: 18 tests PASS (21 − 3 removed `_motor_node_name` tests).

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/legged_control/stand_node.py \
        src/legged_control/tests/test_stand_node.py
git commit -m "refactor: stand_node broadcasts gains to 2 bus nodes instead of 12 motor nodes"
```

---

### Task 4: Update `robot.launch.py` — replace `_motor_nodes` with `_bus_nodes`

**Files:**
- Modify: `src/legged_control/launch/robot.launch.py`

- [ ] **Step 1: Replace `_motor_nodes` function with `_bus_nodes`**

In `src/legged_control/launch/robot.launch.py`, remove the entire `_motor_nodes` function:

```python
def _motor_nodes(joints: list, port_map: dict, motor_hz: float,
                 kp: float, kd: float) -> list:
    """port_map = {'front': '/dev/ttyUSB0', 'rear': '/dev/ttyUSB1'}"""
    return [
        Node(
            package='unitree_actuator_sdk',
            ...
        )
        for j in joints
    ]
```

Replace with:

```python
def _bus_nodes(joints: list, port_map: dict, motor_hz: float,
               kp: float, kd: float) -> list:
    """Start one motor_bus_node per non-empty leg group."""
    groups = {
        'front': ('motor_bus_front', port_map['front']),
        'rear':  ('motor_bus_rear',  port_map['rear']),
    }
    nodes = []
    for group, (node_name, port) in groups.items():
        group_joints = [j for j in joints if _leg_group(j['name']) == group]
        if not group_joints:
            continue
        nodes.append(Node(
            package='legged_control',
            executable='motor_bus_node',
            name=node_name,
            parameters=[{
                'serial_port':  port,
                'joint_names':  [j['name'] for j in group_joints],
                'kp':           kp,
                'kd':           kd,
                'loop_hz':      motor_hz,
            }],
            output='log',
        ))
    return nodes
```

- [ ] **Step 2: Update `_launch_setup` to call `_bus_nodes` instead of `_motor_nodes`**

In `_launch_setup`, replace every call to `_motor_nodes(...)` with `_bus_nodes(...)`. The signature is identical. Two replacements needed — one in the `passive` branch and one in the `stand` branch:

```python
    if mode == 'passive':
        motors = _bus_nodes(joints, port_map, motor_hz, kp=0.0, kd=0.0)
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
        motors = _bus_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
        companion = Node(
            package='legged_control',
            executable='stand_node',
            name='stand_node',
            output='screen',
        )
        return motors + [companion]
```

- [ ] **Step 3: Run full test suite**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/opt/yaml_cpp_vendor/lib:$LD_LIBRARY_PATH" \
PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH" \
python3 -m pytest src/legged_control/tests/ -v
```

Expected: 18 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/launch/robot.launch.py
git commit -m "feat: launch motor_bus_node instead of individual go_m8010_6_node instances"
```

---

### Task 5: Update `CLAUDE.md`

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update the Architecture section for `legged_control`**

In `CLAUDE.md`, find and replace this block in the `### legged_control` section:

Old:
```markdown
- `launch/robot.launch.py` — single entry point, starts 12 motor nodes + mode-specific node
```

New:
```markdown
- `motor_bus_node.py` — owns one RS485 serial port exclusively; cycles all assigned joints
  with sequential `sendRecv` calls, validates each response (`data.correct` + `data.motor_id`),
  publishes `/{ns}/joint_states`. Two instances: `motor_bus_front` (FR/FL on `serial_port_front`)
  and `motor_bus_rear` (RR/RL on `serial_port_rear`).
- `launch/robot.launch.py` — single entry point, starts 2 bus nodes + mode-specific node
```

Also find and replace the stand_node description:

Old:
```markdown
- `stand_node.py` — publishes `default_q` for all 12 joints at 50 Hz on `/joint_commands`
```

New:
```markdown
- `stand_node.py` — publishes `default_q` for all 12 joints at 50 Hz on `/joint_commands`;
  broadcasts kp/kd changes to `/motor_bus_front` and `/motor_bus_rear` via ROS2 parameter service
```

- [ ] **Step 2: Update the Key Topics table if it references individual motor node names**

Check the Motor Namespace Map section — it maps joint names to topic namespaces (`/{ns}/joint_states`), which is unchanged. No update needed there.

- [ ] **Step 3: Update the runtime gain tuning example**

Find:
```bash
ros2 param set /stand_node kp 5.0
ros2 param set /stand_node kd 0.3
```

The example is on `stand_node`, which is correct (users set kp/kd on stand_node, which then broadcasts to bus nodes). No change needed.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md for motor_bus_node architecture"
```

---

### Task 6: Build and smoke-test

- [ ] **Step 1: Build the package**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
```

Expected: build succeeds with no errors.

- [ ] **Step 2: Verify the new executable is registered**

```bash
ros2 run legged_control motor_bus_node --ros-args -p serial_port:=/dev/null -p joint_names:=['FR_hip']
```

Expected: node starts and immediately fails with a serial port error (since `/dev/null` is not a motor). This confirms the entry point is wired up correctly. `Ctrl+C` to stop.

- [ ] **Step 3: Launch passive mode with FR leg**

With hardware connected:
```bash
ros2 launch legged_control robot.launch.py legs:=FR
```

Expected:
- Only `motor_bus_front` node starts (no `motor_bus_rear`, no `go_m8010_6_node`)
- Terminal shows stable position readout for FR joints with no data cross-talk
- Moving a joint by hand changes only that joint's value

- [ ] **Step 4: Commit smoke-test confirmation** (no code change needed — just note it passed)
