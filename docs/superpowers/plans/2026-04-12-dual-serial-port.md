# Dual Serial Port Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Route front legs (FR/FL) to one serial port and rear legs (RR/RL) to a second serial port, configured in `robot.yaml` with optional launch-arg overrides.

**Architecture:** Add `serial_port_front` / `serial_port_rear` to `robot.yaml`; add a `_leg_group()` helper to the launch file; update `_motor_nodes()` to accept a `port_map` dict and select the correct port per joint; replace the single `serial_port` launch arg with two args.

**Tech Stack:** Python 3, ROS2 Humble, `robot.yaml`, `robot.launch.py`

---

### Task 1: Test `_leg_group` helper (TDD — write failing test first)

**Files:**
- Create: `src/legged_control/tests/test_robot_launch.py`

- [ ] **Step 1: Write the failing test**

```python
# src/legged_control/tests/test_robot_launch.py
import importlib.util, os

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'robot.launch.py')
_spec   = importlib.util.spec_from_file_location('robot_launch', os.path.abspath(_LAUNCH))
_mod    = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)   # will fail — _leg_group doesn't exist yet
_leg_group = _mod._leg_group


def test_leg_group_front():
    assert _leg_group('FR_hip')   == 'front'
    assert _leg_group('FL_thigh') == 'front'
    assert _leg_group('FL_calf')  == 'front'


def test_leg_group_rear():
    assert _leg_group('RR_hip')   == 'rear'
    assert _leg_group('RL_thigh') == 'rear'
    assert _leg_group('RL_calf')  == 'rear'
```

> Note: `robot.launch.py` has a dot in the filename so standard `import` doesn't work.
> `importlib.util.spec_from_file_location` loads it by path instead.

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
python -m pytest src/legged_control/tests/test_robot_launch.py -v
```

Expected: `ModuleNotFoundError` or `ImportError` — `_leg_group` does not exist yet.

- [ ] **Step 3: Commit the failing test**

```bash
git add src/legged_control/tests/test_robot_launch.py
git commit -m "test: add failing tests for _leg_group (dual serial port)"
```

---

### Task 2: Implement `_leg_group`, update `_motor_nodes` and `_launch_setup`

**Files:**
- Modify: `src/legged_control/launch/robot.launch.py`

- [ ] **Step 1: Add `_leg_group` helper after `_load_config`**

In `robot.launch.py`, after the `_load_config()` function, add:

```python
def _leg_group(joint_name: str) -> str:
    """'FR_hip' → 'front',  'RR_hip' → 'rear'"""
    return 'front' if joint_name.split('_')[0] in ('FR', 'FL') else 'rear'
```

- [ ] **Step 2: Update `_motor_nodes` signature to accept `port_map`**

Replace:

```python
def _motor_nodes(joints: list, serial_port: str, motor_hz: float,
                 kp: float, kd: float) -> list:
    return [
        Node(
            ...
            parameters=[{
                'serial_port': serial_port,
                ...
            }],
        )
        for j in joints
    ]
```

With:

```python
def _motor_nodes(joints: list, port_map: dict, motor_hz: float,
                 kp: float, kd: float) -> list:
    """port_map = {'front': '/dev/ttyUSB0', 'rear': '/dev/ttyUSB1'}"""
    return [
        Node(
            package='unitree_actuator_sdk',
            executable='go_m8010_6_node',
            namespace=_NS_MAP[j['name']],
            name='motor',
            parameters=[{
                'serial_port': port_map[_leg_group(j['name'])],
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
```

- [ ] **Step 3: Update `_launch_setup` to read two ports and build `port_map`**

Replace the top of `_launch_setup` where `serial_port` is resolved:

```python
def _launch_setup(context, *args, **kwargs):
    mode             = LaunchConfiguration('mode').perform(context).lower()
    legs_arg         = LaunchConfiguration('legs').perform(context)
    sp_front         = LaunchConfiguration('serial_port_front').perform(context)
    sp_rear          = LaunchConfiguration('serial_port_rear').perform(context)

    cfg     = _load_config()
    control = cfg['control']
    if sp_front == _YAML_SENTINEL:
        sp_front = control['serial_port_front']
    if sp_rear == _YAML_SENTINEL:
        sp_rear = control['serial_port_rear']
    port_map = {'front': sp_front, 'rear': sp_rear}
    motor_hz = float(control['motor_hz'])

    active_legs = _parse_legs(legs_arg)
    joints = [j for j in cfg['joints']
              if j['name'].split('_')[0] in active_legs]

    if mode == 'passive':
        motors = _motor_nodes(joints, port_map, motor_hz, kp=0.0, kd=0.0)
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
        motors = _motor_nodes(joints, port_map, motor_hz, kp=kp, kd=kd)
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
```

- [ ] **Step 4: Update `generate_launch_description` to declare the two new args**

Replace the `serial_port` `DeclareLaunchArgument` with:

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='passive',
            description='Operating mode: passive | stand | policy',
        ),
        DeclareLaunchArgument(
            'legs', default_value='all',
            description='Legs to activate: all | FR | FL | RR | RL | comma-separated e.g. FR,FL',
        ),
        DeclareLaunchArgument(
            'serial_port_front', default_value=_YAML_SENTINEL,
            description='Serial port for front legs FR/FL (default: from robot.yaml)',
        ),
        DeclareLaunchArgument(
            'serial_port_rear', default_value=_YAML_SENTINEL,
            description='Serial port for rear legs RR/RL (default: from robot.yaml)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
```

- [ ] **Step 5: Also update the module docstring at the top of `robot.launch.py`**

Replace:

```
  serial_port   [from robot.yaml]   Override serial port for all motors
...
  ros2 launch legged_control robot.launch.py mode:=passive serial_port:=/dev/ttyUSB1
```

With:

```
  serial_port_front   [from robot.yaml]   Override serial port for FR/FL motors
  serial_port_rear    [from robot.yaml]   Override serial port for RR/RL motors
...
  ros2 launch legged_control robot.launch.py serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1
```

---

### Task 3: Update `robot.yaml`

**Files:**
- Modify: `src/legged_control/config/robot.yaml`

- [ ] **Step 1: Replace single `serial_port` with two fields**

In `robot.yaml` under `control:`, replace:

```yaml
  serial_port: /dev/ttyUSB0
```

With:

```yaml
  serial_port_front: /dev/ttyUSB0   # FR, FL legs
  serial_port_rear:  /dev/ttyUSB1   # RR, RL legs
```

---

### Task 4: Run tests and commit

- [ ] **Step 1: Run all tests**

```bash
python -m pytest src/legged_control/tests/ -v
```

Expected: all tests PASS including `test_leg_group_front` and `test_leg_group_rear`.

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/launch/robot.launch.py \
        src/legged_control/config/robot.yaml \
        src/legged_control/tests/test_robot_launch.py
git commit -m "feat: dual serial port support (front FR/FL, rear RR/RL)"
```

---

### Task 5: Update CLAUDE.md

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update the launch parameter table**

Find the launch parameter table and replace the `serial_port` row with:

```markdown
| `serial_port_front` | from `robot.yaml` | serial port for FR/FL motors |
| `serial_port_rear`  | from `robot.yaml` | serial port for RR/RL motors |
```

- [ ] **Step 2: Update the example override command**

Replace:

```bash
ros2 launch legged_control robot.launch.py serial_port:=/dev/ttyUSB1
```

With:

```bash
ros2 launch legged_control robot.launch.py serial_port_front:=/dev/ttyUSB0 serial_port_rear:=/dev/ttyUSB1
```

- [ ] **Step 3: Update the Configuration section**

Find the `robot.yaml` example block in CLAUDE.md and replace `serial_port: /dev/ttyUSB0` with:

```yaml
  serial_port_front: /dev/ttyUSB0   # FR, FL legs
  serial_port_rear:  /dev/ttyUSB1   # RR, RL legs
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md for dual serial port args"
```
