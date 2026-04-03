# Calibration Script Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an interactive Python calibration script that guides a user through suspended + ground robot initialization, auto-writing motor IDs, default joint angles, and PD gains to `robot.yaml`.

**Architecture:** Single `calibrate.py` entry point orchestrates five sequential phases. Business logic lives in `legged_control/calibration/` subpackage. A `RosClient` rclpy node runs in a background thread, feeding thread-safe state to phase code. An `EStopMonitor` thread watches `/joy` independently and can pause execution at any point.

**Tech Stack:** Python 3.10+, rclpy (ROS2 Humble), ruamel.yaml, colorama, pytest + unittest.mock

---

## File Map

| File | Role |
|------|------|
| `src/legged_control/scripts/calibrate.py` | Entry point — loads config, runs phases in sequence |
| `src/legged_control/legged_control/calibration/__init__.py` | Empty |
| `src/legged_control/legged_control/calibration/ui.py` | Colors, phase banners, prompts, live table |
| `src/legged_control/legged_control/calibration/config_io.py` | ruamel.yaml read/patch/write |
| `src/legged_control/legged_control/calibration/oscillation.py` | Sliding-window oscillation detector |
| `src/legged_control/legged_control/calibration/ros_client.py` | rclpy Node — subscribes 12 motor topics + /joy; publishes /joint_commands |
| `src/legged_control/legged_control/calibration/motor_manager.py` | subprocess lifecycle for 12 motor nodes |
| `src/legged_control/legged_control/calibration/estop.py` | Background thread — monitors ESTOP button, triggers pause |
| `src/legged_control/legged_control/calibration/phases/__init__.py` | Empty |
| `src/legged_control/legged_control/calibration/phases/phase0.py` | Env checks + gamepad button binding |
| `src/legged_control/legged_control/calibration/phases/phase1.py` | Suspended: motor ID mapping |
| `src/legged_control/legged_control/calibration/phases/phase2.py` | Suspended: default_q sampling |
| `src/legged_control/legged_control/calibration/phases/phase3.py` | Suspended: PD gain tuning |
| `src/legged_control/legged_control/calibration/phases/phase4.py` | Ground: full-stack verification |
| `src/legged_control/tests/calibration/__init__.py` | Empty |
| `src/legged_control/tests/calibration/test_config_io.py` | |
| `src/legged_control/tests/calibration/test_oscillation.py` | |
| `src/legged_control/tests/calibration/test_phase1_logic.py` | |
| `src/legged_control/tests/calibration/test_phase3_logic.py` | |
| **Modify** `src/legged_control/setup.py` | Add scripts/, tests/, calibration subpackage |

---

## Task 1: Package structure + dependencies

**Files:**
- Modify: `src/legged_control/setup.py`
- Create: all `__init__.py` files and directory skeletons

- [ ] **Step 1: Create directories**

```bash
cd src/legged_control
mkdir -p scripts
mkdir -p legged_control/calibration/phases
mkdir -p tests/calibration
touch legged_control/calibration/__init__.py
touch legged_control/calibration/phases/__init__.py
touch tests/__init__.py
touch tests/calibration/__init__.py
```

- [ ] **Step 2: Update setup.py to include new subpackage and scripts**

Replace the `setup()` call in `src/legged_control/setup.py`:

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
        ('share/' + package_name + '/launch', ['launch/locomotion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'joint_aggregator = legged_control.joint_aggregator:main',
            'teleop_node      = legged_control.teleop_node:main',
            'policy_node      = legged_control.policy_node:main',
        ],
    },
)
```

- [ ] **Step 3: Install Python dependencies on onboard computer**

```bash
pip install ruamel.yaml colorama pytest
```

Expected: no errors; `python3 -c "import ruamel.yaml, colorama"` exits cleanly.

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/
git commit -m "feat(calibration): add package skeleton and deps"
```

---

## Task 2: UI helpers

**Files:**
- Create: `src/legged_control/legged_control/calibration/ui.py`

- [ ] **Step 1: Write ui.py**

```python
"""Terminal UI helpers for the calibration script."""

import shutil
from colorama import Fore, Style, init as _colorama_init

_colorama_init(autoreset=True)

_ESTOP_LABEL: str = '[Ctrl+C=ESTOP]'


def set_estop_label(label: str) -> None:
    """Call after gamepad binding to show button name in prompts."""
    global _ESTOP_LABEL
    _ESTOP_LABEL = label


def phase_banner(number: int, title: str) -> None:
    width = shutil.get_terminal_size().columns
    bar = '─' * width
    print(f'\n{Fore.CYAN}{bar}')
    print(f'  Phase {number}: {title}')
    print(f'{bar}{Style.RESET_ALL}\n')


def info(msg: str) -> None:
    print(f'{Fore.WHITE}{msg}{Style.RESET_ALL}')


def success(msg: str) -> None:
    print(f'{Fore.GREEN}✓ {msg}{Style.RESET_ALL}')


def warn(msg: str) -> None:
    print(f'{Fore.YELLOW}⚠ {msg}{Style.RESET_ALL}')


def error(msg: str) -> None:
    print(f'{Fore.RED}✗ {msg}{Style.RESET_ALL}')


def prompt(msg: str) -> str:
    """Print a prompt with ESTOP reminder and return stripped input."""
    return input(f'{Fore.CYAN}{_ESTOP_LABEL} {msg}{Style.RESET_ALL} ').strip()


def confirm(msg: str) -> bool:
    """Ask a yes/no question. Returns True for 'y'."""
    answer = prompt(f'{msg} [y/n]:')
    return answer.lower() == 'y'


def joint_table(rows: list[tuple[str, float]]) -> None:
    """Print a two-column table: joint name | value (rad)."""
    print(f'\n  {"Joint":<14} {"Angle (rad)":>12}')
    print(f'  {"─"*14} {"─"*12}')
    for name, val in rows:
        print(f'  {name:<14} {val:>+12.4f}')
    print()
```

- [ ] **Step 2: Verify import works**

```bash
cd src/legged_control
python3 -c "from legged_control.calibration.ui import phase_banner, info; phase_banner(0, 'Test'); info('ok')"
```

Expected: colored banner and "ok" printed.

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/legged_control/calibration/ui.py
git commit -m "feat(calibration): add terminal UI helpers"
```

---

## Task 3: Config I/O

**Files:**
- Create: `src/legged_control/legged_control/calibration/config_io.py`
- Create: `src/legged_control/tests/calibration/test_config_io.py`

- [ ] **Step 1: Write failing test**

```python
# tests/calibration/test_config_io.py
import pytest, textwrap, pathlib, tempfile
from legged_control.calibration.config_io import ConfigIO

YAML = textwrap.dedent("""\
    # Top comment
    control:
      kp: 20.0  # gain
      kd: 0.5
    joints:
      - name: FR_hip
        motor_id: 0
        default_q: 0.0
""")

@pytest.fixture
def tmp_yaml(tmp_path):
    p = tmp_path / 'robot.yaml'
    p.write_text(YAML)
    return p

def test_read_returns_dict(tmp_yaml):
    cfg = ConfigIO(tmp_yaml).read()
    assert cfg['control']['kp'] == 20.0

def test_patch_preserves_comments(tmp_yaml):
    io = ConfigIO(tmp_yaml)
    io.patch({'control': {'kp': 30.0}})
    text = tmp_yaml.read_text()
    assert '# Top comment' in text
    assert '# gain' in text
    assert '30.0' in text

def test_patch_joint_field(tmp_yaml):
    io = ConfigIO(tmp_yaml)
    io.patch_joint('FR_hip', 'default_q', 0.42)
    cfg = ConfigIO(tmp_yaml).read()
    assert abs(cfg['joints'][0]['default_q'] - 0.42) < 1e-6

def test_patch_joint_motor_id(tmp_yaml):
    io = ConfigIO(tmp_yaml)
    io.patch_joint('FR_hip', 'motor_id', 7)
    cfg = ConfigIO(tmp_yaml).read()
    assert cfg['joints'][0]['motor_id'] == 7
```

- [ ] **Step 2: Run test — expect failure**

```bash
cd src/legged_control
python3 -m pytest tests/calibration/test_config_io.py -v
```

Expected: `ModuleNotFoundError: No module named 'legged_control.calibration.config_io'`

- [ ] **Step 3: Write config_io.py**

```python
"""ruamel.yaml-based config reader/writer that preserves comments."""

from __future__ import annotations
import pathlib
from ruamel.yaml import YAML


class ConfigIO:
    def __init__(self, path: str | pathlib.Path) -> None:
        self._path = pathlib.Path(path)
        self._yaml = YAML()
        self._yaml.preserve_quotes = True

    def read(self) -> dict:
        with open(self._path) as f:
            return self._yaml.load(f)

    def patch(self, updates: dict) -> None:
        """Deep-merge `updates` into the yaml file, preserving comments."""
        data = self.read()
        _deep_merge(data, updates)
        with open(self._path, 'w') as f:
            self._yaml.dump(data, f)

    def patch_joint(self, joint_name: str, field: str, value) -> None:
        """Update a single field on the joint entry matching `joint_name`."""
        data = self.read()
        for entry in data.get('joints', []):
            if entry.get('name') == joint_name:
                entry[field] = value
                break
        with open(self._path, 'w') as f:
            self._yaml.dump(data, f)


def _deep_merge(base: dict, updates: dict) -> None:
    for k, v in updates.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
```

- [ ] **Step 4: Run test — expect pass**

```bash
python3 -m pytest tests/calibration/test_config_io.py -v
```

Expected: 4 PASSED

- [ ] **Step 5: Commit**

```bash
git add legged_control/calibration/config_io.py tests/calibration/test_config_io.py
git commit -m "feat(calibration): add ConfigIO with comment-preserving yaml r/w"
```

---

## Task 4: Oscillation detector

**Files:**
- Create: `src/legged_control/legged_control/calibration/oscillation.py`
- Create: `src/legged_control/tests/calibration/test_oscillation.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/calibration/test_oscillation.py
from legged_control.calibration.oscillation import OscillationDetector

def test_detects_rapid_reversals():
    d = OscillationDetector(window_sec=1.0, threshold=3)
    # 4 sign reversals in 0.4 s → oscillating
    for t, v in [(0.0, 1.0), (0.1, -1.0), (0.2, 1.0), (0.3, -1.0), (0.4, 1.0)]:
        d.update(t, v)
    assert d.is_oscillating()

def test_stable_motion_not_oscillating():
    d = OscillationDetector(window_sec=1.0, threshold=3)
    for t, v in [(0.0, 1.0), (0.2, 0.8), (0.4, 0.3), (0.6, 0.0)]:
        d.update(t, v)
    assert not d.is_oscillating()

def test_old_samples_expire():
    d = OscillationDetector(window_sec=0.5, threshold=3)
    # Reversals older than window should not count
    for t, v in [(0.0, 1.0), (0.1, -1.0), (0.2, 1.0), (0.3, -1.0)]:
        d.update(t, v)
    # Advance time past window, add stable sample
    d.update(1.0, 0.1)
    assert not d.is_oscillating()

def test_reset_clears_history():
    d = OscillationDetector(window_sec=1.0, threshold=3)
    for t, v in [(0.0, 1.0), (0.1, -1.0), (0.2, 1.0), (0.3, -1.0), (0.4, 1.0)]:
        d.update(t, v)
    assert d.is_oscillating()
    d.reset()
    assert not d.is_oscillating()
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest tests/calibration/test_oscillation.py -v
```

Expected: `ModuleNotFoundError`

- [ ] **Step 3: Write oscillation.py**

```python
"""Sliding-window oscillation detector based on velocity sign reversals."""

from __future__ import annotations


class OscillationDetector:
    """Detects oscillation by counting velocity sign reversals in a time window.

    Args:
        window_sec: Length of the sliding window in seconds.
        threshold:  Number of sign reversals that triggers oscillation.
    """

    def __init__(self, window_sec: float = 1.0, threshold: int = 3) -> None:
        self._window = window_sec
        self._threshold = threshold
        self._history: list[tuple[float, float]] = []  # (timestamp, velocity)

    def update(self, t: float, velocity: float) -> None:
        """Record a new (time, velocity) sample."""
        self._history.append((t, velocity))
        cutoff = t - self._window
        self._history = [(ts, v) for ts, v in self._history if ts >= cutoff]

    def is_oscillating(self) -> bool:
        """Return True if sign reversals in the current window exceed threshold."""
        reversals = 0
        prev_sign: int | None = None
        for _, v in self._history:
            if v == 0.0:
                continue
            sign = 1 if v > 0 else -1
            if prev_sign is not None and sign != prev_sign:
                reversals += 1
            prev_sign = sign
        return reversals >= self._threshold

    def reset(self) -> None:
        self._history.clear()
```

- [ ] **Step 4: Run — expect pass**

```bash
python3 -m pytest tests/calibration/test_oscillation.py -v
```

Expected: 4 PASSED

- [ ] **Step 5: Commit**

```bash
git add legged_control/calibration/oscillation.py tests/calibration/test_oscillation.py
git commit -m "feat(calibration): add sliding-window oscillation detector"
```

---

## Task 5: ROS client

**Files:**
- Create: `src/legged_control/legged_control/calibration/ros_client.py`

- [ ] **Step 1: Write ros_client.py**

```python
"""rclpy node providing thread-safe access to joint states and joystick."""

from __future__ import annotations
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Joy
from nav_msgs.msg import Odometry

# Namespaces matching all_motors.launch.py
_MOTOR_NAMESPACES = [
    'fr/hip', 'fr/thigh', 'fr/calf',
    'fl/hip', 'fl/thigh', 'fl/calf',
    'rr/hip', 'rr/thigh', 'rr/calf',
    'rl/hip', 'rl/thigh', 'rl/calf',
]


class RosClient(Node):
    """Subscribes to all 12 motor joint_states, /joy, /odin1/imu, /odin1/odometry.
    Publishes to /joint_commands.

    All public getters are thread-safe.
    """

    def __init__(self) -> None:
        super().__init__('calibrator')
        self._lock = threading.Lock()

        # Joint state cache: name → (position, velocity)
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}

        # Latest joy and imu/odom for Phase 4 display
        self._joy: Joy | None = None
        self._imu: Imu | None = None
        self._odom: Odometry | None = None

        # Subscribe to each motor's individual topic
        self._subs = []
        for ns in _MOTOR_NAMESPACES:
            self._subs.append(self.create_subscription(
                JointState, f'/{ns}/joint_states',
                self._make_joint_cb(), 10))

        self.create_subscription(Joy,      '/joy',              self._on_joy,  10)
        self.create_subscription(Imu,      '/odin1/imu',        self._on_imu,  10)
        self.create_subscription(Odometry, '/odin1/odometry',   self._on_odom, 10)

        self._cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

    # ---------------------------------------------------------------- callbacks

    def _make_joint_cb(self):
        def cb(msg: JointState) -> None:
            with self._lock:
                for i, name in enumerate(msg.name):
                    if i < len(msg.position):
                        self._joint_pos[name] = msg.position[i]
                    if i < len(msg.velocity):
                        self._joint_vel[name] = msg.velocity[i]
        return cb

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._joy = msg

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self._imu = msg

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom = msg

    # ----------------------------------------------------------------- getters

    def get_joint_positions(self) -> dict[str, float]:
        with self._lock:
            return dict(self._joint_pos)

    def get_joint_velocities(self) -> dict[str, float]:
        with self._lock:
            return dict(self._joint_vel)

    def get_joy(self) -> Joy | None:
        with self._lock:
            return self._joy

    def get_imu(self) -> Imu | None:
        with self._lock:
            return self._imu

    def get_odom(self) -> Odometry | None:
        with self._lock:
            return self._odom

    # ----------------------------------------------------------------- command

    def send_command(self, targets: dict[str, float]) -> None:
        """Publish target joint positions to /joint_commands."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(targets.keys())
        msg.position = list(targets.values())
        self._cmd_pub.publish(msg)
```

- [ ] **Step 2: Verify import**

```bash
python3 -c "from legged_control.calibration.ros_client import RosClient; print('ok')"
```

Expected: `ok` (no rclpy init needed for import)

- [ ] **Step 3: Commit**

```bash
git add legged_control/calibration/ros_client.py
git commit -m "feat(calibration): add RosClient node with thread-safe joint/joy access"
```

---

## Task 6: Motor process manager

**Files:**
- Create: `src/legged_control/legged_control/calibration/motor_manager.py`

- [ ] **Step 1: Write motor_manager.py**

```python
"""Manages subprocess lifecycle for all 12 motor driver nodes."""

from __future__ import annotations
import subprocess
import time


def _joint_name_to_ns(name: str) -> str:
    """'FR_hip' → 'fr/hip'"""
    leg, joint = name.split('_', 1)
    return f'{leg.lower()}/{joint.lower()}'


class MotorManager:
    """Launches and terminates motor driver nodes as subprocesses.

    Args:
        joints_cfg: List of joint dicts from robot.yaml (name, motor_id, default_q, ...).
        serial_port: Serial port string e.g. '/dev/ttyUSB0'.
    """

    def __init__(self, joints_cfg: list[dict], serial_port: str) -> None:
        self._joints = joints_cfg
        self._serial = serial_port
        self._procs: list[subprocess.Popen] = []

    def launch(self, kp: float = 0.0, kd: float = 0.0,
               targets: dict[str, float] | None = None) -> None:
        """Start all motor nodes. If targets is None, each joint uses its default_q."""
        self.shutdown()
        for j in self._joints:
            ns = _joint_name_to_ns(j['name'])
            target_q = (targets or {}).get(j['name'], j['default_q'])
            cmd = [
                'ros2', 'run', 'unitree_actuator_sdk', 'go_m8010_6_node',
                '--ros-args',
                '-r', f'__ns:=/{ns}',
                '-p', f'motor_id:={j["motor_id"]}',
                '-p', f'joint_name:={j["name"]}',
                '-p', f'serial_port:={self._serial}',
                '-p', 'loop_hz:=1000.0',
                '-p', f'kp:={kp}',
                '-p', f'kd:={kd}',
                '-p', f'target_q:={target_q}',
                '-p', 'target_dq:=0.0',
                '-p', 'tau:=0.0',
            ]
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._procs.append(proc)
        time.sleep(1.0)   # allow nodes to initialize

    def shutdown(self) -> None:
        """Terminate all motor nodes."""
        for p in self._procs:
            p.terminate()
        for p in self._procs:
            try:
                p.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                p.kill()
        self._procs.clear()

    def zero_torque(self) -> None:
        """Relaunch all nodes with kp=kd=0 (free-wheeling)."""
        cfg = {j['name']: j['default_q'] for j in self._joints}
        self.launch(kp=0.0, kd=0.0, targets=cfg)
```

- [ ] **Step 2: Verify import**

```bash
python3 -c "from legged_control.calibration.motor_manager import MotorManager; print('ok')"
```

Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add legged_control/calibration/motor_manager.py
git commit -m "feat(calibration): add MotorManager subprocess lifecycle handler"
```

---

## Task 7: ESTOP monitor

**Files:**
- Create: `src/legged_control/legged_control/calibration/estop.py`

- [ ] **Step 1: Write estop.py**

```python
"""Background thread that monitors a gamepad button for emergency stop."""

from __future__ import annotations
import threading
from typing import Callable

from sensor_msgs.msg import Joy


class EStopMonitor:
    """Watches /joy via a RosClient and fires a callback when the ESTOP button
    is pressed.

    After the callback fires, the monitor is automatically re-armed so it will
    fire again if pressed a second time.

    Args:
        ros_client: A RosClient instance already spinning.
        button_index: Index into Joy.buttons array.
        on_estop: Callable invoked (from the monitor thread) when button pressed.
    """

    def __init__(self,
                 ros_client,       # RosClient — avoid circular import
                 button_index: int,
                 on_estop: Callable[[], None]) -> None:
        self._client = ros_client
        self._btn = button_index
        self._on_estop = on_estop
        self._running = False
        self._thread: threading.Thread | None = None
        self._prev_pressed = False

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        import time
        while self._running:
            joy = self._client.get_joy()
            if joy is not None and self._btn < len(joy.buttons):
                pressed = bool(joy.buttons[self._btn])
                # Rising edge detection — fire once per press
                if pressed and not self._prev_pressed:
                    self._on_estop()
                self._prev_pressed = pressed
            time.sleep(0.02)   # 50 Hz polling


class PauseController:
    """Coordinates pause/resume state between EStopMonitor and phase code.

    Usage in a phase:
        pause_ctrl.check()   # call periodically; blocks if paused
    """

    def __init__(self, motor_manager, ui_module) -> None:
        self._mm = motor_manager
        self._ui = ui_module
        self._paused = threading.Event()
        self._paused.set()   # not paused initially (set = allowed to proceed)

    def trigger(self) -> None:
        """Called by EStopMonitor. Zeros torque and enters paused state."""
        self._paused.clear()
        self._mm.zero_torque()
        self._ui.warn('\nEMERGENCY STOP — motors zeroed.')

    def check(self) -> None:
        """Block until not paused. Call this in any phase loop."""
        if not self._paused.is_set():
            answer = self._ui.prompt('Paused. Continue? [y/n]:')
            if answer.lower() == 'y':
                self._paused.set()
                self._ui.info('Resuming...')
            else:
                self._ui.info('Exiting.')
                raise SystemExit(0)
```

- [ ] **Step 2: Verify import**

```bash
python3 -c "from legged_control.calibration.estop import EStopMonitor, PauseController; print('ok')"
```

Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add legged_control/calibration/estop.py
git commit -m "feat(calibration): add EStopMonitor and PauseController"
```

---

## Task 8: Phase 0 — environment check + gamepad binding

**Files:**
- Create: `src/legged_control/legged_control/calibration/phases/phase0.py`

- [ ] **Step 1: Write phase0.py**

```python
"""Phase 0: environment checks and gamepad emergency-stop binding."""

from __future__ import annotations
import os
import subprocess
import time

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO


def run(config_path: str, ros_client) -> int:
    """Run environment checks and bind ESTOP button.

    Returns the button index bound as ESTOP, or -1 if no gamepad found
    and user chose keyboard-only mode.

    Raises SystemExit on any unrecoverable check failure.
    """
    ui.phase_banner(0, 'Environment Check + Gamepad Setup')

    _check_serial(config_path)
    _check_ros_env()
    _check_yaml_writable(config_path)
    _check_python_deps()

    ui.success('Environment checks passed.')
    return _setup_gamepad(config_path, ros_client)


# ----------------------------------------------------------------- env checks

def _check_serial(config_path: str) -> None:
    from legged_control.calibration.config_io import ConfigIO
    cfg = ConfigIO(config_path).read()
    port = cfg['control']['serial_port']
    if not os.path.exists(port):
        ui.error(f'Serial port {port} not found.')
        ui.info(f'  Fix: plug in the motor USB cable, or update control.serial_port in robot.yaml')
        raise SystemExit(1)
    ui.success(f'Serial port {port} found.')


def _check_ros_env() -> None:
    if not os.environ.get('ROS_DISTRO'):
        ui.error('ROS2 environment not sourced.')
        ui.info('  Fix: source /opt/ros/humble/setup.bash && source install/setup.bash')
        raise SystemExit(1)
    ui.success(f'ROS2 ({os.environ["ROS_DISTRO"]}) sourced.')


def _check_yaml_writable(config_path: str) -> None:
    if not os.access(config_path, os.W_OK):
        ui.error(f'robot.yaml is not writable: {config_path}')
        ui.info('  Fix: chmod u+w ' + config_path)
        raise SystemExit(1)
    ui.success('robot.yaml is writable.')


def _check_python_deps() -> None:
    missing = []
    for pkg in ('ruamel.yaml', 'colorama'):
        try:
            __import__(pkg.replace('.', '_') if '.' in pkg else pkg)
        except ImportError:
            missing.append(pkg)
    # ruamel.yaml imports as 'ruamel'
    try:
        import ruamel.yaml  # noqa: F401
    except ImportError:
        if 'ruamel.yaml' not in missing:
            missing.append('ruamel.yaml')
    if missing:
        ui.error(f'Missing Python packages: {", ".join(missing)}')
        ui.info('  Fix: pip install ' + ' '.join(missing))
        raise SystemExit(1)
    ui.success('Python dependencies present.')


# ---------------------------------------------------------------- gamepad setup

def _setup_gamepad(config_path: str, ros_client) -> int:
    if not os.path.exists('/dev/input/js0'):
        ui.warn('No gamepad found at /dev/input/js0.')
        if not ui.confirm('Continue in keyboard-only mode? (Ctrl+C = only ESTOP)'):
            raise SystemExit(0)
        return -1

    # Start joy_node
    joy_proc = subprocess.Popen(
        ['ros2', 'run', 'joy', 'joy_node'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)   # let joy_node start publishing

    ui.info('\nGamepad detected. Press the button you want as EMERGENCY STOP.')

    button_index = _wait_for_button_press(ros_client)
    ui.info(f'Button {button_index} detected.')

    ui.info(f'Press button {button_index} again to confirm.')
    confirmed = _wait_for_button_press(ros_client)

    if confirmed != button_index:
        ui.warn('Different button pressed. Using first button as ESTOP.')

    ConfigIO(config_path).patch({'teleop': {'btn_emergency_stop': button_index}})
    ui.set_estop_label(f'[BTN{button_index}=ESTOP]')
    ui.success(f'Emergency stop armed on button {button_index}.')
    return button_index


def _wait_for_button_press(ros_client, timeout: float = 30.0) -> int:
    """Block until any Joy button is pressed. Returns button index."""
    import time
    deadline = time.time() + timeout
    prev_buttons: list[int] = []
    while time.time() < deadline:
        joy = ros_client.get_joy()
        if joy is not None:
            buttons = list(joy.buttons)
            if prev_buttons and buttons != prev_buttons:
                for i, (old, new) in enumerate(zip(prev_buttons, buttons)):
                    if old == 0 and new == 1:
                        return i
            prev_buttons = buttons
        time.sleep(0.02)
    ui.error('Timeout waiting for button press.')
    raise SystemExit(1)
```

- [ ] **Step 2: Verify import**

```bash
python3 -c "from legged_control.calibration.phases.phase0 import run; print('ok')"
```

Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add legged_control/calibration/phases/phase0.py
git commit -m "feat(calibration): implement Phase 0 env checks and gamepad binding"
```

---

## Task 9: Phase 1 — motor ID mapping

**Files:**
- Create: `src/legged_control/legged_control/calibration/phases/phase1.py`
- Create: `src/legged_control/tests/calibration/test_phase1_logic.py`

- [ ] **Step 1: Write failing tests for pure logic functions**

```python
# tests/calibration/test_phase1_logic.py
from legged_control.calibration.phases.phase1 import (
    detect_moved_joint,
    validate_mapping,
)

def test_detect_most_moved():
    baseline = {'FR_hip': 0.0, 'FR_thigh': 0.8, 'FR_calf': -1.5}
    current  = {'FR_hip': 0.1, 'FR_thigh': 0.8, 'FR_calf': -2.8}
    name, delta = detect_moved_joint(baseline, current, threshold=0.3)
    assert name == 'FR_calf'
    assert abs(delta - 1.3) < 1e-6

def test_detect_below_threshold_returns_none():
    baseline = {'FR_hip': 0.0}
    current  = {'FR_hip': 0.05}
    assert detect_moved_joint(baseline, current, threshold=0.3) is None

def test_validate_mapping_ok():
    joint_names = ['FR_hip', 'FR_thigh']
    mapping = {'FR_hip': 0, 'FR_thigh': 1}
    gaps, dupes = validate_mapping(joint_names, mapping)
    assert gaps == []
    assert dupes == []

def test_validate_mapping_gap():
    joint_names = ['FR_hip', 'FR_thigh']
    mapping = {'FR_hip': 0}
    gaps, dupes = validate_mapping(joint_names, mapping)
    assert 'FR_thigh' in gaps

def test_validate_mapping_duplicate():
    joint_names = ['FR_hip', 'FR_thigh']
    mapping = {'FR_hip': 0, 'FR_thigh': 0}
    _, dupes = validate_mapping(joint_names, mapping)
    assert len(dupes) > 0
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest tests/calibration/test_phase1_logic.py -v
```

Expected: `ImportError`

- [ ] **Step 3: Write phase1.py**

```python
"""Phase 1: suspended motor ID mapping by manual joint wiggling."""

from __future__ import annotations
import time

from legged_control.calibration import ui


# Order: calves first (most isolated), then thighs, then hips
DETECTION_ORDER = [
    'FR_calf', 'FL_calf', 'RR_calf', 'RL_calf',
    'FR_thigh', 'FL_thigh', 'RR_thigh', 'RL_thigh',
    'FR_hip', 'FL_hip', 'RR_hip', 'RL_hip',
]
_WIGGLE_THRESHOLD = 0.3   # rad
_MAX_RETRIES = 3


def run(joints_cfg: list[dict], ros_client, motor_manager,
        pause_ctrl) -> dict[str, int]:
    """Guide user through motor ID mapping. Returns {joint_name: motor_id}."""
    ui.phase_banner(1, 'Motor ID Mapping (suspended)')
    ui.info('All motors will be set to zero-torque so you can wiggle joints freely.')
    input('Confirm robot is suspended off the ground, then press Enter.')

    motor_manager.zero_torque()
    time.sleep(0.5)

    # Seed baseline angles
    baseline = ros_client.get_joint_positions()
    # motor_id pool: all IDs found from the hardware (0-11 initially)
    available_ids = set(range(12))
    mapping: dict[str, int] = {}   # joint_name → motor_id

    for joint_name in DETECTION_ORDER:
        pause_ctrl.check()
        _detect_one_joint(joint_name, baseline, ros_client,
                          available_ids, mapping, pause_ctrl)
        # Refresh baseline after each detection
        baseline = ros_client.get_joint_positions()

    _resolve_conflicts(joints_cfg, mapping, available_ids)
    ui.success('Motor ID mapping complete.')
    _print_mapping(mapping)
    return mapping


def _detect_one_joint(joint_name: str, baseline: dict[str, float],
                      ros_client, available_ids: set[int],
                      mapping: dict[str, int], pause_ctrl) -> None:
    for attempt in range(_MAX_RETRIES):
        pause_ctrl.check()
        ui.info(f'\n→ Wiggle the joint you believe is [{joint_name}] (large movement):')
        result = _wait_for_wiggle(baseline, ros_client, pause_ctrl)
        if result is None:
            ui.warn('No significant movement detected. Try again.')
            continue

        detected_name, delta = result
        # detected_name is the key in joint_positions dict (joint name from motor)
        # We need to find which motor_id this corresponds to
        # Since motor nodes publish with joint_name, detected_name IS the joint name
        # But at this point motor_id mapping is what we're building.
        # The motor nodes are launched with sequential IDs 0-11 initially.
        # We detect the joint_name that moved; the motor_id comes from joints_cfg.
        ui.info(f'  Detected: [{detected_name}] moved Δ={delta:.3f} rad')

        if ui.confirm(f'  Is this [{joint_name}]?'):
            # Find motor_id for detected_name from current joints_cfg
            motor_id = _name_to_id(detected_name, mapping, available_ids)
            if motor_id is None:
                ui.warn(f'Cannot resolve motor_id for {detected_name}; mark pending.')
                return
            mapping[joint_name] = motor_id
            available_ids.discard(motor_id)
            ui.success(f'  {joint_name} → motor_id={motor_id}')
            return
    ui.warn(f'Could not confirm {joint_name} after {_MAX_RETRIES} attempts. Mark as pending.')


def _wait_for_wiggle(baseline: dict[str, float], ros_client,
                     pause_ctrl, poll_hz: float = 20.0,
                     timeout: float = 30.0):
    """Poll joint positions until any joint moves > threshold. Returns (name, delta) or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        pause_ctrl.check()
        current = ros_client.get_joint_positions()
        result = detect_moved_joint(baseline, current, _WIGGLE_THRESHOLD)
        if result is not None:
            return result
        time.sleep(1.0 / poll_hz)
    return None


def _name_to_id(detected_name: str, mapping: dict[str, int],
                available_ids: set[int]) -> int | None:
    """Return next available motor_id. (IDs are assigned sequentially as detected.)"""
    if not available_ids:
        return None
    return min(available_ids)


def _resolve_conflicts(joints_cfg: list[dict], mapping: dict[str, int],
                       available_ids: set[int]) -> None:
    all_names = [j['name'] for j in joints_cfg]
    gaps, dupes = validate_mapping(all_names, mapping)

    if not gaps and not dupes:
        return

    ui.warn('\nConflicts detected:')
    if gaps:
        ui.warn(f'  Unassigned joints: {gaps}')
    if dupes:
        ui.warn(f'  Duplicate motor_ids: {dupes}')

    ui.info('Resolve each unassigned joint manually.')
    for joint_name in gaps:
        remaining = sorted(available_ids)
        ui.info(f'  Available motor_ids: {remaining}')
        raw = ui.prompt(f'  Enter motor_id for [{joint_name}]:')
        try:
            mid = int(raw)
            mapping[joint_name] = mid
            available_ids.discard(mid)
        except ValueError:
            ui.error(f'Invalid input "{raw}"; skipping {joint_name}.')


def _print_mapping(mapping: dict[str, int]) -> None:
    rows = [(name, mid) for name, mid in sorted(mapping.items())]
    ui.info('\nFinal motor ID mapping:')
    for name, mid in rows:
        ui.info(f'  {name:<14} → motor_id={mid}')


# -------------------------------------------------------- pure logic (testable)

def detect_moved_joint(baseline: dict[str, float],
                       current: dict[str, float],
                       threshold: float = 0.3) -> tuple[str, float] | None:
    """Return (joint_name, delta) of most-moved joint, or None if below threshold."""
    if not current:
        return None
    deltas = {name: abs(current[name] - baseline.get(name, 0.0))
              for name in current}
    max_name = max(deltas, key=deltas.__getitem__)
    max_delta = deltas[max_name]
    if max_delta < threshold:
        return None
    return max_name, max_delta


def validate_mapping(joint_names: list[str],
                     mapping: dict[str, int]) -> tuple[list[str], list[int]]:
    """Return (unassigned_joints, duplicate_motor_ids)."""
    gaps = [n for n in joint_names if n not in mapping]
    seen: dict[int, list[str]] = {}
    for name, mid in mapping.items():
        seen.setdefault(mid, []).append(name)
    dupes = [mid for mid, names in seen.items() if len(names) > 1]
    return gaps, dupes
```

- [ ] **Step 4: Run tests — expect pass**

```bash
python3 -m pytest tests/calibration/test_phase1_logic.py -v
```

Expected: 5 PASSED

- [ ] **Step 5: Commit**

```bash
git add legged_control/calibration/phases/phase1.py tests/calibration/test_phase1_logic.py
git commit -m "feat(calibration): implement Phase 1 motor ID mapping"
```

---

## Task 10: Phase 2 — default_q sampling

**Files:**
- Create: `src/legged_control/legged_control/calibration/phases/phase2.py`

- [ ] **Step 1: Write phase2.py**

```python
"""Phase 2: suspended default pose sampling."""

from __future__ import annotations
import time

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO


def run(joints_cfg: list[dict], config_path: str,
        ros_client, motor_manager, pause_ctrl) -> dict[str, float]:
    """Sample standing pose angles and write to robot.yaml.

    Returns dict of {joint_name: default_q}.
    """
    ui.phase_banner(2, 'Default Pose Sampling (suspended)')
    ui.info('Motors stay at zero-torque. Manually position all legs into')
    ui.info('an approximate standing pose, then press Enter to sample.')

    motor_manager.zero_torque()

    while True:
        pause_ctrl.check()
        input('\nAdjust pose, then press Enter to sample...')
        pause_ctrl.check()

        default_q = _sample(ros_client, duration=0.5, rate_hz=50.0)
        _print_table(default_q)

        if ui.confirm('Write these as default_q to robot.yaml?'):
            _write(config_path, default_q)
            ui.success('default_q written.')
            return default_q
        ui.info('Resampling...')


def _sample(ros_client, duration: float, rate_hz: float) -> dict[str, float]:
    """Continuously sample joint positions and return per-joint mean."""
    samples: dict[str, list[float]] = {}
    n = int(duration * rate_hz)
    for _ in range(n):
        for name, val in ros_client.get_joint_positions().items():
            samples.setdefault(name, []).append(val)
        time.sleep(1.0 / rate_hz)
    return {name: sum(vals) / len(vals) for name, vals in samples.items()}


def _print_table(default_q: dict[str, float]) -> None:
    rows = [(name, val) for name, val in default_q.items()]
    ui.joint_table(rows)


def _write(config_path: str, default_q: dict[str, float]) -> None:
    io = ConfigIO(config_path)
    for name, val in default_q.items():
        io.patch_joint(name, 'default_q', round(float(val), 4))
```

- [ ] **Step 2: Verify import**

```bash
python3 -c "from legged_control.calibration.phases.phase2 import run; print('ok')"
```

Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add legged_control/calibration/phases/phase2.py
git commit -m "feat(calibration): implement Phase 2 default_q sampling"
```

---

## Task 11: Phase 3 — PD gain tuning

**Files:**
- Create: `src/legged_control/legged_control/calibration/phases/phase3.py`
- Create: `src/legged_control/tests/calibration/test_phase3_logic.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/calibration/test_phase3_logic.py
from legged_control.calibration.phases.phase3 import (
    suggest_kd_from_overshoot,
    KP_MAX,
)

def test_suggest_kd_increase_on_overshoot():
    # overshoot > 20% of step size → increase kd
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.25, step=0.1)
    assert result > current_kd

def test_suggest_kd_decrease_on_underdamp():
    # overshoot < 5% of step size → decrease kd (overdamped)
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.01, step=0.1)
    assert result < current_kd

def test_suggest_kd_unchanged_in_good_range():
    current_kd = 0.5
    result = suggest_kd_from_overshoot(current_kd, peak_overshoot=0.08, step=0.1)
    assert result == current_kd

def test_kp_max_constant():
    assert KP_MAX == 40
```

- [ ] **Step 2: Run — expect failure**

```bash
python3 -m pytest tests/calibration/test_phase3_logic.py -v
```

Expected: `ImportError`

- [ ] **Step 3: Write phase3.py**

```python
"""Phase 3: suspended PD gain tuning."""

from __future__ import annotations
import time

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.oscillation import OscillationDetector

KP_MAX = 40
KP_START = 5
KP_STEP = 5
KD_START = 0.5
KD_MAX = 2.0

# Use FR_thigh as the step-response test joint (significant gravity load when suspended)
_STEP_TEST_JOINT = 'FR_thigh'
_STEP_SIZE = 0.1   # rad


def run(joints_cfg: list[dict], config_path: str,
        ros_client, motor_manager, pause_ctrl) -> tuple[float, float]:
    """Tune kp then kd while robot is still suspended.

    Returns (kp, kd).
    """
    ui.phase_banner(3, 'PD Gain Tuning (suspended)')

    default_q = {j['name']: j['default_q'] for j in joints_cfg}

    kp = _tune_kp(default_q, ros_client, motor_manager, pause_ctrl)
    kd = _tune_kd(kp, default_q, ros_client, motor_manager, pause_ctrl)

    ConfigIO(config_path).patch({'control': {'kp': round(kp, 2), 'kd': round(kd, 3)}})
    ui.success(f'Gains written: kp={kp}, kd={kd}')
    return kp, kd


def _tune_kp(default_q: dict[str, float], ros_client,
             motor_manager, pause_ctrl) -> float:
    ui.info(f'\n--- kp sweep (kd fixed at {KD_START}) ---')
    ui.info('Motors will hold the standing pose with increasing stiffness.')
    ui.info('Watch for vibration/oscillation. The script will auto-detect and stop.')

    kp = KP_START
    last_stable_kp = KP_START

    while kp <= KP_MAX:
        pause_ctrl.check()
        ui.info(f'\nApplying kp={kp}, kd={KD_START}...')
        motor_manager.launch(kp=kp, kd=KD_START, targets=default_q)
        time.sleep(0.5)

        if _monitor_oscillation(ros_client, pause_ctrl, duration=2.0):
            ui.warn(f'Oscillation detected at kp={kp}!')
            kp = last_stable_kp
            motor_manager.launch(kp=kp, kd=KD_START, targets=default_q)
            break

        last_stable_kp = kp
        ui.success(f'kp={kp} stable.')

        if kp >= KP_MAX:
            ui.info(f'Reached kp cap ({KP_MAX}).')
            break

        if not ui.confirm('Increase kp further?'):
            break

        kp = min(kp + KP_STEP, KP_MAX)

    recommended = round(last_stable_kp * 0.7, 1)
    ui.info(f'Recommended kp = 0.7 × {last_stable_kp} = {recommended}')
    raw = ui.prompt(f'Accept kp={recommended}? [Enter to accept or type value]:')
    if raw:
        try:
            recommended = float(raw)
        except ValueError:
            ui.warn('Invalid input; using recommended value.')
    ui.success(f'kp set to {recommended}')
    return recommended


def _tune_kd(kp: float, default_q: dict[str, float], ros_client,
             motor_manager, pause_ctrl) -> float:
    ui.info(f'\n--- kd tuning (kp={kp} fixed) ---')
    ui.info(f'Sending a small step to {_STEP_TEST_JOINT} and measuring response.')

    kd = KD_START
    test_joint = _STEP_TEST_JOINT

    motor_manager.launch(kp=kp, kd=kd, targets=default_q)
    time.sleep(0.5)

    # Step: perturb then revert
    step_target = dict(default_q)
    step_target[test_joint] = default_q[test_joint] + _STEP_SIZE
    ros_client.send_command(step_target)
    time.sleep(0.2)
    ros_client.send_command(default_q)

    # Record response for 1 second
    target_val = default_q[test_joint]
    peak_overshoot = _measure_peak_overshoot(
        ros_client, test_joint, target_val, duration=1.0)

    suggested_kd = suggest_kd_from_overshoot(kd, peak_overshoot, _STEP_SIZE)
    ui.info(f'Peak overshoot: {peak_overshoot:.4f} rad  (step={_STEP_SIZE} rad)')
    ui.info(f'Suggested kd: {suggested_kd:.3f}')

    raw = ui.prompt(f'Accept kd={suggested_kd:.3f}? [Enter to accept or type value]:')
    if raw:
        try:
            suggested_kd = float(raw)
        except ValueError:
            ui.warn('Invalid input; using suggested value.')
    suggested_kd = min(suggested_kd, KD_MAX)
    ui.success(f'kd set to {suggested_kd}')
    return suggested_kd


def _monitor_oscillation(ros_client, pause_ctrl,
                         duration: float, poll_hz: float = 50.0) -> bool:
    """Return True if oscillation detected in any joint during `duration` seconds."""
    detectors: dict[str, OscillationDetector] = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        pause_ctrl.check()
        t = time.time()
        for name, vel in ros_client.get_joint_velocities().items():
            d = detectors.setdefault(name, OscillationDetector())
            d.update(t, vel)
            if d.is_oscillating():
                return True
        time.sleep(1.0 / poll_hz)
    return False


def _measure_peak_overshoot(ros_client, joint_name: str,
                             target: float, duration: float) -> float:
    """Return maximum absolute deviation from target after a step response."""
    peak = 0.0
    deadline = time.time() + duration
    while time.time() < deadline:
        pos = ros_client.get_joint_positions().get(joint_name, target)
        peak = max(peak, abs(pos - target))
        time.sleep(0.02)
    return peak


# ------------------------------------------------- pure logic (testable)

def suggest_kd_from_overshoot(current_kd: float, peak_overshoot: float,
                               step: float) -> float:
    """Heuristic kd suggestion based on peak overshoot fraction of step size.

    overshoot_frac > 0.20 → increase kd by 50%
    overshoot_frac < 0.05 → decrease kd by 30%
    else                  → keep current
    """
    frac = peak_overshoot / step if step > 0 else 0.0
    if frac > 0.20:
        return round(min(current_kd * 1.5, KD_MAX), 3)
    if frac < 0.05:
        return round(current_kd * 0.7, 3)
    return current_kd
```

- [ ] **Step 4: Run tests — expect pass**

```bash
python3 -m pytest tests/calibration/test_phase3_logic.py -v
```

Expected: 4 PASSED

- [ ] **Step 5: Commit**

```bash
git add legged_control/calibration/phases/phase3.py tests/calibration/test_phase3_logic.py
git commit -m "feat(calibration): implement Phase 3 PD gain tuning"
```

---

## Task 12: Phase 4 — ground verification

**Files:**
- Create: `src/legged_control/legged_control/calibration/phases/phase4.py`

- [ ] **Step 1: Write phase4.py**

```python
"""Phase 4: ground full-stack verification and optional gain fine-tuning."""

from __future__ import annotations
import subprocess
import time
import numpy as np

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.oscillation import OscillationDetector


def run(joints_cfg: list[dict], config_path: str,
        ros_client, motor_manager, pause_ctrl) -> None:
    ui.phase_banner(4, 'Ground Verification')
    ui.warn('Place the robot on flat ground. Have a safety person nearby.')
    input('Ready? Press Enter.')

    cfg = ConfigIO(config_path).read()
    kp = cfg['control']['kp']
    kd = cfg['control']['kd']
    default_q = {j['name']: j['default_q'] for j in joints_cfg}

    motor_manager.launch(kp=kp, kd=kd, targets=default_q)
    ui.info('Control stack running. Live observation (Ctrl+C or ESTOP to pause):')

    _live_display(ros_client, pause_ctrl, duration=None)

    _fine_tune_loop(kp, kd, default_q, config_path,
                    ros_client, motor_manager, pause_ctrl)

    ui.success('Phase 4 complete. Calibration finished.')


def _live_display(ros_client, pause_ctrl, duration=10.0) -> None:
    """Print observation vector summary until user presses Enter or duration expires."""
    import threading
    stop = threading.Event()

    def _input_waiter():
        input('\n[Press Enter to stop live display]\n')
        stop.set()

    t = threading.Thread(target=_input_waiter, daemon=True)
    t.start()

    deadline = time.time() + duration if duration else float('inf')
    while not stop.is_set() and time.time() < deadline:
        pause_ctrl.check()
        _print_obs(ros_client)
        time.sleep(1.0)


def _print_obs(ros_client) -> None:
    joy = ros_client.get_joy()
    imu = ros_client.get_imu()
    odom = ros_client.get_odom()
    pos = ros_client.get_joint_positions()
    vel = ros_client.get_joint_velocities()

    vx = vy = yaw = 0.0
    if joy:
        # Raw axes — user can verify mapping visually
        vx  = joy.axes[1] if len(joy.axes) > 1 else 0.0
        vy  = joy.axes[0] if len(joy.axes) > 0 else 0.0
        yaw = joy.axes[3] if len(joy.axes) > 3 else 0.0

    gx = gy = gz = 0.0
    if odom:
        q = odom.pose.pose.orientation
        gx, gy, gz = _gravity_in_body(q.x, q.y, q.z, q.w)

    wx = wy = wz = 0.0
    if imu:
        wx, wy, wz = imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z

    print(f'\r  cmd_vel : vx={vx:+.2f} vy={vy:+.2f} yaw={yaw:+.2f}')
    print(f'  gravity : x={gx:+.3f} y={gy:+.3f} z={gz:+.3f}  (flat→~0,0,-1)')
    print(f'  ang_vel : x={wx:+.3f} y={wy:+.3f} z={wz:+.3f}')
    if pos:
        pos_str = '  '.join(f'{n}={v:+.2f}' for n, v in list(pos.items())[:4])
        print(f'  joint_pos (first 4): {pos_str}')
    print()


def _gravity_in_body(qx, qy, qz, qw) -> tuple[float, float, float]:
    q_vec = np.array([qx, qy, qz])
    v = np.array([0.0, 0.0, -1.0])
    a = v * (2.0 * qw**2 - 1.0)
    b = np.cross(q_vec, v) * qw * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    g = a - b + c
    return float(g[0]), float(g[1]), float(g[2])


def _fine_tune_loop(kp: float, kd: float, default_q: dict[str, float],
                    config_path: str, ros_client, motor_manager, pause_ctrl) -> None:
    while True:
        pause_ctrl.check()
        if not ui.confirm('\nAdjust gains?'):
            break
        ui.info(f'Current: kp={kp}, kd={kd}')
        raw_kp = ui.prompt(f'New kp (Enter = keep {kp}):')
        raw_kd = ui.prompt(f'New kd (Enter = keep {kd}):')
        try:
            if raw_kp:
                kp = min(float(raw_kp), 40.0)
            if raw_kd:
                kd = min(float(raw_kd), 2.0)
        except ValueError:
            ui.warn('Invalid input; keeping current values.')
            continue

        motor_manager.launch(kp=kp, kd=kd, targets=default_q)
        _live_display(ros_client, pause_ctrl, duration=10.0)

    ConfigIO(config_path).patch({'control': {'kp': round(kp, 2), 'kd': round(kd, 3)}})
    ui.success(f'Final gains written: kp={kp}, kd={kd}')
```

- [ ] **Step 2: Verify import**

```bash
python3 -c "from legged_control.calibration.phases.phase4 import run; print('ok')"
```

Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add legged_control/calibration/phases/phase4.py
git commit -m "feat(calibration): implement Phase 4 ground verification"
```

---

## Task 13: Main entry point

**Files:**
- Create: `src/legged_control/scripts/calibrate.py`

- [ ] **Step 1: Write calibrate.py**

```python
#!/usr/bin/env python3
"""
calibrate.py — Robot initialization and calibration script.

Usage:
    source /opt/ros/humble/setup.bash
    source ~/rc/legged_ws/install/setup.bash
    python3 src/legged_control/scripts/calibrate.py [--config PATH]
"""

import argparse
import signal
import sys
import threading

import rclpy

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO
from legged_control.calibration.ros_client import RosClient
from legged_control.calibration.motor_manager import MotorManager
from legged_control.calibration.estop import EStopMonitor, PauseController
from legged_control.calibration.phases import phase0, phase1, phase2, phase3, phase4

from ament_index_python.packages import get_package_share_directory
import os


def _default_config_path() -> str:
    share = get_package_share_directory('legged_control')
    return os.path.join(share, 'config', 'robot.yaml')


def main() -> None:
    parser = argparse.ArgumentParser(description='Legged robot calibration script')
    parser.add_argument('--config', default=_default_config_path(),
                        help='Path to robot.yaml')
    args = parser.parse_args()
    config_path = args.config

    # Read config
    cfg = ConfigIO(config_path).read()
    joints_cfg  = cfg['joints']
    serial_port = cfg['control']['serial_port']

    # Init ROS
    rclpy.init()
    ros_client = RosClient()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_client,), daemon=True)
    spin_thread.start()

    motor_manager = MotorManager(joints_cfg, serial_port)

    # Signal handler: zero torque + exit on Ctrl+C
    def _sigint(sig, frame):
        ui.warn('\nCtrl+C — zeroing all motors and exiting.')
        motor_manager.zero_torque()
        motor_manager.shutdown()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    pause_ctrl = PauseController(motor_manager, ui)

    try:
        # Phase 0: env check + gamepad
        btn_index = phase0.run(config_path, ros_client)

        if btn_index >= 0:
            estop = EStopMonitor(ros_client, btn_index, pause_ctrl.trigger)
            estop.start()

        # Phase 1: motor ID mapping
        mapping = phase1.run(joints_cfg, ros_client, motor_manager, pause_ctrl)
        # Update joints_cfg with confirmed motor_ids
        for j in joints_cfg:
            if j['name'] in mapping:
                j['motor_id'] = mapping[j['name']]
                ConfigIO(config_path).patch_joint(j['name'], 'motor_id', j['motor_id'])

        # Phase 2: default_q sampling
        phase2.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)
        # Reload default_q after write
        joints_cfg = ConfigIO(config_path).read()['joints']

        # Phase 3: PD gain tuning (still suspended)
        phase3.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)

        # Phase 4: ground verification
        phase4.run(joints_cfg, config_path, ros_client, motor_manager, pause_ctrl)

        ui.success('\n=== Calibration complete. robot.yaml updated. ===')

    except SystemExit as e:
        raise
    finally:
        motor_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Make executable**

```bash
chmod +x src/legged_control/scripts/calibrate.py
```

- [ ] **Step 3: Run all tests**

```bash
cd src/legged_control
python3 -m pytest tests/calibration/ -v
```

Expected: all tests PASSED

- [ ] **Step 4: Verify script imports cleanly**

```bash
python3 -c "import src.legged_control.scripts.calibrate" 2>&1 | head -5
# Or after install:
python3 src/legged_control/scripts/calibrate.py --help
```

Expected: usage message printed, no errors.

- [ ] **Step 5: Commit**

```bash
git add scripts/calibrate.py
git commit -m "feat(calibration): add main entry point and wire all phases"
```

---

## Self-Review

**Spec coverage:**
- Phase 0 env check ✓ | gamepad binding ✓
- Phase 1 motor ID mapping, calf→thigh→hip order ✓ | conflict resolution ✓ | 3 retries ✓
- Phase 2 zero-torque whole-body sampling, 0.5s mean ✓ | resample loop ✓
- Phase 3 kp sweep with oscillation guard ✓ | kp cap 40 ✓ | kd step response ✓ | kd cap 2.0 ✓
- Phase 4 live obs display ✓ | fine-tune loop ✓
- Joint limit clamp: ⚠ — `motor_manager.launch()` passes `target_q` from `default_q`; the motor driver itself clamps via `q_min/q_max` parameters. Explicit clamp should be added to `motor_manager.launch()`.
- ESTOP pause/resume ✓ | Ctrl+C zero-torque exit ✓
- ruamel.yaml comment preservation ✓
- All output written to robot.yaml ✓

**Fix — add clamp to motor_manager.launch():**

In `motor_manager.py`, update `launch()` to clamp target_q:

```python
def launch(self, kp=0.0, kd=0.0, targets=None):
    self.shutdown()
    for j in self._joints:
        ns = _joint_name_to_ns(j['name'])
        raw_q = (targets or {}).get(j['name'], j['default_q'])
        target_q = max(j.get('q_min', -999), min(j.get('q_max', 999), raw_q))
        # ... rest unchanged
```

Add this fix in the commit for Task 6 or as a separate fixup commit.

**Placeholder scan:** None found.

**Type consistency:** `detect_moved_joint` returns `tuple[str, float] | None` — used correctly in `_detect_one_joint`. `validate_mapping` returns `tuple[list[str], list[int]]` — used correctly in `_resolve_conflicts`.
