# teleop_node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `teleop_node` — reads gamepad via `/joy`, applies deadzone + scaling, publishes `geometry_msgs/Twist` to `/cmd_vel` for `policy_node`.

**Architecture:** Two pure functions (`_apply_deadzone`, `_scale_axis`) handle all math and are tested without ROS. `TeleopNode` wraps them in a subscriber/publisher node that reads all config from `robot.yaml`'s `teleop` section. The node is wired into the `policy` launch branch alongside `joy_node`.

**Tech Stack:** Python 3, ROS2 Humble (`rclpy`, `sensor_msgs/Joy`, `geometry_msgs/Twist`), `joy` apt package, `pytest`

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/legged_control/legged_control/teleop_node.py` | **Create** | Pure functions + `TeleopNode` class |
| `src/legged_control/tests/test_teleop_node.py` | **Create** | Unit tests for pure functions |
| `src/legged_control/setup.py` | **Modify** | Add `teleop_node` entry point |
| `src/legged_control/package.xml` | **Modify** | Add `<exec_depend>joy</exec_depend>` |
| `src/legged_control/launch/robot.launch.py` | **Modify** | Add `joy_node` + `teleop_node` to policy branch |
| `CLAUDE.md` | **Modify** | Document teleop_node in architecture section |

---

### Task 1: Pure functions — deadzone + scaling (TDD)

**Files:**
- Create: `src/legged_control/tests/test_teleop_node.py`
- Create: `src/legged_control/legged_control/teleop_node.py` (pure functions only)

- [ ] **Step 1: Write the failing tests**

Create `src/legged_control/tests/test_teleop_node.py`:

```python
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from legged_control.teleop_node import _apply_deadzone, _scale_axis


class TestApplyDeadzone:
    def test_inside_deadzone_returns_zero(self):
        # 0.04 < deadzone 0.05 → zero
        assert _apply_deadzone(0.04, 0.05) == 0.0

    def test_negative_inside_deadzone_returns_zero(self):
        assert _apply_deadzone(-0.03, 0.05) == 0.0

    def test_zero_returns_zero(self):
        assert _apply_deadzone(0.0, 0.05) == 0.0

    def test_full_positive_returns_one(self):
        # (1.0 - 0.05) / (1 - 0.05) = 1.0
        assert abs(_apply_deadzone(1.0, 0.05) - 1.0) < 1e-9

    def test_full_negative_returns_minus_one(self):
        assert abs(_apply_deadzone(-1.0, 0.05) - (-1.0)) < 1e-9

    def test_midrange_value(self):
        # raw=0.55, dz=0.05 → (0.55-0.05)/(1-0.05) = 0.50/0.95
        expected = 0.50 / 0.95
        assert abs(_apply_deadzone(0.55, 0.05) - expected) < 1e-9

    def test_continuous_at_deadzone_boundary(self):
        # Exactly at boundary: (0.05-0.05)/(1-0.05) = 0.0, not a jump
        assert abs(_apply_deadzone(0.05, 0.05) - 0.0) < 1e-9


class TestScaleAxis:
    def test_inside_deadzone_returns_zero(self):
        assert _scale_axis(0.0, 0.05, 1.0, False) == 0.0

    def test_full_positive_no_invert(self):
        assert abs(_scale_axis(1.0, 0.05, 1.0, False) - 1.0) < 1e-9

    def test_full_positive_with_invert(self):
        assert abs(_scale_axis(1.0, 0.05, 1.0, True) - (-1.0)) < 1e-9

    def test_scales_by_max_vel(self):
        # max_vel=0.5 halves the output
        assert abs(_scale_axis(1.0, 0.05, 0.5, False) - 0.5) < 1e-9

    def test_negative_input_no_invert(self):
        assert abs(_scale_axis(-1.0, 0.05, 1.0, False) - (-1.0)) < 1e-9

    def test_negative_input_with_invert(self):
        assert abs(_scale_axis(-1.0, 0.05, 1.0, True) - 1.0) < 1e-9
```

- [ ] **Step 2: Run tests — verify they fail**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
python3 -m pytest src/legged_control/tests/test_teleop_node.py -v
```

Expected: `ModuleNotFoundError` or `ImportError` — `teleop_node` does not exist yet.

- [ ] **Step 3: Implement pure functions**

Create `src/legged_control/legged_control/teleop_node.py` with only the pure functions (no ROS imports at module level):

```python
"""
teleop_node — reads /joy, publishes /cmd_vel for policy_node.

Pure functions (_apply_deadzone, _scale_axis) have no ROS2 dependency
and can be unit-tested directly.
"""

import os
import yaml


def _apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone and linearly remap to [-1, 1].

    Inside the deadzone (|value| < deadzone), returns 0.0.
    Outside, linearly remaps so that the deadzone edge maps to 0
    and ±1 maps to ±1 (no discontinuity at the boundary).
    """
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _scale_axis(raw: float, deadzone: float, max_vel: float, invert: bool) -> float:
    """Apply deadzone, scale to physical units, and optionally invert.

    Returns velocity in the same units as max_vel (m/s or rad/s).
    """
    scaled = _apply_deadzone(raw, deadzone) * max_vel
    return -scaled if invert else scaled
```

Do not add the `TeleopNode` class or any `import rclpy` yet — that is Task 2.

- [ ] **Step 4: Run tests — verify they pass**

```bash
python3 -m pytest src/legged_control/tests/test_teleop_node.py -v
```

Expected: 13 tests, all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/teleop_node.py \
        src/legged_control/tests/test_teleop_node.py
git commit -m "feat: add teleop_node pure functions with unit tests"
```

---

### Task 2: TeleopNode class

**Files:**
- Modify: `src/legged_control/legged_control/teleop_node.py` (append node class + main)

- [ ] **Step 1: Append TeleopNode and main to teleop_node.py**

Open `src/legged_control/legged_control/teleop_node.py` and append after the pure functions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


class TeleopNode(Node):
    """Gamepad → /cmd_vel bridge.

    Reads all configuration from robot.yaml teleop section.
    Publishes geometry_msgs/Twist on every /joy message received.
    During e-stop (btn_emergency_stop held), publishes zero Twist every frame
    so policy_node keeps receiving commands and stays in standing posture.
    """

    def __init__(self):
        super().__init__('teleop_node')
        cfg = self._load_teleop_config()

        self._max_vx    = float(cfg['max_vx'])
        self._max_vy    = float(cfg['max_vy'])
        self._max_yaw   = float(cfg['max_yaw'])
        self._deadzone  = float(cfg['deadzone'])
        self._axis_vx   = int(cfg['axis_vx'])
        self._axis_vy   = int(cfg['axis_vy'])
        self._axis_yaw  = int(cfg['axis_yaw'])
        self._invert_vx  = bool(cfg['invert_vx'])
        self._invert_vy  = bool(cfg['invert_vy'])
        self._invert_yaw = bool(cfg['invert_yaw'])
        self._btn_estop  = int(cfg['btn_emergency_stop'])

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.get_logger().info(
            f'teleop_node ready  '
            f'(max_vx={self._max_vx}, max_vy={self._max_vy}, '
            f'max_yaw={self._max_yaw}, deadzone={self._deadzone}, '
            f'btn_estop={self._btn_estop})'
        )

    def _load_teleop_config(self) -> dict:
        share = get_package_share_directory('legged_control')
        with open(os.path.join(share, 'config', 'robot.yaml')) as f:
            return yaml.safe_load(f)['teleop']

    def _on_joy(self, msg: Joy) -> None:
        twist = Twist()

        estop_active = (
            self._btn_estop >= 0
            and self._btn_estop < len(msg.buttons)
            and msg.buttons[self._btn_estop] == 1
        )

        if not estop_active:
            axes = msg.axes
            dz   = self._deadzone

            def _safe(idx: int, max_v: float, invert: bool) -> float:
                if idx >= len(axes):
                    self.get_logger().warn(
                        f'Axis index {idx} out of range (axes has {len(axes)} elements)',
                        throttle_duration_sec=5.0,
                    )
                    return 0.0
                return _scale_axis(axes[idx], dz, max_v, invert)

            twist.linear.x  = _safe(self._axis_vx,  self._max_vx,  self._invert_vx)
            twist.linear.y  = _safe(self._axis_vy,  self._max_vy,  self._invert_vy)
            twist.angular.z = _safe(self._axis_yaw, self._max_yaw, self._invert_yaw)

        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

- [ ] **Step 2: Re-run pure function tests — verify still pass**

The appended imports must not break existing tests:

```bash
python3 -m pytest src/legged_control/tests/test_teleop_node.py -v
```

Expected: 13 tests, all PASS (pure functions are still importable without a running ROS2 environment).

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/legged_control/teleop_node.py
git commit -m "feat: implement TeleopNode (/joy → /cmd_vel, e-stop support)"
```

---

### Task 3: Wiring — setup.py, package.xml, launch, CLAUDE.md

**Files:**
- Modify: `src/legged_control/setup.py`
- Modify: `src/legged_control/package.xml`
- Modify: `src/legged_control/launch/robot.launch.py`
- Modify: `CLAUDE.md`

- [ ] **Step 1: Add entry point in setup.py**

In `src/legged_control/setup.py`, find the `console_scripts` list and add one line:

```python
        'teleop_node         = legged_control.teleop_node:main',
```

The full `entry_points` block should look like:

```python
    entry_points={
        'console_scripts': [
            'passive_monitor_node = legged_control.passive_monitor_node:main',
            'stand_node           = legged_control.stand_node:main',
            'motor_bus_node       = legged_control.motor_bus_node:main',
            'joint_aggregator     = legged_control.joint_aggregator:main',
            'policy_node          = legged_control.policy_node:main',
            'teleop_node         = legged_control.teleop_node:main',
        ],
    },
```

- [ ] **Step 2: Add joy dependency in package.xml**

In `src/legged_control/package.xml`, add after the existing `<depend>` lines and before `<export>`:

```xml
  <exec_depend>joy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
```

The full relevant section should be:

```xml
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <exec_depend>joy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
```

- [ ] **Step 3: Wire into robot.launch.py — policy branch**

In `src/legged_control/launch/robot.launch.py`, find the `if mode == 'policy':` block and replace its return statement to append `joy_node` and `teleop_node`:

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
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
            ),
            Node(
                package='legged_control',
                executable='teleop_node',
                name='teleop_node',
                output='screen',
            ),
        ]
```

- [ ] **Step 4: Update CLAUDE.md**

In `CLAUDE.md`, find the architecture section listing nodes under `legged_control` and add `teleop_node` after `policy_node`:

```
- `teleop_node.py` — subscribes `/joy` (gamepad via `joy_node`); applies deadzone + scaling from `robot.yaml teleop` section; publishes `geometry_msgs/Twist` to `/cmd_vel` for `policy_node`. Auto-started in `policy` mode alongside `joy_node`.
```

Also update the **`policy`** mode description under "Mode descriptions" to mention teleop:

Replace (approximately):
```
**`policy`** — Not yet implemented. Will run the learned locomotion policy.
```

With:
```
**`policy`** — Loads a TorchScript RL model from `robot.yaml policy.model_path` and runs inference at 50 Hz. Gamepad commands (via `joy_node` + `teleop_node`) are read from `/cmd_vel`; falls back to zero velocity if no gamepad is connected.
```

- [ ] **Step 5: Build and verify entry point is registered**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_control
source install/setup.bash
ros2 run legged_control teleop_node --help 2>&1 | head -5
```

Expected: the command resolves (may exit quickly with a ROS init error without a running ROS master — that is fine; what matters is the entry point is found).

- [ ] **Step 6: Run pure function tests one final time**

```bash
python3 -m pytest src/legged_control/tests/test_teleop_node.py -v
```

Expected: 13 tests, all PASS.

- [ ] **Step 7: Commit**

```bash
git add src/legged_control/setup.py \
        src/legged_control/package.xml \
        src/legged_control/launch/robot.launch.py \
        CLAUDE.md
git commit -m "feat: wire teleop_node into policy launch + add joy dependency"
```
