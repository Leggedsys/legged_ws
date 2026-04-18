# Stance Height Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `stance_height` the single authoritative source for standing height and add real-time height control via gamepad triggers (LT/RT on Betop Kunpeng 20).

**Architecture:** At `GaitNode.__init__`, derive `_default_targets` from `stance_height` via IK instead of reading `default_q` directly; expose a `_rederive_defaults(h)` method that updates both `_default_targets` and `_nominal_feet` z whenever height changes. `teleop_node` reads LT (axis 2) / RT (axis 5) and publishes the height rate as `Twist.linear.z`; `gait_node` integrates this rate into `stance_height` each tick during WAIT/TROT phases.

**Tech Stack:** ROS2 Humble, Python ament_python, PyYAML, pytest, rclpy.parameter

---

## File Map

| File | Change |
|------|--------|
| `src/legged_control/config/robot.yaml` | Add `teleop.axis_lt/rt/max_dz`; add `gait.stance_height_min/max` |
| `src/legged_control/config/robot_sim.yaml` | Same additions |
| `src/legged_control/legged_control/gait_node.py` | Add `_ik_derived_targets`, `_clamp_height` pure fns; add `_rederive_defaults` method; update `__init__`, `_on_cmd_vel`, `_tick` |
| `src/legged_control/legged_control/teleop_node.py` | Read LT/RT axes; publish `linear.z` |
| `src/legged_control/tests/test_gait_node.py` | Add tests for `_ik_derived_targets` and `_clamp_height` |
| `src/legged_control/tests/test_teleop_node.py` | Create new file; test trigger dz computation |

---

## Task 1: Add new YAML config keys

**Files:**
- Modify: `src/legged_control/config/robot.yaml`
- Modify: `src/legged_control/config/robot_sim.yaml`

No tests — these are config values; correctness is verified when gait_node reads them in later tasks.

- [ ] **Step 1: Add keys to `robot.yaml`**

In the `teleop:` section of `src/legged_control/config/robot.yaml`, add after `btn_emergency_stop: -1`:

```yaml
  axis_lt: 2          # left trigger axis  (0.0=released, 1.0=fully pressed)
  axis_rt: 5          # right trigger axis
  max_dz:  0.03       # m/s height rate limit
```

In the `gait:` section, add after `motion_blend_duration: 0.6`:

```yaml
  stance_height_min: 0.20   # m
  stance_height_max: 0.35   # m
```

- [ ] **Step 2: Add same keys to `robot_sim.yaml`**

In `src/legged_control/config/robot_sim.yaml`, add a `teleop:` section at the end:

```yaml
teleop:
  max_vx:  0.05
  max_vy:  0.02
  max_yaw: 0.1
  deadzone: 0.05
  axis_vx:  1
  axis_vy:  0
  axis_yaw: 3
  invert_vx:  true
  invert_vy:  false
  invert_yaw: true
  btn_emergency_stop: -1
  axis_lt: 2
  axis_rt: 5
  max_dz:  0.03
```

In the `gait:` section of `robot_sim.yaml`, add after `fault_hold_kd: 0.2`:

```yaml
  stance_height_min: 0.20
  stance_height_max: 0.35
```

- [ ] **Step 3: Commit**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
git add src/legged_control/config/robot.yaml src/legged_control/config/robot_sim.yaml
git commit -m "config: add stance_height_min/max and trigger axis keys to robot yaml files"
```

---

## Task 2: Add pure helper functions `_ik_derived_targets` and `_clamp_height` with tests

**Files:**
- Modify: `src/legged_control/legged_control/gait_node.py`
- Modify: `src/legged_control/tests/test_gait_node.py`

These are standalone pure functions — no ROS2 dependency — and can be fully unit-tested.

**Key facts for tests (FL leg):**
- `default_q`: hip=0.0, thigh=−0.37373, calf=−1.488672
- `direction/zero_offset`: (1,0.0), (1,1.254), (−1,−2.791)
- `q_urdf = direction × q_motor + zero_offset` → (0.0, 0.88027, −1.302328)
- FK("FL", those angles) → hip-frame foot ≈ (0.0, 0.1127, −0.280)
- Body-frame nominal_xy for FL ≈ (0.1426, 0.1592) (hip_x + fk_x, hip_y + fk_y)
- FK stance height ≈ 0.280 m → roundtrip IK should recover original motor angles

- [ ] **Step 1: Write failing tests in `test_gait_node.py`**

Add these imports to `src/legged_control/tests/test_gait_node.py`:

```python
from legged_control.gait_node import (
    _blend_targets,
    _body_to_hip,
    _clamp_cmd_vel,
    _clamp_height,
    _command_is_fresh,
    _command_with_timeout,
    _foot_target,
    _ik_derived_targets,
    _is_effectively_zero_cmd,
    _leg_stride,
    _motor_targets_from_urdf,
    _phase_is_stance,
    _rate_limit_targets,
)
```

Add these test functions at the end of `test_gait_node.py`:

```python
# ── _ik_derived_targets ──────────────────────────────────────────────────────

_FL_JOINTS = [
    {"direction": 1, "zero_offset": 0.0,    "q_min": -0.533, "q_max":  0.467},
    {"direction": 1, "zero_offset": 1.254,  "q_min": -3.133, "q_max":  0.663},
    {"direction": -1, "zero_offset": -2.791, "q_min": -5.554, "q_max": 0.000},
]
# q_urdf = direction * default_q + zero_offset
_FL_Q_URDF = (0.0, 1 * (-0.37373) + 1.254, -1 * (-1.488672) + (-2.791))
# = (0.0, 0.88027, -1.302328)
_FL_NOMINAL_XY = (0.1426, 0.1592)   # body-frame x,y from FK
_FL_NOMINAL_H  = 0.280              # approximate FK stance height


def test_ik_derived_targets_fl_roundtrip():
    """IK at the nominal FK height should recover the original motor angles."""
    targets = _ik_derived_targets("FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, _FL_NOMINAL_H)
    assert targets is not None
    assert targets[0] == pytest.approx(0.0,        abs=1e-2)
    assert targets[1] == pytest.approx(-0.37373,   abs=1e-2)
    assert targets[2] == pytest.approx(-1.488672,  abs=1e-2)


def test_ik_derived_targets_lower_height_changes_calf():
    """Lowering stance height bends the knee; calf motor angle must change."""
    targets_nom   = _ik_derived_targets("FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, 0.280)
    targets_lower = _ik_derived_targets("FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, 0.220)
    assert targets_nom   is not None
    assert targets_lower is not None
    assert not math.isclose(targets_nom[2], targets_lower[2], abs_tol=0.01)


def test_ik_derived_targets_returns_none_when_unreachable():
    """h=0.40 m exceeds leg reach → IK returns None."""
    targets = _ik_derived_targets("FL", _FL_NOMINAL_XY, _FL_JOINTS, _FL_Q_URDF, 0.40)
    assert targets is None


# ── _clamp_height ────────────────────────────────────────────────────────────

def test_clamp_height_no_clamp():
    assert _clamp_height(0.28, 0.03, 0.02, 0.20, 0.35) == pytest.approx(0.2806)


def test_clamp_height_clamps_max():
    assert _clamp_height(0.345, 0.03, 0.02, 0.20, 0.35) == pytest.approx(0.35)


def test_clamp_height_clamps_min():
    assert _clamp_height(0.205, -0.03, 0.02, 0.20, 0.35) == pytest.approx(0.20)
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source install/setup.bash
pytest src/legged_control/tests/test_gait_node.py -v -k "ik_derived or clamp_height" 2>&1 | tail -20
```

Expected: `ImportError: cannot import name '_ik_derived_targets'` (or similar).

- [ ] **Step 3: Add `_ik_derived_targets` and `_clamp_height` to `gait_node.py`**

Add these two functions immediately after `_motor_targets_from_urdf` (around line 141) in `src/legged_control/legged_control/gait_node.py`:

```python
def _ik_derived_targets(
    leg: str,
    nominal_xy: tuple[float, float],
    leg_joints: list[dict],
    leg_default_q_urdf: tuple[float, float, float],
    h: float,
) -> list[float] | None:
    """Compute motor targets for one leg with foot placed at stance height h (m, positive down).

    Returns None if IK fails (target is unreachable for this leg geometry).
    """
    x_nom, y_nom = nominal_xy
    foot_hip = _body_to_hip(leg, (x_nom, y_nom, -h))
    q_urdf = inverse_kinematics(leg, foot_hip, leg_default_q_urdf)
    if q_urdf is None:
        return None
    targets, _ = _motor_targets_from_urdf(leg_joints, q_urdf)
    return targets


def _clamp_height(
    current_h: float, dz_rate: float, dt: float, h_min: float, h_max: float
) -> float:
    """Integrate dz_rate over dt and clamp the result to [h_min, h_max]."""
    return max(h_min, min(h_max, current_h + dz_rate * dt))
```

- [ ] **Step 4: Rebuild and run tests to confirm they pass**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
colcon build --packages-select legged_control --symlink-install 2>&1 | tail -5
source install/setup.bash
pytest src/legged_control/tests/test_gait_node.py -v 2>&1 | tail -20
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/gait_node.py \
        src/legged_control/tests/test_gait_node.py
git commit -m "feat: add _ik_derived_targets and _clamp_height pure functions with tests"
```

---

## Task 3: Add `_rederive_defaults` method and IK-derived `_default_targets` at init

**Files:**
- Modify: `src/legged_control/legged_control/gait_node.py`

Replace the direct `_default_targets = [joint["default_q"] ...]` assignment in `__init__` with a call to `_rederive_defaults`. After this task, STANDUP will ramp to IK-derived poses at exactly `stance_height`.

- [ ] **Step 1: Add `import rclpy.parameter` near the top of `gait_node.py`**

Find the imports block in `gait_node.py`. After `import rclpy`, add:

```python
import rclpy.parameter
```

The file currently starts with:
```python
import rclpy
from rclpy.node import Node
```

Change to:
```python
import rclpy
import rclpy.parameter
from rclpy.node import Node
```

- [ ] **Step 2: Add `_rederive_defaults` method to `GaitNode`**

Add this method to the `GaitNode` class, after `_load_config` (around line 316):

```python
def _rederive_defaults(self, h: float) -> None:
    """Recompute _default_targets and _nominal_feet z for all legs at stance height h."""
    for leg in _LEG_ORDER:
        x_nom, y_nom, _ = self._nominal_feet[leg]
        targets = _ik_derived_targets(
            leg,
            (x_nom, y_nom),
            self._leg_joints[leg],
            self._leg_default_q_urdf[leg],
            h,
        )
        if targets is None:
            self.get_logger().warn(
                f"[gait] IK failed for {leg} at stance_height={h:.4f} m; "
                "keeping previous _default_targets for that leg"
            )
            continue
        for joint, target in zip(self._leg_joints[leg], targets):
            idx = self._joint_names.index(joint["name"])
            self._default_targets[idx] = target
        self._nominal_feet[leg] = (x_nom, y_nom, -h)
```

- [ ] **Step 3: Replace the direct `_default_targets` init in `__init__` with `_rederive_defaults`**

In `__init__`, find this line (around line 212):

```python
        self._default_targets = [float(joint["default_q"]) for joint in self._joint_cfg]
```

Replace it with (note: `_default_targets` is still initialized from `default_q` first, then overwritten by IK to allow per-leg fallback on IK failure):

```python
        self._default_targets = [float(joint["default_q"]) for joint in self._joint_cfg]
        # Will be overwritten by IK-derived values below; kept as fallback per-leg.
```

Then, after the `fk_stance_height` block (around line 233, after `self.declare_parameter("stance_height", fk_stance_height)`), add a call to `_rederive_defaults`. The placement is tricky: `_rederive_defaults` calls `self.get_logger()` which requires the node to be initialized, and uses `self._nominal_feet` / `self._leg_joints` / etc. which are already set. Add it just before the `self.declare_parameter("step_height", ...)` line. 

Find this section in `__init__`:

```python
        fk_stance_height = -sum(
            self._nominal_feet[leg][2] for leg in _LEG_ORDER
        ) / len(_LEG_ORDER)
        self.declare_parameter(
            "stance_height", fk_stance_height
        )
        self.declare_parameter("step_height", float(gait_cfg.get("step_height", 0.06)))
```

Change to:

```python
        fk_stance_height = -sum(
            self._nominal_feet[leg][2] for leg in _LEG_ORDER
        ) / len(_LEG_ORDER)
        self.declare_parameter(
            "stance_height", fk_stance_height
        )
        self._rederive_defaults(fk_stance_height)
        self.declare_parameter("step_height", float(gait_cfg.get("step_height", 0.06)))
```

- [ ] **Step 4: Rebuild and run all tests**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
colcon build --packages-select legged_control --symlink-install 2>&1 | tail -5
source install/setup.bash
pytest src/legged_control/tests/test_gait_node.py -v 2>&1 | tail -20
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/legged_control/legged_control/gait_node.py
git commit -m "feat: derive _default_targets from IK at init so standup ramps to correct height"
```

---

## Task 4: Add height integration in `_tick` (gait_node.py)

**Files:**
- Modify: `src/legged_control/legged_control/gait_node.py`

After this task, moving the RT trigger raises the robot; LT lowers it. Height changes are silently ignored during STANDUP and FAULT.

- [ ] **Step 1: Declare new parameters in `__init__`**

In `__init__`, find the line:

```python
        self.declare_parameter("skip_standup", False)
```

Before it, add the two new gait parameters:

```python
        self.declare_parameter(
            "stance_height_min", float(gait_cfg.get("stance_height_min", 0.20))
        )
        self.declare_parameter(
            "stance_height_max", float(gait_cfg.get("stance_height_max", 0.35))
        )
        self.declare_parameter("skip_standup", False)
```

- [ ] **Step 2: Add `_dz_rate` instance variable in `__init__`**

Find this block in `__init__` (around line 272):

```python
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._joint_state_seen = False
```

Change to:

```python
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._dz_rate = 0.0
        self._joint_state_seen = False
```

- [ ] **Step 3: Update `_on_cmd_vel` to capture height rate**

Find `_on_cmd_vel`:

```python
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self._last_cmd_t = time.monotonic()
```

Replace with:

```python
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self._dz_rate = float(msg.linear.z)
        self._last_cmd_t = time.monotonic()
```

- [ ] **Step 4: Add `_integrate_height` helper method**

Add this method to `GaitNode` immediately after `_rederive_defaults` (which was added in Task 3 after `_load_config`):

```python
    def _integrate_height(self) -> None:
        """Integrate _dz_rate into stance_height for WAIT/TROT phases."""
        if abs(self._dz_rate) <= 0:
            return
        current_h = float(self.get_parameter("stance_height").value)
        h_min = float(self.get_parameter("stance_height_min").value)
        h_max = float(self.get_parameter("stance_height_max").value)
        new_h = _clamp_height(current_h, self._dz_rate, self._dt, h_min, h_max)
        if abs(new_h - current_h) > 0.0005:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    "stance_height",
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    new_h,
                )
            ])
            self._rederive_defaults(new_h)
```

- [ ] **Step 5: Call `_integrate_height` in `_tick` for WAIT and TROT phases**

In `_tick`, find the WAIT phase block:

```python
        if self._phase == _PHASE_WAIT:
            self._publish_positions(list(self._default_targets))
            if self._joint_state_seen:
```

Replace with:

```python
        if self._phase == _PHASE_WAIT:
            self._integrate_height()
            self._publish_positions(list(self._default_targets))
            if self._joint_state_seen:
```

Find the TROT phase block (at the bottom of `_tick`):

```python
        targets, clipped_any = self._trot_targets()
        self._publish_positions(targets)
        self._update_limit_safety(clipped_any)
```

Replace with:

```python
        self._integrate_height()
        targets, clipped_any = self._trot_targets()
        self._publish_positions(targets)
        self._update_limit_safety(clipped_any)
```

- [ ] **Step 6: Rebuild and run all tests**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
colcon build --packages-select legged_control --symlink-install 2>&1 | tail -5
source install/setup.bash
pytest src/legged_control/tests/test_gait_node.py -v 2>&1 | tail -20
```

Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/legged_control/legged_control/gait_node.py
git commit -m "feat: integrate stance_height from Twist.linear.z in WAIT/TROT phases"
```

---

## Task 5: Update teleop_node to read LT/RT triggers and publish `linear.z`

**Files:**
- Modify: `src/legged_control/legged_control/teleop_node.py`
- Create: `src/legged_control/tests/test_teleop_node.py`

`_scale_axis` already exists and handles deadzone + scaling. The trigger computation is:
`dz = _scale_axis(rt_raw, deadzone, max_dz, invert=False) − _scale_axis(lt_raw, deadzone, max_dz, invert=False)`

Convention (Betop Kunpeng 20 / Linux joy_node): 0.0 = released, 1.0 = fully pressed. Same deadzone as other axes.

- [ ] **Step 1: Write failing tests — create `test_teleop_node.py`**

Create `src/legged_control/tests/test_teleop_node.py`:

```python
import pytest

from legged_control.teleop_node import _scale_axis


def _dz(lt: float, rt: float, deadzone: float = 0.05, max_dz: float = 0.03) -> float:
    """Compute height rate from trigger values (mirrors teleop_node implementation)."""
    return (
        _scale_axis(rt, deadzone, max_dz, invert=False)
        - _scale_axis(lt, deadzone, max_dz, invert=False)
    )


def test_trigger_both_released_gives_zero():
    assert _dz(lt=0.0, rt=0.0) == pytest.approx(0.0)


def test_trigger_rt_fully_pressed_gives_max_dz():
    assert _dz(lt=0.0, rt=1.0) == pytest.approx(0.03)


def test_trigger_lt_fully_pressed_gives_neg_max_dz():
    assert _dz(lt=1.0, rt=0.0) == pytest.approx(-0.03)


def test_trigger_within_deadzone_gives_zero():
    assert _dz(lt=0.0, rt=0.03) == pytest.approx(0.0)   # 0.03 < deadzone 0.05


def test_trigger_partial_differential():
    """Both triggers partially pressed → dz is scaled difference."""
    deadzone, max_dz = 0.05, 0.03
    # _apply_deadzone(0.7) = (0.7 - 0.05) / 0.95 = 0.6842
    # _apply_deadzone(0.3) = (0.3 - 0.05) / 0.95 = 0.2632
    expected = (0.6842 - 0.2632) * max_dz
    assert _dz(lt=0.3, rt=0.7) == pytest.approx(expected, abs=1e-4)


def test_trigger_both_fully_pressed_cancels():
    assert _dz(lt=1.0, rt=1.0) == pytest.approx(0.0)
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
source install/setup.bash
pytest src/legged_control/tests/test_teleop_node.py -v 2>&1 | tail -15
```

Expected: `ImportError` or `AttributeError` since `_dz_rate` functionality is not yet wired. (The pure `_scale_axis` already exists so the tests may actually pass — that's fine, it means the helper is ready and we just need to wire the node class in the next step.)

- [ ] **Step 3: Add trigger reading to `TeleopNode.__init__`**

In `teleop_node.py`, inside `TeleopNode.__init__`, after:

```python
            self._btn_estop  = int(cfg['btn_emergency_stop'])
```

Add:

```python
            self._axis_lt   = int(cfg.get('axis_lt',  2))
            self._axis_rt   = int(cfg.get('axis_rt',  5))
            self._max_dz    = float(cfg.get('max_dz', 0.03))
```

- [ ] **Step 4: Publish `linear.z` in `_on_joy`**

In `_on_joy`, find the `if not estop_active:` block. After:

```python
                twist.angular.z = _safe(self._axis_yaw, self._max_yaw, self._invert_yaw)
```

Add:

```python
                axes = msg.axes
                lt_raw = axes[self._axis_lt] if 0 <= self._axis_lt < len(axes) else 0.0
                rt_raw = axes[self._axis_rt] if 0 <= self._axis_rt < len(axes) else 0.0
                twist.linear.z = (
                    _scale_axis(rt_raw, dz, self._max_dz, invert=False)
                    - _scale_axis(lt_raw, dz, self._max_dz, invert=False)
                )
```

Note: `axes` is already defined earlier in the same `if not estop_active:` block as `axes = msg.axes`. Use that variable directly — do not add a duplicate assignment.

Full `_on_joy` after the change:

```python
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
                    if idx < 0 or idx >= len(axes):
                        self.get_logger().warn(
                            f'Axis index {idx} out of range (axes has {len(axes)} elements)',
                            throttle_duration_sec=5.0,
                        )
                        return 0.0
                    return _scale_axis(axes[idx], dz, max_v, invert)

                twist.linear.x  = _safe(self._axis_vx,  self._max_vx,  self._invert_vx)
                twist.linear.y  = _safe(self._axis_vy,  self._max_vy,  self._invert_vy)
                twist.angular.z = _safe(self._axis_yaw, self._max_yaw, self._invert_yaw)

                lt_raw = axes[self._axis_lt] if 0 <= self._axis_lt < len(axes) else 0.0
                rt_raw = axes[self._axis_rt] if 0 <= self._axis_rt < len(axes) else 0.0
                twist.linear.z = (
                    _scale_axis(rt_raw, dz, self._max_dz, invert=False)
                    - _scale_axis(lt_raw, dz, self._max_dz, invert=False)
                )

            self._pub.publish(twist)
```

- [ ] **Step 5: Rebuild and run all tests**

```bash
cd /home/grayerd/Desktop/Projects/rc/legged_ws
colcon build --packages-select legged_control --symlink-install 2>&1 | tail -5
source install/setup.bash
pytest src/legged_control/tests/ -v 2>&1 | tail -30
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/legged_control/legged_control/teleop_node.py \
        src/legged_control/tests/test_teleop_node.py
git commit -m "feat: read LT/RT triggers in teleop_node and publish stance height rate on Twist.linear.z"
```
