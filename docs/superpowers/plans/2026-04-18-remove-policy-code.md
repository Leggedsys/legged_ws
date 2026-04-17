# Remove Policy Code from dev/position-control Branch — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove all RL policy runtime code from the `dev/position-control` branch so the branch contains only position control, simulation, and hardware infrastructure.

**Architecture:** Pure deletion and reference cleanup — no logic changes. Files `policy_node.py` and `policy_monitor_node.py` are deleted; their references are removed from `robot.launch.py`, `setup.py`, `robot.yaml`, `CLAUDE.md`, and `README.md`. Joint alignment parameters (`direction`, `zero_offset`, `gear_ratio`) are kept because they are used by `kinematics.py` and the Gazebo bridges.

**Tech Stack:** Python, ROS2 Humble, YAML, colcon

---

### Task 1: Delete policy source files and test files

**Files:**
- Delete: `src/legged_control/legged_control/policy_node.py`
- Delete: `src/legged_control/legged_control/policy_monitor_node.py`
- Delete: `src/legged_control/tests/test_policy_node.py`
- Delete: `src/legged_control/tests/test_policy_monitor_node.py`

- [ ] **Step 1: Delete the four files**

```bash
rm src/legged_control/legged_control/policy_node.py
rm src/legged_control/legged_control/policy_monitor_node.py
rm src/legged_control/tests/test_policy_node.py
rm src/legged_control/tests/test_policy_monitor_node.py
```

- [ ] **Step 2: Verify no torch imports remain anywhere**

```bash
grep -r "import torch" src/legged_control/
```

Expected: no output.

---

### Task 2: Remove `policy` mode from `robot.launch.py`

**Files:**
- Modify: `src/legged_control/launch/robot.launch.py`

- [ ] **Step 1: Remove the `policy` mode block (lines 157–196) and update the docstring and error message**

Replace the entire file with the following content (diff described below):

1. **Docstring** (lines 5–6): Remove `| policy` from the mode list. Change to:
   ```
     mode          [passive]           passive | stand | standup | position_control
   ```
   Remove the `mode:=policy` usage example line (line 15 of original).

2. **`if mode == "policy":` block** (lines 157–196): Delete entirely.

3. **Error message** (line 234): Change to:
   ```python
       raise RuntimeError(
           f"Unknown mode '{mode}'. Valid modes: passive, stand, standup, position_control"
       )
   ```

4. **`DeclareLaunchArgument` for mode** (lines 242–245): Change description to:
   ```python
               description="Operating mode: passive | stand | standup | position_control",
   ```

- [ ] **Step 2: Verify the file parses without errors**

```bash
python3 -c "import importlib.util, os; s=importlib.util.spec_from_file_location('x','src/legged_control/launch/robot.launch.py'); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); print('OK')"
```

Expected: `OK`

- [ ] **Step 3: Run existing launch unit tests to confirm nothing is broken**

```bash
cd src/legged_control && python3 -m pytest tests/test_robot_launch.py -v
```

Expected: all tests PASS (the tests only cover `_leg_group` and `_parse_legs`, neither of which is touched).

---

### Task 3: Remove `policy:` section from `robot.yaml`

**Files:**
- Modify: `src/legged_control/config/robot.yaml`

- [ ] **Step 1: Remove the `policy:` block (lines 190–196)**

Delete these lines:
```yaml
policy:
  # Path to TorchScript model file (.pt). Node will not start if empty — set before launching.
  model_path: '/home/grayerd/Desktop/Projects/rc/legged_ws/models/policy.pt'
  # Observation normalization (mean and std for each of the 45 dims).
  # Leave empty to use no normalization (raw values passed to model).
  obs_mean: []
  obs_std:  []
```

- [ ] **Step 2: Remove `policy_hz` and `action_scale` from the `control:` section (lines 134, 137–148)**

Delete these lines from the `control:` block:
```yaml
  policy_hz: 50.0
  # Policy action is interpreted as delta from default_q, then scaled per joint.
  # target_q[i] = default_q[i] + action_scale[leg_group][joint_type] * action[i]
  # leg_group: 'front' (FR/FL) or 'rear' (RR/RL); joint_type: 'hip', 'thigh', 'calf'
  action_scale:
    front:
      hip:   0.025
      thigh: 0.035
      calf:  0.02
    rear:
      hip:   0.03
      thigh: 0.045
      calf:  0.025
```

- [ ] **Step 3: Update the header comment for `direction`/`zero_offset` (line 13)**

Change:
```yaml
# direction / zero_offset: URDF alignment (used by policy_node, NOT by motor_bus_node).
```
To:
```yaml
# direction / zero_offset: URDF alignment (used by kinematics/gait_node and simulation bridges, NOT by motor_bus_node).
```

- [ ] **Step 4: Verify the YAML still parses**

```bash
python3 -c "import yaml; yaml.safe_load(open('src/legged_control/config/robot.yaml')); print('OK')"
```

Expected: `OK`

---

### Task 4: Remove policy entry points from `setup.py`

**Files:**
- Modify: `src/legged_control/setup.py`

- [ ] **Step 1: Remove the two policy entry point lines (lines 40–41)**

Delete:
```python
            "policy_node          = legged_control.policy_node:main",
            "policy_monitor_node  = legged_control.policy_monitor_node:main",
```

- [ ] **Step 2: Verify setup.py syntax**

```bash
python3 -c "import ast; ast.parse(open('src/legged_control/setup.py').read()); print('OK')"
```

Expected: `OK`

---

### Task 5: Update `CLAUDE.md` — remove policy documentation

**Files:**
- Modify: `CLAUDE.md` (workspace root)

- [ ] **Step 1: Update launch parameters table**

In the "Launch parameters" table, change the `mode` row `Options` cell from:
```
`passive`, `stand`, `standup`, `policy`
```
to:
```
`passive`, `stand`, `standup`, `position_control`
```
(Remove `policy` entry.)

- [ ] **Step 2: Remove the `policy` mode description block**

Delete the entire `**\`policy\`**` paragraph (the one describing three-phase startup, 50 Hz inference loop, TorchScript model, IMU/odometry requirements, `/cmd_vel`, `teleop_node`, `policy_monitor_node`).

- [ ] **Step 3: Remove `policy_node.py` and `policy_monitor_node.py` from the Architecture section**

Delete these two bullet points from the `legged_control` architecture list:
- `policy_node.py — 50 Hz inference loop; ...`
- `policy_monitor_node.py — read-only terminal dashboard for \`policy\` mode; ...`

- [ ] **Step 4: Remove policy-only topics from the Key Topics table**

Delete the rows for `/odin1/imu`, `/odin1/odometry` only if they are described as policy-only. Keep `/cmd_vel` (also used by `gait_node` via `teleop_node`). Keep `/joint_states_aggregated` (used by `gait_node`).

- [ ] **Step 5: Update the `robot.yaml` config example in CLAUDE.md**

In the YAML snippet under "Configuration (`config/robot.yaml`)", remove:
- `policy_hz: 50.0` from the `control:` block
- The entire `action_scale:` sub-block from `control:`
- The entire `policy:` top-level block (`model_path`, `obs_mean`, `obs_std`)

- [ ] **Step 6: Update the `launch/robot.launch.py` description line** in the Architecture section

Change the description that mentions `policy` mode to only list: `passive`, `stand`, `standup`, `position_control`.

---

### Task 6: Update `README.md` — remove policy section

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Delete the entire "第三步：策略行走（policy 模式）" section**

Delete from `## 第三步：策略行走（policy 模式）` (line 77) through the LiDAR timeout protection paragraph ending at `电机保持上一帧位置。` (line 142), inclusive.

- [ ] **Step 2: Remove the `mode:=policy` note at the bottom**

Delete line 158:
```
注意：`mode:=policy` 固定使用 12 关节输入输出契约，因此只能配合 `legs:=all` 启动。
```

---

### Task 7: Commit

- [ ] **Step 1: Stage and commit**

```bash
git add \
  src/legged_control/legged_control/policy_node.py \
  src/legged_control/legged_control/policy_monitor_node.py \
  src/legged_control/tests/test_policy_node.py \
  src/legged_control/tests/test_policy_monitor_node.py \
  src/legged_control/launch/robot.launch.py \
  src/legged_control/config/robot.yaml \
  src/legged_control/setup.py \
  CLAUDE.md \
  README.md
git commit -m "refactor: remove policy runtime from position-control branch"
```

- [ ] **Step 2: Verify clean working tree**

```bash
git status
```

Expected: `nothing to commit, working tree clean`
