# Simulation Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor `real2sim/` → `processing/`, create `sim/` layer, wire up Gazebo simulation stack that produces identical obs topics to real hardware, enabling `policy_node.py` to run unchanged in both environments.

**Architecture:** Three data-source-agnostic layers: `real/` (hardware drivers), `sim/` (Gazebo bridges), shared `processing/` (obs computation). `processing/` nodes consume only standard ROS2 topics — they don't know or care where data comes from.

**Tech Stack:** ROS2 Humble, ament_python, Gazebo, ros2_control

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Rename dir | `real2sim/` → `processing/` | Shared obs layer |
| Create dir | `sim/` | Simulation data source |
| Move | `gazebo_control_bridge.py` → `sim/` | Gazebo joint bridge |
| Move | `policy_node_sim.py` → `sim/` | Temporary sim policy |
| Create | `sim/__init__.py` | Package marker |
| Modify | `tests/test_teleop_node.py:7` | Update import path |
| Modify | `tests/test_height_scan.py:6` | Update import path |
| Modify | `setup.py:41-51` | Update entry_points |
| Create | `launch/gazebo_sim.launch.py` | Full sim stack entry |
| Modify | `dog_urdf/urdf/dog_urdf.urdf` | Add depth camera sensor |
| Create | `scripts/install_deps.sh` | Deployment helper |

---

### Task 1: Create processing/ and sim/ directories

**Files:**
- Create: `src/legged_control/legged_control/processing/__init__.py`
- Create: `src/legged_control/legged_control/sim/__init__.py`
- Rename: `src/legged_control/legged_control/real2sim/` → `src/legged_control/legged_control/processing/` (move all 4 files)

- [ ] **Step 1: Rename real2sim to processing**

```bash
cd src/legged_control/legged_control
git mv real2sim processing
```

- [ ] **Step 2: Create sim directory**

```bash
mkdir -p src/legged_control/legged_control/sim
touch src/legged_control/legged_control/sim/__init__.py
```

- [ ] **Step 3: Move gazebo_control_bridge and policy_node_sim into sim**

```bash
cd src/legged_control/legged_control
git mv gazebo_control_bridge.py sim/
git mv policy_node_sim.py sim/
```

- [ ] **Step 4: Verify file structure**

```bash
ls src/legged_control/legged_control/processing/  # 4 py files + __init__
ls src/legged_control/legged_control/sim/          # 2 py files + __init__
ls src/legged_control/legged_control/real/         # unchanged
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor: rename real2sim to processing, create sim layer"
```

---

### Task 2: Update setup.py entry_points

**Files:**
- Modify: `src/legged_control/setup.py:41-53`

- [ ] **Step 1: Update entry_points paths**

Replace the `console_scripts` dict in `setup.py`:

```python
entry_points={
    "console_scripts": [
        "obs_monitor_node      = legged_control.test.obs_monitor_node:main",
        "vel_viz_node          = legged_control.test.vel_viz_node:main",
        "motor_bus_node        = legged_control.real.motor_bus_node:main",
        "joint_aggregator      = legged_control.real.joint_aggregator:main",
        "teleop_node           = legged_control.processing.teleop_node:main",
        "urdf_joint_state_bridge = legged_control.processing.urdf_joint_state_bridge:main",
        "gazebo_control_bridge = legged_control.sim.gazebo_control_bridge:main",
        "state_estimator_node  = legged_control.processing.state_estimator_node:main",
        "height_scan_node      = legged_control.processing.height_scan_node:main",
        "policy_node           = legged_control.policy_node:main",
        "policy_node_sim       = legged_control.sim.policy_node_sim:main",
    ],
},
```

- [ ] **Step 2: Rebuild and verify no import errors**

```bash
bash -c 'source /opt/ros/humble/setup.bash && colcon build --packages-select legged_control' 2>&1 | tail -3
```

Expected: `Summary: 1 package finished`

- [ ] **Step 3: Verify all entry points still load**

```bash
bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && python3 << '\''EOF'\''
from legged_control.processing.state_estimator_node import StateEstimatorNode
from legged_control.processing.height_scan_node import HeightScanNode
from legged_control.processing.teleop_node import TeleopNode
from legged_control.processing.urdf_joint_state_bridge import URDFJointStateBridgeNode
from legged_control.sim.gazebo_control_bridge import GazeboControlBridgeNode
from legged_control.sim.policy_node_sim import main as sim_policy_main
print("ALL IMPORTS OK")
EOF'
```

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/setup.py
git commit -m "refactor: update setup.py entry_points for processing/sim split"
```

---

### Task 3: Update test imports

**Files:**
- Modify: `src/legged_control/tests/test_teleop_node.py:7`
- Modify: `src/legged_control/tests/test_height_scan.py:6`

- [ ] **Step 1: Fix test_teleop_node.py import**

```python
# Line 7: change
from legged_control.real2sim.teleop_node import (
# to
from legged_control.processing.teleop_node import (
```

- [ ] **Step 2: Fix test_height_scan.py import**

```python
# Line 6: change
    from legged_control.real2sim.height_scan_node import _build_height_scan as _f
# to
    from legged_control.processing.height_scan_node import _build_height_scan as _f
```

- [ ] **Step 3: Run all tests**

```bash
bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && for t in test_motor_bus_node test_joint_aggregator test_kinematics test_height_scan test_state_estimator test_teleop_node; do python3 -m pytest src/legged_control/tests/${t}.py -q; done' 2>&1
```

Expected: all 65 tests pass

- [ ] **Step 4: Commit**

```bash
git add src/legged_control/tests/
git commit -m "test: update imports for processing layer rename"
```

---

### Task 4: Update launch files for new entry_point names

**Files:**
- Modify: `src/legged_control/launch/real.launch.py` (line: state_estimator_node)
- Modify: `src/legged_control/launch/test.launch.py` (line: state_estimator_node, height_scan_node, teleop_node, urdf_joint_state_bridge, obs, vel_viz)
- Modify: `src/legged_control/launch/robot.launch.py` (line: state_estimator_node, height_scan_node, teleop_node)
- Modify: `src/legged_control/launch/gazebo_policy.launch.py` (line: state_estimator_node, height_scan_node, teleop_node, gazebo_control_bridge, policy_node_sim)

**Note:** The `executable` names in Node() actions are the entry_point names from setup.py. These did NOT change — only the module paths changed. So no launch file changes needed for existing nodes.

- [ ] **Step 1: Verify launch files still reference correct executables**

```bash
grep -rn "executable=" src/legged_control/launch/*.py | sort
```

All `executable` values (e.g., `motor_bus_node`, `state_estimator_node`) match setup.py entry points. No changes needed.

- [ ] **Step 2: Commit (empty — verification only)**

No commit needed. All executable names unchanged.

---

### Task 5: Add depth camera to URDF

**Files:**
- Modify: `install/dog_urdf/share/dog_urdf/urdf/dog_urdf.urdf` (add sensor block before closing `</robot>`)

- [ ] **Step 1: Add depth camera Gazebo sensor to base_link**

Insert before `</robot>` at end of URDF:

```xml
  <gazebo reference="base_link">
    <sensor type="depth" name="camera_depth_sensor">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <pose>0.15 0 0.12 0 0.5236 0</pose>
      <camera name="camera_depth_camera">
        <horizontal_fov>1.51844</horizontal_fov>
        <image>
          <width>848</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_depth_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>camera</namespace>
          <argument>image_raw:=depth/image_rect_raw</argument>
          <argument>camera_info:=depth/camera_info</argument>
        </ros>
        <camera_name>camera_depth</camera_name>
        <frame_name>camera_depth_optical_frame</frame_name>
        <hack_baseline>0.07</hack_baseline>
      </plugin>
    </sensor>
  </gazebo>
```

Also add a static transform from `base_link` to `camera_depth_optical_frame` in the URDF so TF works in Gazebo:

```xml
  <link name="camera_depth_optical_frame"/>
  <joint name="camera_depth_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_depth_optical_frame"/>
    <origin xyz="0.15 0 0.12" rpy="-1.5707963 0 -1.5707963"/>
  </joint>
```

- [ ] **Step 2: Verify URDF is valid XML**

```bash
python3 -c "import xml.etree.ElementTree as ET; ET.parse('install/dog_urdf/share/dog_urdf/urdf/dog_urdf.urdf'); print('OK')"
```

- [ ] **Step 3: Commit**

```bash
git add install/dog_urdf/share/dog_urdf/urdf/dog_urdf.urdf
git commit -m "feat(urdf): add depth camera sensor for Gazebo simulation"
```

---

### Task 6: Create gazebo_sim.launch.py

**Files:**
- Create: `src/legged_control/launch/gazebo_sim.launch.py`

- [ ] **Step 1: Write gazebo_sim.launch.py**

```python
"""gazebo_sim.launch.py — full simulation stack with policy.

Launches:
  gazebo_physics.launch.py    — Gazebo + robot model + ros2_control
  gazebo_control_bridge        — Gazebo joints ↔ /joint_states_aggregated
  imu_filter_madgwick          — /odin1/imu → /odin1/imu/filtered
  processing nodes             — state_estimator, height_scan, teleop
  policy_node                  — reads obs topics → /joint_commands
  test nodes                   — obs_monitor, vel_viz
  rviz2                        — visualization

Launch args:
  urdf_path         [auto]          Path to simulation URDF
  spawn_z           [0.50]          Robot spawn height
  model_path        []              Path to TorchScript .pt policy file
  rviz              [true]          Launch RViz2
  gui               [true]          Launch Gazebo GUI
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    _DOG_URDF_SHARE = get_package_share_directory("dog_urdf")
except Exception:
    _DOG_URDF_SHARE = ""


def _default_urdf_path():
    if _DOG_URDF_SHARE:
        return os.path.join(_DOG_URDF_SHARE, "urdf", "dog_urdf.urdf")
    return ""


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("legged_control")
    robot_cfg = os.path.join(share, "config", "robot_sim.yaml")
    model_path = LaunchConfiguration("model_path").perform(context)

    physics_launch = os.path.join(share, "launch", "gazebo_physics.launch.py")
    rviz_cfg = os.path.join(share, "config", "test.rviz")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physics_launch),
            launch_arguments={
                "urdf_path": LaunchConfiguration("urdf_path"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "gui": LaunchConfiguration("gui"),
            }.items(),
        ),
        # ── sim data source ────────────────────────────────────────────
        Node(
            package="legged_control",
            executable="gazebo_control_bridge",
            name="gazebo_control_bridge",
            output="screen",
        ),
        # ── IMU pipeline ───────────────────────────────────────────────
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            parameters=[{
                "use_mag": False,
                "publish_tf": False,
                "fixed_frame": "base_link",
                "world_frame": "enu",
            }],
            remappings=[
                ("imu/data_raw", "odin1/imu"),
                ("imu/data", "odin1/imu/filtered"),
            ],
            output="log",
        ),
        # ── processing layer (shared with real) ────────────────────────
        Node(
            package="legged_control",
            executable="joint_aggregator",
            name="joint_aggregator",
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="urdf_joint_state_bridge",
            name="urdf_joint_state_bridge",
            output="log",
        ),
        Node(
            package="legged_control",
            executable="state_estimator_node",
            name="state_estimator_node",
            parameters=[{"config_path": robot_cfg}],
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="height_scan_node",
            name="height_scan_node",
            output="log",
        ),
        # ── teleop ─────────────────────────────────────────────────────
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="log",
        ),
        Node(
            package="legged_control",
            executable="teleop_node",
            name="teleop_node",
            output="log",
        ),
        # ── policy ─────────────────────────────────────────────────────
        Node(
            package="legged_control",
            executable="policy_node",
            name="policy_node",
            parameters=[{
                "model_path": model_path,
                "config_path": robot_cfg,
            }],
            output="screen",
        ),
        # ── test / visualization ───────────────────────────────────────
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": open(_default_urdf_path()).read(),
            }],
            output="log",
        ),
        Node(
            package="legged_control",
            executable="obs_monitor_node",
            name="obs_monitor_node",
            output="screen",
        ),
        Node(
            package="legged_control",
            executable="vel_viz_node",
            name="vel_viz_node",
            output="log",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value=_default_urdf_path(),
        ),
        DeclareLaunchArgument("spawn_z", default_value="0.50"),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="Path to TorchScript .pt policy file",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
```

- [ ] **Step 2: Verify launch file parses**

```bash
bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch legged_control gazebo_sim.launch.py --show-args'
```

Expected: lists `urdf_path`, `spawn_z`, `model_path`, `rviz`, `gui`

- [ ] **Step 3: Commit**

```bash
git add src/legged_control/launch/gazebo_sim.launch.py
git commit -m "feat: add gazebo_sim.launch.py — full simulation stack"
```

---

### Task 7: Create deployment script

**Files:**
- Create: `scripts/install_deps.sh`

- [ ] **Step 1: Write install_deps.sh**

```bash
#!/bin/bash
set -e

echo "=== installing system dependencies ==="
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-imu-filter-madgwick \
    ros-humble-joy \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers

echo "=== setting up udev rules ==="
# Serial port access (motor bus)
sudo usermod -a -G dialout $USER

# Odin1 USB access
sudo tee /etc/udev/rules.d/99-odin-usb.rules << 'UEOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
UEOF
sudo udevadm control --reload && sudo udevadm trigger

echo "=== done. re-login or reboot for group changes ==="
```

- [ ] **Step 2: Make executable**

```bash
chmod +x scripts/install_deps.sh
```

- [ ] **Step 3: Commit**

```bash
git add scripts/install_deps.sh
git commit -m "feat: add deployment dependency install script"
```

---

### Task 8: Delete deprecated gazebo_policy.launch.py

**Files:**
- Delete: `src/legged_control/launch/gazebo_policy.launch.py`

- [ ] **Step 1: Delete old launch file**

```bash
git rm src/legged_control/launch/gazebo_policy.launch.py
```

- [ ] **Step 2: Commit**

```bash
git commit -m "chore: remove deprecated gazebo_policy.launch.py (replaced by gazebo_sim.launch.py)"
```

---

### Task 9: Final verification — build and test

- [ ] **Step 1: Full build**

```bash
bash -c 'source /opt/ros/humble/setup.bash && colcon build' 2>&1 | tail -5
```

Expected: all 4 packages finished

- [ ] **Step 2: Run all tests**

```bash
bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && for t in test_motor_bus_node test_joint_aggregator test_kinematics test_height_scan test_state_estimator test_teleop_node; do python3 -m pytest src/legged_control/tests/${t}.py -q 2>&1 | tail -1; done'
```

Expected: all 65 pass

- [ ] **Step 3: Verify import chain**

```bash
bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -c "
from legged_control.real.motor_bus_node import MotorBusNode
from legged_control.real.joint_aggregator import JointAggregatorNode
from legged_control.processing.state_estimator_node import StateEstimatorNode
from legged_control.processing.height_scan_node import HeightScanNode
from legged_control.processing.teleop_node import TeleopNode
from legged_control.processing.urdf_joint_state_bridge import URDFJointStateBridgeNode
from legged_control.sim.gazebo_control_bridge import GazeboControlBridgeNode
from legged_control.kinematics import leg_kinematic_velocity
from legged_control.test.obs_monitor_node import PassiveMonitorNode
from legged_control.test.vel_viz_node import VelVizNode
print(\"ALL 10 IMPORTS OK\")
"'
```

- [ ] **Step 4: Commit (if any final fixes needed, otherwise skip)**
