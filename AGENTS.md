# AGENTS.md

Agent guide for `/home/grayerd/Desktop/Projects/rc/legged_ws`.

This repository is a ROS2 Humble workspace for a legged robot.
Current packages:
- `src/legged_control` - Python locomotion and calibration stack
- `src/unitree_actuator_sdk` - Python ROS2 wrapper around the Unitree motor SDK
- `src/odin_ros_driver` - C++17 ROS1/ROS2 driver for the Odin1 LiDAR

This file is for coding agents. Prefer small, surgical changes that match the surrounding style.

## Instruction Sources

- Repository guidance exists in `CLAUDE.md`; this file incorporates the important parts.
- No existing `AGENTS.md` was present at the workspace root.
- No Cursor rules were found in `.cursor/rules/` or `.cursorrules`.
- No Copilot instructions were found in `.github/copilot-instructions.md`.

## Working Assumptions

- Workspace root is `/home/grayerd/Desktop/Projects/rc/legged_ws`.
- ROS distro is expected to be Humble unless a task explicitly targets ROS1.
- Many run commands touch real hardware. Do not run motor or LiDAR nodes unless the user wants hardware interaction.
- Prefer verifying pure-Python tests first; they do not require devices.

## Environment Bootstrap

Run these from the workspace root unless noted otherwise:

```bash
source /opt/ros/humble/setup.bash
```

After building the workspace, also source:

```bash
source install/setup.bash
```

If you are working only inside `src/legged_control`, pure Python tests can also be run from that package directory without rebuilding.

## Build Commands

Build the full workspace:

```bash
source /opt/ros/humble/setup.bash && colcon build
```

Build one package:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select legged_control
source /opt/ros/humble/setup.bash && colcon build --packages-select unitree_actuator_sdk
source /opt/ros/humble/setup.bash && colcon build --packages-select odin_ros_driver
```

Build with fresh compile commands for C++ work:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select odin_ros_driver --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Use the vendor build helper for the LiDAR package when you specifically want its scripted clean build flow:

```bash
source /opt/ros/humble/setup.bash && ./src/odin_ros_driver/script/build_ros2.sh
./src/odin_ros_driver/script/build_ros2.sh -c
```

## Test Commands

Only `legged_control` currently has checked-in automated tests.
They are pytest tests under `src/legged_control/tests/calibration/`.

Run the full Python test suite for that package from the package directory:

```bash
python3 -m pytest tests/calibration -v
```

Run one test file:

```bash
python3 -m pytest tests/calibration/test_config_io.py -v
python3 -m pytest tests/calibration/test_oscillation.py -v
python3 -m pytest tests/calibration/test_phase1_logic.py -v
python3 -m pytest tests/calibration/test_phase3_logic.py -v
```

Run one specific test case:

```bash
python3 -m pytest tests/calibration/test_phase3_logic.py::test_suggest_kd_increase_on_overshoot -v
```

Run tests through colcon after building and sourcing the workspace:

```bash
source /opt/ros/humble/setup.bash && colcon test --packages-select legged_control
source /opt/ros/humble/setup.bash && colcon test-result --verbose
```

There are no repository C++ unit tests or ROS integration tests checked in for `odin_ros_driver` or `unitree_actuator_sdk`.
If you touch those packages, at minimum build them successfully.

## Lint And Validation Commands

There is no canonical repo-wide lint configuration checked in.
No `ruff`, `black`, `flake8`, `clang-format`, or `ament_lint` config files were found.

Use lightweight validation instead of inventing a new lint stack:

```bash
python3 -m compileall src/legged_control/legged_control src/unitree_actuator_sdk/unitree_motor_ros2
python3 -m pytest src/legged_control/tests/calibration -v
source /opt/ros/humble/setup.bash && colcon build --packages-select legged_control
source /opt/ros/humble/setup.bash && colcon build --packages-select unitree_actuator_sdk
source /opt/ros/humble/setup.bash && colcon build --packages-select odin_ros_driver
```

For C++ changes, treat a clean `colcon build --packages-select odin_ros_driver` as the primary validation gate.

## Common Run Commands

LiDAR full launch:

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

Single motor node:

```bash
ros2 run unitree_actuator_sdk go_m8010_6_node
```

Single motor node with parameters:

```bash
ros2 run unitree_actuator_sdk go_m8010_6_node --ros-args -p serial_port:=/dev/ttyUSB0 -p motor_id:=0 -p loop_hz:=1000.0
```

Calibration script:

```bash
python3 src/legged_control/scripts/calibrate.py
```

## Code Style Overview

Follow existing local style before applying general preferences.
This repo mixes three styles: ROS2 Python nodes, Python calibration utilities/tests, and legacy C++ driver code.

### Python Style

- Use 4-space indentation.
- Keep module-level docstrings for nodes, scripts, and small utility modules.
- Prefer type hints on new or modified functions; existing Python code already uses `-> None`, `list[str]`, `dict[str, int]`, and union syntax.
- Use built-in generics (`list[str]`, `dict[str, float]`) instead of `typing.List`/`typing.Dict`.
- Use `from __future__ import annotations` when a file already uses it or when it simplifies forward references and modern type unions.
- Prefer `snake_case` for modules, functions, variables, and methods.
- Use `PascalCase` for classes like ROS nodes and helper objects.
- Use `UPPER_SNAKE_CASE` for constants such as `KP_MAX` and `_STEP_SIZE`.
- Keep ROS topic names and parameter names as string literals that match existing conventions, usually absolute topics like `'/joint_commands'`.
- Prefer small helper methods over deeply nested logic.

### Python Imports

- Group imports in this order when touching a file: standard library, third-party/ROS, local package imports.
- Separate groups with a blank line.
- Prefer one import per line.
- Some existing tests use compact imports on one line; do not spread mechanical formatting churn unless you are already editing that block.

### Python Error Handling

- Fail fast for invalid startup configuration, unsupported architecture, or impossible parameter values.
- Raise exceptions for construction-time failures that should stop the node, for example invalid `loop_hz` or unsupported CPU architecture.
- For recoverable runtime issues, log clearly and continue with a safe fallback when the current code already does so.
- When translating an exception, preserve context with `raise ... from exc`.
- Use `try`/`finally` around ROS node lifetime so `destroy_node()` and `rclpy.shutdown()` always happen.
- Avoid broad `except Exception` unless you are converting the failure into a user-facing fallback or log message.

### Python Files And Config

- `legged_control` reads `config/robot.yaml` via package share paths; preserve that pattern.
- Calibration code preserves YAML comments through `ConfigIO`; use that helper instead of rewriting `robot.yaml` manually.
- Keep testable pure logic separated from interactive or ROS-dependent code when possible; `phase1.py` and `phase3.py` are the model to follow.

### C++ Style

- Target C++17.
- Match the existing brace and indentation style in the file you are editing; `odin_ros_driver` is not fully uniform.
- Keep includes near the top, with project headers first and library/system headers after, matching surrounding style.
- Preserve the ROS1/ROS2 compatibility structure guarded by `#ifdef ROS2` where it already exists.
- Prefer `const` correctness and pass heavy objects by const reference where practical.
- Use descriptive names; the codebase favors explicit names over terse abbreviations in new helper code.
- Reuse existing logging paths (`RCLCPP_*`, `ROS_*`, or existing `std::cerr` diagnostics) instead of introducing a new logging pattern inside a file.

### Naming And Architecture Conventions

- Python node classes inherit from `rclpy.node.Node` and expose a `main()` entry point.
- Console scripts are registered in `setup.py`; if you add a node, update the package entry points.
- Shared config loaders usually resolve package paths with `get_package_share_directory()` plus `os.path.join(...)`.
- Hardware-facing state is usually cached on `self._...` fields.
- Internal helper functions that are intentionally private often use a leading underscore.
- Pure constants at file scope are uppercase, even when logically private.

### Comments And Documentation

- Keep comments for non-obvious behavior, hardware assumptions, or safety constraints.
- Do not add redundant comments that just restate the code.
- Preserve useful module docstrings and usage blocks in scripts.
- If a file already contains explanatory section dividers, follow that structure instead of replacing it.

## Practical Agent Advice

- Before editing, check whether the target package is Python (`ament_python`) or C++ (`ament_cmake`).
- Avoid broad reformatting; mixed legacy formatting is normal here.
- Be cautious with commands that access `/dev/ttyUSB*` or real sensors.
- When changing `unitree_actuator_sdk`, remember its Python extension depends on the active ROS Python runtime; rebuilding is often required.
- When changing `odin_ros_driver`, verify architecture-sensitive library selection and avoid breaking ROS1 compatibility unless the task is explicitly ROS2-only.
- When changing calibration logic, prefer adding or updating pytest coverage in `src/legged_control/tests/calibration/`.

## Minimum Verification Expectations

- Python logic change in `legged_control`: run the relevant pytest file, or the single test case if the change is very focused.
- Python node change in `unitree_actuator_sdk`: run `python3 -m compileall` and build the package.
- C++ change in `odin_ros_driver`: rebuild `odin_ros_driver` and note that hardware runtime verification may still be needed.

If you are unsure, choose the smallest safe change, keep hardware assumptions explicit, and verify with the narrowest relevant command.
