# Remove Policy Code from dev/position-control Branch

**Date:** 2026-04-18  
**Branch:** dev/position-control  
**Scope:** Delete all RL policy runtime code; keep position control, simulation, and hardware alignment infrastructure.

## Goal

The `dev/position-control` branch focuses exclusively on analytical-IK position control and Gazebo/RViz simulation. Policy (TorchScript RL inference) belongs on a separate branch. This spec defines what to remove and what to keep.

## What Gets Removed

### Files to delete
- `src/legged_control/legged_control/policy_node.py` — 50 Hz TorchScript inference engine; the only file that imports `torch`
- `src/legged_control/legged_control/policy_monitor_node.py` — terminal dashboard for policy mode
- `src/legged_control/tests/test_policy_node.py`
- `src/legged_control/tests/test_policy_monitor_node.py`

### Edits
| File | Change |
|------|--------|
| `robot.launch.py` | Remove `policy` from allowed modes; delete the policy launch block (nodes: `joint_aggregator`, `joy_node`, `teleop_node`, `policy_node`, `policy_monitor_node` launched only in policy mode — keep `teleop_node`/`joint_aggregator` if referenced by `position_control` mode) |
| `robot.yaml` | Delete only the `policy:` section (`model_path`, `obs_mean`, `obs_std`); all joint parameters (`direction`, `zero_offset`, `gear_ratio`, limits) are kept |
| `setup.py` | Remove `policy_node` and `policy_monitor_node` entry points |
| `package.xml` | Remove any policy-exclusive dependencies (torch is a system dep not declared here — confirm nothing to remove) |
| `CLAUDE.md` | Remove `policy` mode description, `policy_node.py` architecture entry, policy topics, policy parameters |
| `README.md` | Remove policy mode usage section |

## What Is Kept

- All position-control nodes: `gait_node`, `kinematics`, `gazebo_control_bridge`, `fake_motor_bus_node`, `urdf_joint_state_bridge`
- All Gazebo/RViz launch files
- `joint_aggregator` — used by `gait_node` via `/joint_states_aggregated`
- `teleop_node` — used by `position_control` mode
- `standup_node` — independent `standup` mode, unrelated to policy
- All joint alignment parameters in `robot.yaml` (`direction`, `zero_offset`, `gear_ratio`) — shared by position control and simulation

## Constraints

- No code logic changes — this is pure deletion and reference cleanup.
- Single commit.
- After the change, `robot.launch.py` must still support: `passive`, `stand`, `standup`, `position_control`.
