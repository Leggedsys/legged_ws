# Dual Serial Port Support

**Date:** 2026-04-12
**Status:** Approved

## Problem

All 12 motor nodes currently share a single serial port. The hardware now uses two separate serial buses:

- Front bus (`/dev/ttyUSB0`): FR and FL legs (6 motors)
- Rear bus (`/dev/ttyUSB1`): RR and RL legs (6 motors)

## Solution

Replace the single `serial_port` field with two fields in `robot.yaml`, and update the launch file to route each motor node to the correct port based on its leg group.

## Changes

### `config/robot.yaml`

Remove `serial_port`, add:

```yaml
control:
  serial_port_front: /dev/ttyUSB0   # FR, FL
  serial_port_rear:  /dev/ttyUSB1   # RR, RL
```

### `launch/robot.launch.py`

**Launch arguments:** Replace `serial_port` with `serial_port_front` and `serial_port_rear`, both defaulting to a sentinel that reads from yaml.

**Leg group helper:**

```python
def _leg_group(joint_name: str) -> str:
    """'FR_hip' → 'front',  'RR_hip' → 'rear'"""
    leg = joint_name.split('_')[0]
    return 'front' if leg in ('FR', 'FL') else 'rear'
```

**`_motor_nodes` signature change:**

```python
def _motor_nodes(joints, port_map: dict, motor_hz, kp, kd):
    # port_map = {'front': '/dev/ttyUSB0', 'rear': '/dev/ttyUSB1'}
    # each joint picks port_map[_leg_group(j['name'])]
```

**`_launch_setup`:** reads both ports from yaml (or launch arg overrides), builds `port_map`, passes to `_motor_nodes`. Passive and stand modes are unchanged in logic.

## Error Handling

- Missing yaml keys raise `KeyError` at launch time — no extra validation needed.
- The existing `legs` partial-selection arg still works: only activated legs create motor nodes; the other serial port is never opened.

## Removed

- `serial_port` launch argument (breaking change — any existing scripts using `serial_port:=...` must switch to `serial_port_front:=...` / `serial_port_rear:=...`).
