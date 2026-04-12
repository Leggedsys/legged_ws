# Motor Bus Node Design

**Date:** 2026-04-12
**Status:** Approved

## Problem

Multiple `go_m8010_6_node` instances share the same RS485 serial port by each opening an independent file descriptor. At 1000 Hz, their concurrent `sendRecv` calls collide on the wire: transmitted command frames overlap, and response bytes are stolen by whichever process calls `recv` first. This causes:

- Garbage position values (CRC-failed frames decoded as floats)
- Data cross-talk (motor A's position published under motor B's joint_states topic)

The Unitree SDK provides a batch `sendRecv(vector<MotorCmd>, vector<MotorData>)` API designed for a single master to cycle all motors on a bus sequentially. The fix is to ensure exactly one process owns each serial port.

## Solution

Replace the 12 individual `go_m8010_6_node` instances with 2 `motor_bus_node` instances — one per serial port. Each bus node owns its port exclusively and cycles through all assigned joints sequentially in every tick.

## Architecture

```
launch → motor_bus_front  (serial_port_front, FR/FL joints)
       → motor_bus_rear   (serial_port_rear,  RR/RL joints)

Each bus node (1000 Hz tick):
  for each joint:
    sendRecv(cmd, data)        # sequential, no collision
    validate data.correct + data.motor_id
    publish /{ns}/joint_states  # only on valid response

stand_node → AsyncParametersClient("/motor_bus_front")
           → AsyncParametersClient("/motor_bus_rear")
           → ros2 param set kp/kd  # 2 clients instead of 12
```

## New File

### `src/legged_control/legged_control/motor_bus_node.py`

**ROS2 parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `serial_port` | string | Serial device path |
| `joint_names` | string[] | Joints managed by this bus node |
| `kp` | float | Position gain (runtime-tunable) |
| `kd` | float | Velocity damping (runtime-tunable) |

**Initialisation:**
1. Load `robot.yaml`, filter joints matching `joint_names` parameter
2. Build per-joint `MotorCmd` and `MotorData` objects; set `motorType`, `mode`, `id`, `target_q` from yaml `default_q`
3. Create one `JointState` publisher per joint at `/{ns}/joint_states`
4. Subscribe to `/joint_commands`; on receipt, update `_targets[joint_name]` for any matching joint
5. Register `add_on_set_parameters_callback` for runtime kp/kd updates
6. Start 1000 Hz timer

**Control loop (1000 Hz tick):**
```python
for cmd, data, pub, name in zip(self._cmds, self._datas, self._pubs, self._names):
    cmd.kp = self.get_parameter('kp').value
    cmd.kd = self.get_parameter('kd').value
    cmd.q  = self._targets[name]
    self._serial.sendRecv(cmd, data)
    if data.correct and data.motor_id == cmd.id:
        pub.publish(make_joint_state(name, data))
```

**Node names:** `/motor_bus_front` (FR/FL), `/motor_bus_rear` (RR/RL)

**Register in `setup.py`:**
```python
'motor_bus_node = legged_control.motor_bus_node:main',
```

## Modified Files

### `src/legged_control/legged_control/stand_node.py`

Replace 12-client `_gain_clients` with 2 bus-node clients:

```python
self._gain_clients = [
    AsyncParametersClient(self, '/motor_bus_front'),
    AsyncParametersClient(self, '/motor_bus_rear'),
]
```

Remove the `_motor_node_name()` helper (no longer needed).

### `src/legged_control/launch/robot.launch.py`

Replace `_motor_nodes()` with `_bus_nodes()`:

- Split the active `joints` list into `front_joints` (FR/FL) and `rear_joints` (RR/RL) using `_leg_group()`
- Start `motor_bus_front` node only if `front_joints` is non-empty
- Start `motor_bus_rear` node only if `rear_joints` is non-empty
- Pass `kp=0.0, kd=0.0` in passive mode; `kp/kd` from yaml in stand mode

```python
def _bus_nodes(front_joints, rear_joints, sp_front, sp_rear, motor_hz, kp, kd):
    nodes = []
    for name, joints, port in [
        ('motor_bus_front', front_joints, sp_front),
        ('motor_bus_rear',  rear_joints,  sp_rear),
    ]:
        if not joints:
            continue
        nodes.append(Node(
            package='legged_control',
            executable='motor_bus_node',
            name=name,
            parameters=[{
                'serial_port':  port,
                'joint_names':  [j['name'] for j in joints],
                'kp': kp,
                'kd': kd,
                'loop_hz': motor_hz,
            }],
            output='log',
        ))
    return nodes
```

## Unchanged

- `robot.yaml` — no config changes needed
- `passive_monitor_node.py` — subscribes to same `/{ns}/joint_states` topics, unaffected
- `go_m8010_6_node.py` — kept in `unitree_actuator_sdk` package, not used in launch

## legs Partial Selection

The `legs` launch arg continues to work. The launch file filters `joints` by active legs before splitting into front/rear lists. If `legs:=FR`, `rear_joints` is empty and `motor_bus_rear` is not started.

## Testing

- Unit test `motor_bus_node` helpers (joint filtering, target update logic) without ROS2
- Integration: launch with `legs:=FR`, verify `/fr/hip/joint_states` etc. publish at expected rate with no data cross-talk
