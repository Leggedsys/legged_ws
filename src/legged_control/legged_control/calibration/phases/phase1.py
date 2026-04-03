"""Phase 1: suspended motor ID mapping by manual joint wiggling."""

from __future__ import annotations
import time

from legged_control.calibration import ui


DETECTION_ORDER = [
    'FR_calf', 'FL_calf', 'RR_calf', 'RL_calf',
    'FR_thigh', 'FL_thigh', 'RR_thigh', 'RL_thigh',
    'FR_hip', 'FL_hip', 'RR_hip', 'RL_hip',
]
_WIGGLE_THRESHOLD = 0.3
_MAX_RETRIES = 3


def run(joints_cfg: list[dict], ros_client, motor_manager,
        pause_ctrl) -> dict[str, int]:
    """Guide user through motor ID mapping. Returns {joint_name: motor_id}."""
    ui.phase_banner(1, 'Motor ID Mapping (suspended)')
    ui.info('All motors will be set to zero-torque so you can wiggle joints freely.')
    input('Confirm robot is suspended off the ground, then press Enter.')

    motor_manager.zero_torque()
    time.sleep(0.5)

    baseline = ros_client.get_joint_positions()
    available_ids = set(range(12))
    mapping: dict[str, int] = {}

    for joint_name in DETECTION_ORDER:
        pause_ctrl.check()
        _detect_one_joint(joint_name, baseline, ros_client,
                          available_ids, mapping, pause_ctrl)
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
        ui.info(f'  Detected: [{detected_name}] moved Δ={delta:.3f} rad')

        if detected_name in mapping.values():
            existing = [k for k, v in mapping.items() if v == detected_name]
            ans = ui.prompt(
                f'  motor corresponding to [{detected_name}] already assigned to {existing}. '
                f'Overwrite or re-wiggle? [o/r]:')
            if ans.lower() == 'r':
                continue

        if ui.confirm(f'  Is this [{joint_name}]?'):
            motor_id = _next_available_id(available_ids)
            if motor_id is None:
                ui.warn('No available motor IDs left; mark as pending.')
                return
            mapping[joint_name] = motor_id
            available_ids.discard(motor_id)
            ui.success(f'  {joint_name} → motor_id={motor_id}')
            return

    ui.warn(f'Could not confirm {joint_name} after {_MAX_RETRIES} attempts. Mark as pending.')


def _wait_for_wiggle(baseline: dict[str, float], ros_client,
                     pause_ctrl, poll_hz: float = 20.0,
                     timeout: float = 30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        pause_ctrl.check()
        current = ros_client.get_joint_positions()
        result = detect_moved_joint(baseline, current, _WIGGLE_THRESHOLD)
        if result is not None:
            return result
        time.sleep(1.0 / poll_hz)
    return None


def _next_available_id(available_ids: set[int]) -> int | None:
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
    ui.info('\nFinal motor ID mapping:')
    for name, mid in sorted(mapping.items()):
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
