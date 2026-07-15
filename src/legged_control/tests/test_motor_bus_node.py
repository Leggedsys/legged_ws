# src/legged_control/tests/test_motor_bus_node.py
from legged_control.real.motor_bus_node import (
    _BACKOFF_ENTER,
    _BACKOFF_EXIT,
    _BACKOFF_INTERVAL,
    _ack_summary,
    _backoff_next,
    _ema_update,
    _filter_joints,
    _health_summary,
    _ns_from_joint_name,
    _should_poll,
)


def test_ns_hip():
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_calf():
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_thigh():
    assert _ns_from_joint_name('RR_thigh') == 'rr/thigh'


def test_filter_joints_subset():
    joints = [
        {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0},
        {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8},
        {'name': 'RR_hip',   'motor_id': 6, 'default_q': 0.0},
    ]
    result = _filter_joints(joints, ['FR_hip', 'FR_thigh'])
    assert len(result) == 2
    assert result[0]['name'] == 'FR_hip'
    assert result[1]['name'] == 'FR_thigh'


def test_filter_joints_empty_names():
    joints = [{'name': 'FR_hip', 'motor_id': 0, 'default_q': 0.0}]
    assert _filter_joints(joints, []) == []


def test_filter_joints_unknown_name():
    joints = [{'name': 'FR_hip', 'motor_id': 0, 'default_q': 0.0}]
    assert _filter_joints(joints, ['XX_hip']) == []


def test_filter_joints_preserves_yaml_order():
    joints = [
        {'name': 'FR_hip',   'motor_id': 0, 'default_q': 0.0},
        {'name': 'FR_thigh', 'motor_id': 1, 'default_q': 0.8},
        {'name': 'FR_calf',  'motor_id': 2, 'default_q': -1.5},
    ]
    result = _filter_joints(joints, ['FR_calf', 'FR_hip'])
    assert [j['name'] for j in result] == ['FR_hip', 'FR_calf']


import time


_GAINS_TIMEOUT = 0.2  # must match motor_bus_node


def _gains_are_fresh(stamp: float | None, now: float) -> bool:
    """Mirrors the timeout logic in motor_bus_node._tick."""
    if stamp is None:
        return False
    return (now - stamp) < _GAINS_TIMEOUT


def test_gains_fresh_within_timeout():
    stamp = time.monotonic()
    assert _gains_are_fresh(stamp, stamp + 0.1) is True


def test_gains_stale_after_timeout():
    stamp = time.monotonic()
    assert _gains_are_fresh(stamp, stamp + 0.3) is False


def test_gains_stale_when_never_received():
    assert _gains_are_fresh(None, time.monotonic()) is False


def test_health_summary_percentages_and_worst():
    ok = {'FR_hip': 94, 'FL_hip': 15}
    attempts = {'FR_hip': 100, 'FL_hip': 100}
    line, worst = _health_summary(ok, attempts, ticks=1000, elapsed=10.0)
    assert 'FR_hip 94%' in line
    assert 'FL_hip 15%' in line
    assert 'loop 100Hz' in line
    assert worst == 15


def test_health_summary_no_attempts_counts_as_healthy():
    line, worst = _health_summary({}, {'FR_hip': 0}, ticks=0, elapsed=10.0)
    assert 'FR_hip 100%' in line
    assert worst == 100


def test_health_summary_zero_elapsed_no_crash():
    line, worst = _health_summary({'a': 1}, {'a': 2}, ticks=5, elapsed=0.0)
    assert 'a 50%' in line
    assert worst == 50


def test_health_summary_marks_backoff_motors():
    line, _ = _health_summary(
        {'FR_hip': 90, 'FL_hip': 30}, {'FR_hip': 100, 'FL_hip': 100},
        ticks=100, elapsed=10.0, backoff=frozenset({'FL_hip'}),
    )
    assert 'FL_hip* 30%' in line
    assert 'FR_hip 90%' in line  # healthy motor has no star


def test_ema_converges_to_reply_rate():
    ema = 1.0
    for _ in range(500):        # all failures → EMA decays toward 0
        ema = _ema_update(ema, False)
    assert ema < 0.01
    for _ in range(500):        # all successes → EMA recovers toward 1
        ema = _ema_update(ema, True)
    assert ema > 0.99


def test_backoff_hysteresis():
    # healthy stays healthy above the enter threshold
    assert _backoff_next(False, _BACKOFF_ENTER + 0.05) is False
    # drops below enter threshold → backoff
    assert _backoff_next(False, _BACKOFF_ENTER - 0.05) is True
    # inside the hysteresis band a backoff motor stays in backoff...
    assert _backoff_next(True, (_BACKOFF_ENTER + _BACKOFF_EXIT) / 2) is True
    # ...and a healthy motor stays healthy — no flapping
    assert _backoff_next(False, (_BACKOFF_ENTER + _BACKOFF_EXIT) / 2) is False
    # recovers only above the exit threshold
    assert _backoff_next(True, _BACKOFF_EXIT + 0.05) is False


def test_should_poll_healthy_every_tick():
    assert all(_should_poll(t, False, phase=2) for t in range(10))


def test_ack_summary_reports_silent_motors():
    line, silent = _ack_summary({'FR_hip': 38, 'FL_hip': 0, 'FL_calf': 3})
    assert 'FR_hip 38ack' in line
    assert 'FL_hip 0ack' in line
    assert silent == ['FL_hip']


def test_ack_summary_all_acked():
    _, silent = _ack_summary({'FR_hip': 40, 'FR_thigh': 39})
    assert silent == []


def test_should_poll_backoff_every_nth_staggered():
    for phase in range(_BACKOFF_INTERVAL + 2):  # phases beyond N wrap around
        polled = [t for t in range(4 * _BACKOFF_INTERVAL)
                  if _should_poll(t, True, phase)]
        assert len(polled) == 4
        assert all(t % _BACKOFF_INTERVAL == phase % _BACKOFF_INTERVAL
                   for t in polled)
