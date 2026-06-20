"""Tests for policy_node input-freshness / health guards (#6, #8)."""

from legged_control.policy_node import _inputs_usable, _is_fresh

_MAX = 0.1


def test_is_fresh_within_window():
    assert _is_fresh(0.05, _MAX)


def test_is_fresh_rejects_stale():
    assert not _is_fresh(0.2, _MAX)


def test_is_fresh_rejects_never_received():
    assert not _is_fresh(None, _MAX)


def test_inputs_usable_when_fresh_and_healthy():
    assert _inputs_usable(0.02, 0.02, 1.0, _MAX)


def test_inputs_blocked_by_stale_obs():
    assert not _inputs_usable(0.5, 0.02, 1.0, _MAX)


def test_inputs_blocked_by_stale_estimate():
    assert not _inputs_usable(0.02, 0.5, 1.0, _MAX)


def test_inputs_blocked_by_unhealthy_estimate():
    assert not _inputs_usable(0.02, 0.02, 0.0, _MAX)


def test_inputs_blocked_before_any_data():
    assert not _inputs_usable(None, None, 0.0, _MAX)
