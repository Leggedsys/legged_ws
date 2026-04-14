# src/legged_control/tests/test_joint_aggregator.py
"""Tests for joint_aggregator pure helper functions (no ROS2 required)."""

import pytest


def test_ns_from_joint_name_hip():
    from legged_control.joint_aggregator import _ns_from_joint_name

    assert _ns_from_joint_name("FR_hip") == "fr/hip"


def test_ns_from_joint_name_calf():
    from legged_control.joint_aggregator import _ns_from_joint_name

    assert _ns_from_joint_name("RL_calf") == "rl/calf"


def test_ns_from_joint_name_thigh():
    from legged_control.joint_aggregator import _ns_from_joint_name

    assert _ns_from_joint_name("FL_thigh") == "fl/thigh"


def test_freshness_status_reports_unseen_and_stale():
    from legged_control.joint_aggregator import _freshness_status

    latest = {
        "FR_hip": {"position": 0.0, "velocity": 0.0, "stamp": None},
        "FR_thigh": {"position": 0.0, "velocity": 0.0, "stamp": 1.0},
        "FR_calf": {"position": 0.0, "velocity": 0.0, "stamp": 1.8},
    }
    unseen, stale = _freshness_status(latest, now_mono=2.0, timeout=0.5)
    assert unseen == ["FR_hip"]
    assert stale == ["FR_thigh"]


def test_freshness_status_accepts_all_fresh():
    from legged_control.joint_aggregator import _freshness_status

    latest = {
        "FR_hip": {"position": 0.0, "velocity": 0.0, "stamp": 1.7},
        "FR_thigh": {"position": 0.0, "velocity": 0.0, "stamp": 1.8},
    }
    unseen, stale = _freshness_status(latest, now_mono=2.0, timeout=0.5)
    assert unseen == []
    assert stale == []
