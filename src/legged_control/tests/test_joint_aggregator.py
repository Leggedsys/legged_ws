# src/legged_control/tests/test_joint_aggregator.py
"""Tests for joint_aggregator pure helper functions (no ROS2 required)."""
import pytest


def test_ns_from_joint_name_hip():
    from legged_control.joint_aggregator import _ns_from_joint_name
    assert _ns_from_joint_name('FR_hip') == 'fr/hip'


def test_ns_from_joint_name_calf():
    from legged_control.joint_aggregator import _ns_from_joint_name
    assert _ns_from_joint_name('RL_calf') == 'rl/calf'


def test_ns_from_joint_name_thigh():
    from legged_control.joint_aggregator import _ns_from_joint_name
    assert _ns_from_joint_name('FL_thigh') == 'fl/thigh'
