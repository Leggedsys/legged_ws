import sys
import os

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from legged_control.policy_monitor_node import (
    _clip_count,
    _format_panel,
    _fresh_label,
    _leg_abs_mean,
)


def test_fresh_label_none():
    assert _fresh_label(None, now=1.0, timeout=0.5) == "--"


def test_fresh_label_ok():
    assert _fresh_label(0.8, now=1.0, timeout=0.5) == "OK"


def test_fresh_label_stale():
    assert _fresh_label(0.4, now=1.0, timeout=0.5) == "STALE"


def test_clip_count_detects_limits():
    target_q = np.array([0.0, 1.0, 1.5], dtype=np.float32)
    q_mins = np.array([0.0, -1.0, 0.0], dtype=np.float32)
    q_maxs = np.array([2.0, 1.0, 2.0], dtype=np.float32)
    assert _clip_count(target_q, q_mins, q_maxs) == 2


def test_leg_abs_mean_groups_per_leg():
    q_rel = np.arange(12, dtype=np.float32)
    order = [
        "FL_hip",
        "FR_hip",
        "RL_hip",
        "RR_hip",
        "FL_thigh",
        "FR_thigh",
        "RL_thigh",
        "RR_thigh",
        "FL_calf",
        "FR_calf",
        "RL_calf",
        "RR_calf",
    ]
    result = _leg_abs_mean(q_rel, order)
    assert result["FL"] == (0.0 + 4.0 + 8.0) / 3.0
    assert result["RR"] == (3.0 + 7.0 + 11.0) / 3.0


def test_format_panel_contains_key_fields():
    text = _format_panel(
        model_name="policy.pt",
        imu_label="OK",
        odom_label="OK",
        joints_label="OK",
        cmd_label="STALE",
        cmd_vel=np.array([0.0, 0.1, -0.2], dtype=np.float32),
        ang_vel=np.array([1.0, 2.0, 3.0], dtype=np.float32),
        gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
        q_rel_abs={"FL": 0.1, "FR": 0.2, "RL": 0.3, "RR": 0.4},
        clip_count=2,
    )
    assert "model=policy.pt" in text
    assert "cmd=STALE" in text
    assert "clip=2/12" in text
    assert "gravity" in text
