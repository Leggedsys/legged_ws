#!/usr/bin/env python3
"""
auto_tune_pd.py — automated PD gain tuning via chirp bandwidth measurement.

Requires real.launch.py to be running (motor_bus_front at minimum).
Tests FR leg only. Hip is NOT moved (hardware constraint).
Scales kd proportionally with kp. Calf gains set at ~2/3 of hip/thigh.
Target: bandwidth >= 4 Hz without oscillation.
"""
import glob
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

_YAML_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

F0 = 0.5
F1 = 8.0
RAMP_DUR = 3.0
TEST_DUR = 18.0   # total seconds (includes ramp)
TARGET_BW = 4.0   # Hz

# kp to try, in order
KP_SCHEDULE = [2.0, 2.5, 3.0, 3.5, 4.0, 4.5]

# Fixed ratio: kd/kp and kp_calf/kp derived from original yaml values
KD_RATIO = 0.07 / 1.5       # ≈ 0.047
KP_CALF_RATIO = 1.0 / 1.5   # ≈ 0.667
KD_CALF_RATIO = 0.05 / 1.0  # = 0.05


def _ros2_param_set(node: str, param: str, value: float) -> bool:
    try:
        subprocess.run(
            ["ros2", "param", "set", node, param, str(round(value, 4))],
            check=True, capture_output=True, timeout=5,
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return False


def set_gains(kp: float, kd: float, kp_calf: float, kd_calf: float) -> None:
    for bus in ["/motor_bus_front", "/motor_bus_rear"]:
        ok = _ros2_param_set(bus, "kp", kp)
        _ros2_param_set(bus, "kd", kd)
        if not ok:
            print(f"    (bus {bus} not reachable — skipped)")

    for joint in ["FR_calf", "FL_calf", "RR_calf", "RL_calf"]:
        bus = "/motor_bus_front" if joint.split("_")[0] in ("FR", "FL") else "/motor_bus_rear"
        _ros2_param_set(bus, f"kp_{joint}", kp_calf)
        _ros2_param_set(bus, f"kd_{joint}", kd_calf)


def run_chirp_test() -> Path:
    before = set(glob.glob(os.path.expanduser("~/.legged_logs/leg_track_*")))
    cmd = [
        "ros2", "run", "legged_control", "leg_track_node",
        "--ros-args",
        "-p", "leg:=FR",
        "-p", "mode:=chirp",
        "-p", f"freq:={float(F0)}",
        "-p", f"freq_end:={float(F1)}",
        "-p", f"duration:={float(TEST_DUR)}",
        "-p", "hip_amp:=0.0",
        "-p", "thigh_amp:=0.25",
        "-p", "calf_amp:=0.25",
        "-p", "thigh_center:=0.8",
        "-p", "calf_center:=-1.5",
        "-p", f"ramp_dur:={float(RAMP_DUR)}",
    ]
    subprocess.run(cmd, timeout=TEST_DUR + 15)

    after = set(glob.glob(os.path.expanduser("~/.legged_logs/leg_track_*")))
    new_dirs = sorted(after - before)
    if not new_dirs:
        raise RuntimeError("No new log directory after chirp test")
    csv_path = Path(new_dirs[-1]) / "leg_track.csv"
    if not csv_path.exists():
        raise RuntimeError(f"CSV not found: {csv_path}")
    return csv_path


def estimate_bandwidth(csv_path: Path, joint: str = "FR_thigh") -> dict:
    data = np.genfromtxt(csv_path, delimiter=",", names=True)

    t = data["t_mono"] - data["t_mono"][0]
    cmd = data[f"cmd_{joint}"]
    act = data[f"act_{joint}"]

    # Use test phase only (after ramp + 0.5s buffer)
    t_test = t - RAMP_DUR
    mask = t_test > 0.5
    t_test = t_test[mask]
    cmd = cmd[mask]
    act = act[mask]

    if len(t_test) < 20:
        return {"bandwidth_hz": 0.0, "oscillation": False, "freq_centers": [], "ratios": []}

    total_test = t_test[-1]
    freq = F0 + (F1 - F0) * t_test / total_test

    # Per-frequency-bin amplitude ratio
    n_bins = 15
    edges = np.linspace(F0, F1, n_bins + 1)
    centers = (edges[:-1] + edges[1:]) / 2
    ratios = []
    for i in range(n_bins):
        m = (freq >= edges[i]) & (freq < edges[i + 1])
        if m.sum() < 5:
            ratios.append(np.nan)
            continue
        c = cmd[m] - cmd[m].mean()
        a = act[m] - act[m].mean()
        c_rms = float(np.sqrt(np.mean(c ** 2)))
        a_rms = float(np.sqrt(np.mean(a ** 2)))
        ratios.append(a_rms / c_rms if c_rms > 0.005 else np.nan)
    ratios = np.array(ratios)

    # Find -3dB frequency
    bw = None
    for i in range(len(centers) - 1):
        if np.isnan(ratios[i]) or np.isnan(ratios[i + 1]):
            continue
        if ratios[i] >= 0.707 >= ratios[i + 1]:
            t_interp = (0.707 - ratios[i]) / (ratios[i + 1] - ratios[i])
            bw = centers[i] + t_interp * (centers[i + 1] - centers[i])
            break
    if bw is None:
        bw = F1 if (not np.isnan(ratios[0]) and ratios[0] < 0.707) else float(np.nan)

    # Oscillation: actual RMS > 1.5× commanded at low freq (<2Hz)
    oscillation = False
    low = freq < 2.0
    if low.sum() > 20:
        c_rms = float(np.sqrt(np.mean((cmd[low] - cmd[low].mean()) ** 2)))
        a_rms = float(np.sqrt(np.mean((act[low] - act[low].mean()) ** 2)))
        if c_rms > 0.01 and a_rms / c_rms > 1.5:
            oscillation = True

    return {
        "bandwidth_hz": float(bw) if bw is not None else float("nan"),
        "oscillation": oscillation,
        "freq_centers": centers.tolist(),
        "ratios": [float(r) for r in ratios],
    }


def print_ratio_bar(freq_centers, ratios):
    """ASCII bar chart of tracking ratio vs frequency."""
    print("    Freq(Hz) | Ratio | Bar")
    for f, r in zip(freq_centers, ratios):
        if np.isnan(r):
            continue
        bar = "█" * int(r * 20)
        marker = " ← -3dB" if abs(r - 0.707) < 0.1 else ""
        print(f"    {f:5.1f}    | {r:5.3f} | {bar}{marker}")


def main():
    print("=" * 60)
    print(f"Auto PD Tuner  target={TARGET_BW} Hz  hip=FIXED")
    print(f"Testing: FR thigh+calf chirp {F0}→{F1} Hz  {TEST_DUR}s")
    print("=" * 60)
    print()

    results = []
    final = {"kp": KP_SCHEDULE[0], "kd": round(KP_SCHEDULE[0] * KD_RATIO, 3),
             "kp_calf": round(KP_SCHEDULE[0] * KP_CALF_RATIO, 2),
             "kd_calf": round(KP_SCHEDULE[0] * KP_CALF_RATIO * KD_CALF_RATIO, 3)}

    for kp in KP_SCHEDULE:
        kd = round(kp * KD_RATIO, 3)
        kp_calf = round(kp * KP_CALF_RATIO, 2)
        kd_calf = round(kp_calf * KD_CALF_RATIO, 3)

        print(f"┌─ kp={kp:.1f}  kd={kd:.3f}  kp_calf={kp_calf:.2f}  kd_calf={kd_calf:.3f}")
        set_gains(kp, kd, kp_calf, kd_calf)
        time.sleep(1.0)

        print(f"│  Running {TEST_DUR}s chirp test...")
        try:
            csv_path = run_chirp_test()
        except Exception as e:
            print(f"│  [ERROR] {e}")
            continue

        result = estimate_bandwidth(csv_path)
        bw = result["bandwidth_hz"]
        osc = result["oscillation"]
        print(f"│  Bandwidth: {bw:.2f} Hz  Oscillation: {osc}")
        print_ratio_bar(result["freq_centers"], result["ratios"])

        results.append({"kp": kp, "kd": kd, "kp_calf": kp_calf, "kd_calf": kd_calf,
                        "bw": bw, "osc": osc, "csv": str(csv_path)})

        if osc:
            print(f"└─ [STOP] Oscillation at kp={kp:.1f} — backing off to previous")
            if len(results) >= 2:
                prev = results[-2]
                final = {"kp": prev["kp"], "kd": prev["kd"],
                         "kp_calf": prev["kp_calf"], "kd_calf": prev["kd_calf"]}
            else:
                final = {"kp": kp, "kd": kd, "kp_calf": kp_calf, "kd_calf": kd_calf}
            break

        final = {"kp": kp, "kd": kd, "kp_calf": kp_calf, "kd_calf": kd_calf}

        if not np.isnan(bw) and bw >= TARGET_BW:
            print(f"└─ [DONE] Target {TARGET_BW} Hz reached!")
            break
        else:
            print(f"└─ {bw:.2f} Hz < {TARGET_BW} Hz, stepping up...\n")

    print()
    print("=" * 60)
    print("RESULT SUMMARY")
    print(f"{'kp':>5} {'kd':>6} {'kp_c':>6} {'kd_c':>6} {'bw_hz':>7} {'osc':>5}")
    for r in results:
        bw_str = f"{r['bw']:.2f}" if not np.isnan(r["bw"]) else "  nan"
        print(f"{r['kp']:>5.1f} {r['kd']:>6.3f} {r['kp_calf']:>6.2f} "
              f"{r['kd_calf']:>6.3f} {bw_str:>7} {str(r['osc']):>5}")
    print()
    print("RECOMMENDED (apply to robot.yaml control section):")
    print(f"  kp:       {final['kp']}")
    print(f"  kd:       {final['kd']}")
    print(f"  kp_calf:  {final['kp_calf']}")
    print(f"  kd_calf:  {final['kd_calf']}")

    # Apply final gains
    print()
    print("Applying recommended gains now...")
    set_gains(final["kp"], final["kd"], final["kp_calf"], final["kd_calf"])
    print("Done. Verify with another chirp test if needed.")


if __name__ == "__main__":
    main()
