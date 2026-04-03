"""Manages subprocess lifecycle for all 12 motor driver nodes."""

from __future__ import annotations
import subprocess
import time


def _joint_name_to_ns(name: str) -> str:
    """'FR_hip' → 'fr/hip'"""
    leg, joint = name.split('_', 1)
    return f'{leg.lower()}/{joint.lower()}'


class MotorManager:
    """Launches and terminates motor driver nodes as subprocesses.

    Args:
        joints_cfg: List of joint dicts from robot.yaml (name, motor_id, default_q, q_min, q_max).
        serial_port: Serial port string e.g. '/dev/ttyUSB0'.
    """

    def __init__(self, joints_cfg: list[dict], serial_port: str) -> None:
        self._joints = joints_cfg
        self._serial = serial_port
        self._procs: list[subprocess.Popen] = []

    def launch(self, kp: float = 0.0, kd: float = 0.0,
               targets: dict[str, float] | None = None) -> None:
        """Start all motor nodes. If targets is None, each joint uses its default_q.

        target_q is clamped to [q_min, q_max] before sending.
        """
        self.shutdown()
        for j in self._joints:
            ns = _joint_name_to_ns(j['name'])
            raw_q = (targets or {}).get(j['name'], j['default_q'])
            target_q = max(j.get('q_min', -999.0), min(j.get('q_max', 999.0), raw_q))
            cmd = [
                'ros2', 'run', 'unitree_actuator_sdk', 'go_m8010_6_node',
                '--ros-args',
                '-r', f'__ns:=/{ns}',
                '-p', f'motor_id:={j["motor_id"]}',
                '-p', f'joint_name:={j["name"]}',
                '-p', f'serial_port:={self._serial}',
                '-p', 'loop_hz:=1000.0',
                '-p', f'kp:={kp}',
                '-p', f'kd:={kd}',
                '-p', f'target_q:={target_q}',
                '-p', 'target_dq:=0.0',
                '-p', 'tau:=0.0',
            ]
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._procs.append(proc)
        time.sleep(1.0)   # allow nodes to initialize

    def shutdown(self) -> None:
        """Terminate all motor nodes."""
        for p in self._procs:
            p.terminate()
        for p in self._procs:
            try:
                p.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                p.kill()
        self._procs.clear()

    def zero_torque(self) -> None:
        """Relaunch all nodes with kp=kd=0 (free-wheeling)."""
        cfg = {j['name']: j['default_q'] for j in self._joints}
        self.launch(kp=0.0, kd=0.0, targets=cfg)
