"""
position_gait_node

Open-loop position-control gait demo driven by /cmd_vel.

This mode assumes the robot is powered on in the desired neutral pose, so the
motor_bus zero-offset calibration makes that pose correspond to q=0 for all
joints. The node then publishes small joint-space trajectories around zero.

No IMU or policy model is used. Intended for safe air-hanging demos.
"""

import os
import math
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEG_PHASE = {
    "FL": 0.0,
    "RR": 0.0,
    "FR": math.pi,
    "RL": math.pi,
}

TURN_SIGN = {
    "FL": -1.0,
    "RL": -1.0,
    "FR": 1.0,
    "RR": 1.0,
}

# Motor-frame positive directions measured on the real robot.
# hip   : positive = outward(+1) / inward(-1)
# thigh : positive = forward(+1) / backward(-1)
# calf  : positive = forward(+1) / backward(-1)
HIP_OUTWARD_SIGN = {
    "FR": -1.0,
    "FL": 1.0,
    "RR": 1.0,
    "RL": -1.0,
}

THIGH_FORWARD_SIGN = {
    "FR": 1.0,
    "FL": -1.0,
    "RR": 1.0,
    "RL": -1.0,
}

CALF_FORWARD_SIGN = {
    "FR": -1.0,
    "FL": 1.0,
    "RR": -1.0,
    "RL": 1.0,
}


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


def _normalized_drive(
    cmd_vel: np.ndarray, max_vx: float, max_yaw: float
) -> tuple[float, float]:
    vx = 0.0 if max_vx <= 0.0 else _clamp_unit(float(cmd_vel[0]) / max_vx)
    yaw = 0.0 if max_yaw <= 0.0 else _clamp_unit(float(cmd_vel[2]) / max_yaw)
    return vx, yaw


def _activity(vx_drive: float, yaw_drive: float, min_command: float) -> float:
    magnitude = max(abs(vx_drive), abs(yaw_drive))
    return magnitude if magnitude >= min_command else 0.0


def _command_with_timeout(
    cmd_vel: np.ndarray, last_cmd_t: float | None, now: float, timeout: float
) -> np.ndarray:
    if last_cmd_t is None or now - last_cmd_t > timeout:
        return np.zeros(3, dtype=np.float32)
    return cmd_vel


def _joint_targets(
    phase: float,
    vx_drive: float,
    yaw_drive: float,
    joint_names: list[str],
    hip_amp: float,
    thigh_amp: float,
    calf_amp: float,
) -> np.ndarray:
    targets = np.zeros(len(joint_names), dtype=np.float32)
    for i, name in enumerate(joint_names):
        leg, joint = name.split("_", 1)
        stride = _clamp_unit(vx_drive + 0.5 * TURN_SIGN[leg] * yaw_drive)
        leg_phase = phase + LEG_PHASE[leg]
        swing = math.sin(leg_phase)
        lift = max(0.0, math.cos(leg_phase))

        if joint == "hip":
            targets[i] = (
                HIP_OUTWARD_SIGN[leg] * TURN_SIGN[leg] * hip_amp * yaw_drive * swing
            )
        elif joint == "thigh":
            stride_term = THIGH_FORWARD_SIGN[leg] * 0.5 * thigh_amp * stride * swing
            lift_term = 0.5 * thigh_amp * abs(stride) * lift
            targets[i] = stride_term + lift_term
        elif joint == "calf":
            stride_term = CALF_FORWARD_SIGN[leg] * 0.35 * calf_amp * stride * swing
            lift_term = 0.65 * calf_amp * abs(stride) * lift
            targets[i] = stride_term + lift_term
    return targets


def _clip_targets(
    targets: np.ndarray, joint_names: list[str], safe_ranges: dict
) -> np.ndarray:
    clipped = targets.copy()
    for i, name in enumerate(joint_names):
        joint = name.split("_", 1)[1]
        lower, upper = safe_ranges[joint]
        clipped[i] = float(np.clip(clipped[i], lower, upper))
    return clipped


class PositionGaitNode(Node):
    _CMD_TIMEOUT = 0.5

    def __init__(self) -> None:
        super().__init__("position_gait_node")

        cfg = self._load_config()
        teleop_cfg = cfg["teleop"]
        gait_cfg = cfg["position_demo"]
        self._joint_names = [j["name"] for j in cfg["joints"]]

        self._max_vx = float(teleop_cfg["max_vx"])
        self._max_yaw = float(teleop_cfg["max_yaw"])
        self._min_command = float(gait_cfg["min_command"])
        self._base_frequency = float(gait_cfg["base_frequency_hz"])
        self._frequency_scale = float(gait_cfg["frequency_scale_hz"])
        self._hip_amp = float(gait_cfg["hip_amplitude"])
        self._thigh_amp = float(gait_cfg["thigh_amplitude"])
        self._calf_amp = float(gait_cfg["calf_amplitude"])
        self._loop_hz = float(gait_cfg["loop_hz"])
        self._safe_ranges = {
            "hip": (float(gait_cfg["hip_min"]), float(gait_cfg["hip_max"])),
            "thigh": (float(gait_cfg["thigh_min"]), float(gait_cfg["thigh_max"])),
            "calf": (float(gait_cfg["calf_min"]), float(gait_cfg["calf_max"])),
        }

        self._cmd_vel = np.zeros(3, dtype=np.float32)
        self._phase = 0.0
        self._last_cmd_t = None

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_timer(1.0 / self._loop_hz, self._tick)

        self.get_logger().info(
            f"Position gait node ready — {self._loop_hz:.1f} Hz  "
            f"amps=(hip={self._hip_amp}, thigh={self._thigh_amp}, calf={self._calf_amp})"
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        with open(os.path.join(share, "config", "robot.yaml")) as f:
            return yaml.safe_load(f)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )
        self._last_cmd_t = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        cmd_vel = _command_with_timeout(
            self._cmd_vel, self._last_cmd_t, now, self._CMD_TIMEOUT
        )
        if self._last_cmd_t is None or now - self._last_cmd_t > self._CMD_TIMEOUT:
            self.get_logger().warn(
                "position_gait_node: /cmd_vel stale or absent — falling back to zero command",
                throttle_duration_sec=2.0,
            )
        vx_drive, yaw_drive = _normalized_drive(cmd_vel, self._max_vx, self._max_yaw)
        activity = _activity(vx_drive, yaw_drive, self._min_command)

        if activity <= 0.0:
            self._phase = 0.0
            targets = np.zeros(len(self._joint_names), dtype=np.float32)
        else:
            freq = self._base_frequency + self._frequency_scale * activity
            self._phase = (self._phase + 2.0 * math.pi * freq / self._loop_hz) % (
                2.0 * math.pi
            )
            targets = _joint_targets(
                self._phase,
                vx_drive,
                yaw_drive,
                self._joint_names,
                self._hip_amp,
                self._thigh_amp,
                self._calf_amp,
            )
            targets = _clip_targets(targets, self._joint_names, self._safe_ranges)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = targets.tolist()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PositionGaitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
