"""Position-control gait node using analytical IK."""

from __future__ import annotations

import math
import os
import time
from collections import deque

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
import rclpy.parameter
from rcl_interfaces.msg import (
    Parameter,
    ParameterType,
    ParameterValue,
    SetParametersResult,
)
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from legged_control.kinematics import forward_kinematics, inverse_kinematics
from legged_control.standup_node import _smoothstep


_PHASE_STANDUP = "STANDUP"
_PHASE_WAIT = "WAIT"
_PHASE_TROT = "TROT"
_PHASE_FAULT = "FAULT"
_PHASE_PASSIVE = "PASSIVE"
_PHASE_LIE_DOWN = "LIE_DOWN"
_STANDUP_TARGET_TOL = 0.05
_LIE_DOWN_ZERO_TOL = 0.03
_JOINT_SETTLED_VEL_TOL = 0.05

_HIP_POSITIONS = {
    "FL": (0.1426, 0.04654, 0.0),
    "FR": (0.1426, -0.04654, 0.0),
    "RL": (-0.1426, 0.04654, 0.0),
    "RR": (-0.1426, -0.04654, 0.0),
}

_PHASE_OFFSETS = {"FL": 0.0, "RR": 0.0, "FR": math.pi, "RL": math.pi}
_LEG_ORDER = ("FL", "FR", "RL", "RR")


def _command_with_timeout(
    cmd_vel: tuple[float, float, float],
    last_cmd_t: float | None,
    now: float,
    timeout: float,
) -> tuple[float, float, float]:
    if last_cmd_t is None or now - last_cmd_t > timeout:
        return 0.0, 0.0, 0.0
    return cmd_vel


def _command_is_fresh(last_cmd_t: float | None, now: float, timeout: float) -> bool:
    return last_cmd_t is not None and now - last_cmd_t <= timeout


def _phase_is_stance(phase: float) -> bool:
    return phase < math.pi


def _is_effectively_zero_cmd(
    cmd_vel: tuple[float, float, float], eps: float = 1e-6
) -> bool:
    return all(abs(value) <= eps for value in cmd_vel)


def _has_motion_command(cmd_vel: tuple[float, float, float], eps: float = 1e-6) -> bool:
    return not _is_effectively_zero_cmd(cmd_vel, eps)


def _body_to_hip(
    leg: str, foot_body: tuple[float, float, float]
) -> tuple[float, float, float]:
    hip_x, hip_y, hip_z = _HIP_POSITIONS[leg]
    x, y, z = foot_body
    return x - hip_x, y - hip_y, z - hip_z


def _leg_stride(
    nominal_foot: tuple[float, float],
    leg: str,
    cmd_vx: float,
    cmd_vy: float,
    cmd_yaw: float,
    gait_freq: float,
    max_stride_x: float,
    max_stride_y: float,
    yaw_stride_scale: float,
) -> tuple[float, float]:
    x_nom, y_nom = nominal_foot
    stride_x = cmd_vx / (2.0 * gait_freq)
    stride_y = cmd_vy / (2.0 * gait_freq)

    # Yaw command maps to tangential foot motion about the body center.
    stride_x += yaw_stride_scale * (-cmd_yaw * y_nom) / (2.0 * gait_freq)
    stride_y += yaw_stride_scale * (cmd_yaw * x_nom) / (2.0 * gait_freq)

    stride_x = max(-max_stride_x, min(max_stride_x, stride_x))
    stride_y = max(-max_stride_y, min(max_stride_y, stride_y))
    return stride_x, stride_y


def _cubic_bezier(p0: float, p1: float, p2: float, p3: float, t: float) -> float:
    u = 1.0 - t
    return (
        (u**3) * p0
        + 3.0 * (u**2) * t * p1
        + 3.0 * u * (t**2) * p2
        + (t**3) * p3
    )


def _foot_target(
    nominal_foot: tuple[float, float, float],
    leg: str,
    phase: float,
    stance_height: float,
    step_height: float,
    stride_x: float,
    stride_y: float,
) -> tuple[float, float, float]:
    x_nom, y_nom, _ = nominal_foot
    # Hip frame: z negative = below hip. stance_height is a positive distance.
    z_stance = -stance_height
    if _phase_is_stance(phase):
        s = _smoothstep(phase / math.pi)
        x = x_nom + (0.5 - s) * stride_x
        y = y_nom + (0.5 - s) * stride_y
        z = z_stance
    else:
        t = (phase - math.pi) / math.pi
        x_start = x_nom - 0.5 * stride_x
        x_end = x_nom + 0.5 * stride_x
        y_start = y_nom - 0.5 * stride_y
        y_end = y_nom + 0.5 * stride_y
        z_ctrl = z_stance + (4.0 / 3.0) * step_height
        x = _cubic_bezier(x_start, x_start, x_end, x_end, t)
        y = _cubic_bezier(y_start, y_start, y_end, y_end, t)
        z = _cubic_bezier(z_stance, z_ctrl, z_ctrl, z_stance, t)
    return x, y, z


def _motor_targets_from_urdf(
    leg_joints: list[dict], q_urdf: tuple[float, float, float]
) -> tuple[list[float], bool]:
    targets = []
    clipped = False
    for joint_cfg, q_joint_urdf in zip(leg_joints, q_urdf):
        q_motor = float(joint_cfg["direction"]) * (
            float(q_joint_urdf) - float(joint_cfg["zero_offset"])
        )
        q_clipped = min(
            max(q_motor, float(joint_cfg["q_min"])), float(joint_cfg["q_max"])
        )
        clipped = clipped or not math.isclose(q_motor, q_clipped, abs_tol=1e-9)
        targets.append(q_clipped)
    return targets, clipped


def _ik_derived_targets(
    leg: str,
    nominal_xy: tuple[float, float],
    leg_joints: list[dict],
    leg_default_q_urdf: tuple[float, float, float],
    h: float,
) -> list[float] | None:
    """Compute motor targets for one leg with foot placed at stance height h (m, positive down).

    Returns None if IK fails (target is unreachable for this leg geometry).
    """
    x_nom, y_nom = nominal_xy
    foot_hip = _body_to_hip(leg, (x_nom, y_nom, -h))
    q_urdf = inverse_kinematics(leg, foot_hip, leg_default_q_urdf)
    if q_urdf is None:
        return None
    targets, _ = _motor_targets_from_urdf(
        leg_joints, q_urdf
    )  # clipping flag unused here
    return targets


def _clamp_height(
    current_h: float, dz_rate: float, dt: float, h_min: float, h_max: float
) -> float:
    """Integrate dz_rate over dt and clamp the result to [h_min, h_max]."""
    return max(h_min, min(h_max, current_h + dz_rate * dt))


def _clamp_cmd_vel(
    cmd_vel: tuple[float, float, float],
    max_vx: float,
    max_vy: float,
    max_yaw: float,
) -> tuple[float, float, float]:
    vx, vy, yaw = cmd_vel
    return (
        max(-max_vx, min(max_vx, vx)),
        max(-max_vy, min(max_vy, vy)),
        max(-max_yaw, min(max_yaw, yaw)),
    )


def _rate_limit_targets(
    previous: list[float] | None, targets: list[float], max_delta: float
) -> list[float]:
    if previous is None:
        return list(targets)
    out = []
    for prev, target in zip(previous, targets):
        delta = target - prev
        delta = max(-max_delta, min(max_delta, delta))
        out.append(prev + delta)
    return out


def _blend_targets(a: list[float], b: list[float], alpha: float) -> list[float]:
    alpha = max(0.0, min(1.0, alpha))
    return [(1.0 - alpha) * x + alpha * y for x, y in zip(a, b)]


class GaitNode(Node):
    def __init__(self) -> None:
        super().__init__("gait_node")

        self.declare_parameter("config_path", "")
        cfg = self._load_config()
        self._joint_cfg = cfg["joints"]
        self._joint_names = [joint["name"] for joint in self._joint_cfg]
        self._joint_map = {joint["name"]: joint for joint in self._joint_cfg}
        self._leg_joints = {
            leg: [
                self._joint_map[f"{leg}_hip"],
                self._joint_map[f"{leg}_thigh"],
                self._joint_map[f"{leg}_calf"],
            ]
            for leg in _LEG_ORDER
        }
        self._leg_default_q_urdf = {
            leg: tuple(
                float(joint["direction"]) * float(joint["default_q"])
                + float(joint["zero_offset"])
                for joint in self._leg_joints[leg]
            )
            for leg in _LEG_ORDER
        }
        self._nominal_feet = {
            leg: (
                _HIP_POSITIONS[leg][0] + foot_hip[0],
                _HIP_POSITIONS[leg][1] + foot_hip[1],
                foot_hip[2],
            )
            for leg, foot_hip in (
                (leg, forward_kinematics(leg, self._leg_default_q_urdf[leg]))
                for leg in _LEG_ORDER
            )
        }
        self._default_targets = [float(joint["default_q"]) for joint in self._joint_cfg]
        # Will be overwritten by IK-derived values below; kept as per-leg fallback on IK failure.

        control_cfg = cfg["control"]
        standup_cfg = cfg.get("standup", {})
        gait_cfg = cfg.get("gait", {})

        kp_init = float(control_cfg["kp"])
        kd_init = float(control_cfg["kd"])
        self.declare_parameter("kp", kp_init)
        self.declare_parameter("kd", kd_init)
        # In position_control, standup is operator-triggered after motors are already
        # online, so there is no need to keep the legacy pre-ramp hold.
        self.declare_parameter("hold_duration", 0.0)
        self.declare_parameter(
            "ramp_duration", float(standup_cfg.get("ramp_duration", 8.0))
        )
        self.declare_parameter(
            "lie_down_duration", float(standup_cfg.get("lie_down_duration", 2.0))
        )
        fk_stance_height = -sum(self._nominal_feet[leg][2] for leg in _LEG_ORDER) / len(
            _LEG_ORDER
        )
        self.declare_parameter("stance_height", fk_stance_height)
        self._rederive_defaults(fk_stance_height)
        self.declare_parameter("step_height", float(gait_cfg.get("step_height", 0.06)))
        self.declare_parameter("gait_freq", float(gait_cfg.get("gait_freq", 2.0)))
        self.declare_parameter(
            "max_stride_x", float(gait_cfg.get("max_stride_x", 0.08))
        )
        self.declare_parameter(
            "max_stride_y", float(gait_cfg.get("max_stride_y", 0.04))
        )
        self.declare_parameter("idle_timeout", float(gait_cfg.get("idle_timeout", 1.0)))
        self.declare_parameter(
            "safety_limit_window", float(gait_cfg.get("safety_limit_window", 0.5))
        )
        self.declare_parameter(
            "safety_limit_ratio", float(gait_cfg.get("safety_limit_ratio", 0.8))
        )
        self.declare_parameter("max_cmd_vx", float(gait_cfg.get("max_cmd_vx", 0.15)))
        self.declare_parameter("max_cmd_vy", float(gait_cfg.get("max_cmd_vy", 0.08)))
        self.declare_parameter("max_cmd_yaw", float(gait_cfg.get("max_cmd_yaw", 0.4)))
        self.declare_parameter(
            "max_joint_speed", float(gait_cfg.get("max_joint_speed", 2.0))
        )
        self.declare_parameter(
            "fault_hold_kp", float(gait_cfg.get("fault_hold_kp", 0.8))
        )
        self.declare_parameter(
            "fault_hold_kd", float(gait_cfg.get("fault_hold_kd", 0.2))
        )
        self.declare_parameter(
            "motion_blend_duration", float(gait_cfg.get("motion_blend_duration", 0.6))
        )
        self.declare_parameter(
            "yaw_stride_scale", float(gait_cfg.get("yaw_stride_scale", 1.8))
        )
        self.declare_parameter(
            "tracking_error_threshold",
            float(gait_cfg.get("tracking_error_threshold", 0.20)),
        )
        self.declare_parameter(
            "tracking_error_window",
            float(gait_cfg.get("tracking_error_window", 0.4)),
        )
        self.declare_parameter(
            "tracking_error_ratio",
            float(gait_cfg.get("tracking_error_ratio", 0.7)),
        )
        self.declare_parameter(
            "tracking_error_grace_period",
            float(gait_cfg.get("tracking_error_grace_period", 1.0)),
        )
        self.declare_parameter(
            "stance_height_min", float(gait_cfg.get("stance_height_min", 0.20))
        )
        self.declare_parameter(
            "stance_height_max", float(gait_cfg.get("stance_height_max", 0.35))
        )
        self.declare_parameter("skip_standup", False)

        self._loop_hz = float(control_cfg.get("gait_hz", 50.0))
        self._dt = 1.0 / self._loop_hz
        self._phase = _PHASE_PASSIVE
        self._phase_start: float | None = None
        self._oscillator = 0.0
        self._last_cmd_t: float | None = None
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._dz_rate = 0.0  # written by _on_cmd_vel, read by _tick; safe under SingleThreadedExecutor
        self._joint_state_seen = False
        self._latest_joint_pos = {name: None for name in self._joint_names}
        self._latest_joint_vel = {name: None for name in self._joint_names}
        self._fault_broadcast = False
        self._passive_broadcast = False
        self._estop_active = False
        self._stand_requested = False
        self._last_published_targets: list[float] | None = None
        self._lie_down_start_targets: list[float] | None = None
        self._motion_started_at: float | None = None

        self._gain_clients = [
            self.create_client(SetParameters, "/motor_bus_front/set_parameters"),
            self.create_client(SetParameters, "/motor_bus_rear/set_parameters"),
        ]

        self._limit_hits = deque()
        self._tracking_error_hits = deque()

        self._pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/posture_command", self._on_posture_command, 10)
        self.create_subscription(Bool, "/emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(
            JointState, "/joint_states_aggregated", self._on_joint_state, 10
        )
        self.add_on_set_parameters_callback(self._on_gains_changed)
        self.create_timer(self._dt, self._tick)

        yaml_stance = float(gait_cfg.get("stance_height", 0.28))
        if abs(fk_stance_height - yaml_stance) > 0.005:
            self.get_logger().warn(
                f"[gait] stance_height in yaml ({yaml_stance:.4f} m) differs from "
                f"default_q FK ({fk_stance_height:.4f} m); using FK value"
            )
        self.get_logger().info(
            f"gait_node ready — position_control at {self._loop_hz:.1f} Hz  "
            f"kp={kp_init}  kd={kd_init}  stance_height={fk_stance_height:.4f} m"
        )
        self.get_logger().info(
            "[gait] waiting for /posture_command=true before enabling torque and starting standup"
        )

    def _load_config(self) -> dict:
        share = get_package_share_directory("legged_control")
        config_path = str(self.get_parameter("config_path").value or "").strip()
        if not config_path:
            config_path = os.path.join(share, "config", "robot.yaml")
        with open(config_path) as f:
            return yaml.safe_load(f)

    def _rederive_defaults(self, h: float) -> None:
        """Recompute _default_targets and _nominal_feet z for all legs at stance height h."""
        for leg in _LEG_ORDER:
            x_nom, y_nom, _ = self._nominal_feet[leg]
            targets = _ik_derived_targets(
                leg,
                (x_nom, y_nom),
                self._leg_joints[leg],
                self._leg_default_q_urdf[leg],
                h,
            )
            if targets is None:
                self.get_logger().warn(
                    f"[gait] IK failed for {leg} at stance_height={h:.4f} m; "
                    "keeping previous _default_targets for that leg"
                )
                continue
            for joint, target in zip(self._leg_joints[leg], targets):
                idx = self._joint_names.index(joint["name"])
                self._default_targets[idx] = target
            self._nominal_feet[leg] = (x_nom, y_nom, -h)

    def _integrate_height(self) -> None:
        """Integrate _dz_rate into stance_height for WAIT/TROT phases.

        Only called from WAIT/TROT; caller is responsible for phase isolation.
        """
        if abs(self._dz_rate) < 1e-9:
            return
        current_h = float(self.get_parameter("stance_height").value)
        h_min = float(self.get_parameter("stance_height_min").value)
        h_max = float(self.get_parameter("stance_height_max").value)
        new_h = _clamp_height(current_h, self._dz_rate, self._dt, h_min, h_max)
        # 0.01 mm threshold: prevents IK on floating-point-only ticks near the clamp boundary.
        # (Spec suggested 0.5 mm, but that silently swallows moderate trigger inputs at 50 Hz.)
        if abs(new_h - current_h) > 1e-5:
            self.set_parameters(
                [
                    rclpy.parameter.Parameter(
                        "stance_height",
                        rclpy.parameter.Parameter.Type.DOUBLE,
                        new_h,
                    )
                ]
            )
            self._rederive_defaults(new_h)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self._dz_rate = float(msg.linear.z)
        self._last_cmd_t = time.monotonic()

    def _on_emergency_stop(self, msg: Bool) -> None:
        was_active = self._estop_active
        self._estop_active = bool(msg.data)
        if self._estop_active and not was_active:
            self._phase = _PHASE_FAULT
            self._fault_broadcast = False
            self.get_logger().error("[gait] emergency stop requested")
        elif was_active and not self._estop_active:
            self.get_logger().warn(
                "[gait] emergency stop cleared, but node remains in FAULT until relaunch"
            )

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state_seen = True
        positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        velocities = {name: vel for name, vel in zip(msg.name, msg.velocity)}
        for name in self._joint_names:
            if name in positions:
                self._latest_joint_pos[name] = float(positions[name])
            if name in velocities:
                self._latest_joint_vel[name] = float(velocities[name])

    def _on_posture_command(self, msg: Bool) -> None:
        if self._phase == _PHASE_FAULT:
            self.get_logger().warn(
                "[gait] ignoring posture command while FAULT is active",
                throttle_duration_sec=1.0,
            )
            return

        if bool(msg.data):
            if self._phase != _PHASE_PASSIVE:
                self.get_logger().info(
                    "[gait] posture command=true received while already active; keeping current posture",
                    throttle_duration_sec=1.0,
                )
                return
            self._stand_requested = True
            self.get_logger().info("[gait] posture command=true received — preparing standup")
            return

        if self._phase == _PHASE_PASSIVE:
            self.get_logger().info(
                "[gait] posture command=false received while already passive; keeping current posture",
                throttle_duration_sec=1.0,
            )
            return

        if self._phase == _PHASE_LIE_DOWN:
            self.get_logger().warn(
                "[gait] posture command=false ignored while lie-down is in progress",
                throttle_duration_sec=1.0,
            )
            return

        self._phase = _PHASE_LIE_DOWN
        self._phase_start = time.monotonic()
        self._lie_down_start_targets = list(
            self._last_published_targets or self._default_targets
        )
        self._motion_started_at = None
        self.get_logger().info("[gait] posture command=false received — starting lie-down")

    def _on_gains_changed(self, params: list) -> SetParametersResult:
        new_kp = next((p.value for p in params if p.name == "kp"), None)
        new_kd = next((p.value for p in params if p.name == "kd"), None)
        if new_kp is not None or new_kd is not None:
            kp = new_kp if new_kp is not None else self.get_parameter("kp").value
            kd = new_kd if new_kd is not None else self.get_parameter("kd").value
            self._broadcast_gains(float(kp), float(kd))
            self.get_logger().info(f"[gait] gains updated → kp={kp}  kd={kd}")
        return SetParametersResult(successful=True)

    def _broadcast_gains(self, kp: float, kd: float) -> None:
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name="kp",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(kp)
                ),
            ),
            Parameter(
                name="kd",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(kd)
                ),
            ),
        ]
        for client in self._gain_clients:
            if client.service_is_ready():
                client.call_async(req)

    def _publish_positions(self, positions: list[float]) -> None:
        max_joint_speed = float(self.get_parameter("max_joint_speed").value)
        max_delta = max_joint_speed * self._dt
        positions = _rate_limit_targets(
            self._last_published_targets, positions, max_delta
        )
        self._last_published_targets = list(positions)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = positions
        self._pub.publish(msg)

    def _fault_targets(self) -> list[float]:
        return list(self._last_published_targets or self._default_targets)

    def _enter_active_mode(self, now: float) -> None:
        self._broadcast_gains(
            float(self.get_parameter("kp").value), float(self.get_parameter("kd").value)
        )
        self._stand_requested = False
        self._phase = (
            _PHASE_WAIT
            if bool(self.get_parameter("skip_standup").value)
            else _PHASE_STANDUP
        )
        self._phase_start = now
        self._fault_broadcast = False
        self._passive_broadcast = False
        actual = [self._latest_joint_pos[n] for n in self._joint_names]
        self._last_published_targets = (
            [float(v) for v in actual] if all(v is not None for v in actual) else None
        )
        self._lie_down_start_targets = None
        if self._phase == _PHASE_WAIT:
            self.get_logger().info(
                "[gait] posture command=true received — skipping standup and holding default pose"
            )
        else:
            self.get_logger().info("[gait] posture command=true received — starting standup")

    def _enter_passive_mode(self) -> None:
        self._phase = _PHASE_PASSIVE
        self._phase_start = None
        self._oscillator = 0.0
        self._joint_state_seen = False
        self._limit_hits.clear()
        self._tracking_error_hits.clear()
        self._last_published_targets = None
        self._lie_down_start_targets = None
        self._fault_broadcast = False
        self._stand_requested = False
        self._motion_started_at = None
        if not self._passive_broadcast:
            self._broadcast_gains(0.0, 0.0)
            self._passive_broadcast = True
            self.get_logger().warn(
                "[gait] passive mode — torque disabled until /posture_command=true requests standup"
            )

    def _standup_targets(self, elapsed: float) -> tuple[list[float], bool]:
        hold_duration = float(self.get_parameter("hold_duration").value)
        ramp_duration = float(self.get_parameter("ramp_duration").value)
        if elapsed < hold_duration:
            return [0.0] * len(self._joint_names), False
        if elapsed < hold_duration + ramp_duration:
            alpha = _smoothstep((elapsed - hold_duration) / ramp_duration)
            return [alpha * q for q in self._default_targets], False
        return list(self._default_targets), True

    def _lie_down_targets(self, elapsed: float) -> tuple[list[float], bool]:
        duration = max(float(self.get_parameter("lie_down_duration").value), 1e-6)
        alpha = _smoothstep(elapsed / duration)
        start_targets = self._lie_down_start_targets or self._default_targets
        return _blend_targets(
            start_targets, [0.0] * len(start_targets), alpha
        ), alpha >= 1.0

    def _is_near_zero_pose(self, tol: float = _LIE_DOWN_ZERO_TOL) -> bool:
        values = [self._latest_joint_pos[name] for name in self._joint_names]
        if any(value is None for value in values):
            return False
        return all(abs(float(value)) <= tol for value in values)

    def _is_settled(self, vel_tol: float = _JOINT_SETTLED_VEL_TOL) -> bool:
        values = [self._latest_joint_vel[name] for name in self._joint_names]
        if any(value is None for value in values):
            return False
        return all(abs(float(value)) <= vel_tol for value in values)

    def _is_near_targets(self, targets: list[float], tol: float) -> bool:
        values = [self._latest_joint_pos[name] for name in self._joint_names]
        if any(value is None for value in values):
            return False
        return all(
            abs(float(value) - target) <= tol for value, target in zip(values, targets)
        )

    def _current_motion_cmd(self, now: float) -> tuple[float, float, float]:
        idle_timeout = float(self.get_parameter("idle_timeout").value)
        cmd_vx, cmd_vy, cmd_yaw = _command_with_timeout(
            self._cmd_vel, self._last_cmd_t, now, idle_timeout
        )
        return _clamp_cmd_vel(
            (cmd_vx, cmd_vy, cmd_yaw),
            float(self.get_parameter("max_cmd_vx").value),
            float(self.get_parameter("max_cmd_vy").value),
            float(self.get_parameter("max_cmd_yaw").value),
        )

    def _trot_targets(self) -> tuple[list[float], bool]:
        now = time.monotonic()
        step_height = float(self.get_parameter("step_height").value)
        gait_freq = float(self.get_parameter("gait_freq").value)
        max_stride_x = float(self.get_parameter("max_stride_x").value)
        max_stride_y = float(self.get_parameter("max_stride_y").value)
        yaw_stride_scale = float(self.get_parameter("yaw_stride_scale").value)
        cmd_vx, cmd_vy, cmd_yaw = self._current_motion_cmd(now)
        if _is_effectively_zero_cmd((cmd_vx, cmd_vy, cmd_yaw)):
            self._motion_started_at = None
            return list(self._default_targets), False

        if self._motion_started_at is None:
            self._motion_started_at = now

        self._oscillator = (self._oscillator + 2.0 * math.pi * gait_freq * self._dt) % (
            2.0 * math.pi
        )

        targets_by_name = {}
        clipped_any = False
        for leg in _LEG_ORDER:
            leg_phase = (self._oscillator + _PHASE_OFFSETS[leg]) % (2.0 * math.pi)
            stride_x, stride_y = _leg_stride(
                self._nominal_feet[leg][:2],
                leg,
                cmd_vx,
                cmd_vy,
                cmd_yaw,
                gait_freq,
                max_stride_x,
                max_stride_y,
                yaw_stride_scale,
            )
            foot_body = _foot_target(
                self._nominal_feet[leg],
                leg,
                leg_phase,
                float(self.get_parameter("stance_height").value),
                step_height,
                stride_x,
                stride_y,
            )
            foot_hip = _body_to_hip(leg, foot_body)
            q_urdf = inverse_kinematics(leg, foot_hip, self._leg_default_q_urdf[leg])
            if q_urdf is None:
                for joint in self._leg_joints[leg]:
                    targets_by_name[joint["name"]] = float(joint["default_q"])
                clipped_any = True
                self.get_logger().warn(
                    f"[gait] IK failed for {leg}; holding default_q for that leg",
                    throttle_duration_sec=1.0,
                )
                continue
            leg_targets, leg_clipped = _motor_targets_from_urdf(
                self._leg_joints[leg], q_urdf
            )
            clipped_any = clipped_any or leg_clipped
            for joint, target in zip(self._leg_joints[leg], leg_targets):
                targets_by_name[joint["name"]] = target

        gait_targets = [targets_by_name[name] for name in self._joint_names]
        blend_duration = float(self.get_parameter("motion_blend_duration").value)
        if blend_duration > 0.0 and self._motion_started_at is not None:
            alpha = (now - self._motion_started_at) / blend_duration
            gait_targets = _blend_targets(self._default_targets, gait_targets, alpha)

        return gait_targets, clipped_any

    def _update_limit_safety(self, hit_limit: bool) -> None:
        window_seconds = float(self.get_parameter("safety_limit_window").value)
        limit_ratio = float(self.get_parameter("safety_limit_ratio").value)
        window_len = max(1, int(round(window_seconds * self._loop_hz)))
        self._limit_hits.append(hit_limit)
        while len(self._limit_hits) > window_len:
            self._limit_hits.popleft()
        if len(self._limit_hits) < window_len:
            return
        ratio = sum(self._limit_hits) / float(window_len)
        if ratio >= limit_ratio:
            self._phase = _PHASE_FAULT
            self._fault_broadcast = False
            self.get_logger().error(
                f"[gait] sustained joint limit / IK failure ratio {ratio:.2f} >= {limit_ratio:.2f}; entering FAULT"
            )

    def _tracking_error_exceeded(self, targets: list[float], threshold: float) -> bool:
        values = [self._latest_joint_pos[name] for name in self._joint_names]
        if any(value is None for value in values):
            return False
        return any(
            abs(float(value) - target) > threshold
            for value, target in zip(values, targets)
        )

    def _update_tracking_error_safety(self, targets: list[float]) -> None:
        now = time.monotonic()
        grace_period = float(self.get_parameter("tracking_error_grace_period").value)
        if self._phase != _PHASE_TROT:
            self._tracking_error_hits.clear()
            return
        if self._motion_started_at is not None and now - self._motion_started_at < grace_period:
            self._tracking_error_hits.clear()
            return

        threshold = float(self.get_parameter("tracking_error_threshold").value)
        window_seconds = float(self.get_parameter("tracking_error_window").value)
        error_ratio = float(self.get_parameter("tracking_error_ratio").value)
        window_len = max(1, int(round(window_seconds * self._loop_hz)))
        self._tracking_error_hits.append(
            self._tracking_error_exceeded(targets, threshold)
        )
        while len(self._tracking_error_hits) > window_len:
            self._tracking_error_hits.popleft()
        if len(self._tracking_error_hits) < window_len:
            return
        ratio = sum(self._tracking_error_hits) / float(window_len)
        if ratio >= error_ratio:
            self._phase = _PHASE_FAULT
            self._fault_broadcast = False
            self.get_logger().error(
                f"[gait] sustained joint tracking error ratio {ratio:.2f} >= {error_ratio:.2f}; entering FAULT"
            )

    def _tick(self) -> None:
        now = time.monotonic()

        if self._estop_active:
            self._phase = _PHASE_FAULT

        if self._phase != _PHASE_FAULT and self._phase == _PHASE_PASSIVE:
            if self._stand_requested:
                self._enter_active_mode(now)
            else:
                self._enter_passive_mode()
                return

        if self._phase_start is None:
            self._phase_start = now

        if self._phase == _PHASE_FAULT:
            if not self._fault_broadcast:
                self._broadcast_gains(
                    float(self.get_parameter("fault_hold_kp").value),
                    float(self.get_parameter("fault_hold_kd").value),
                )
                self._fault_broadcast = True
                self.get_logger().warn(
                    "[gait] FAULT active — commanding safe default pose with reduced gains"
                )
            self._publish_positions(self._fault_targets())
            return

        if self._phase == _PHASE_LIE_DOWN:
            targets, done = self._lie_down_targets(now - self._phase_start)
            self._publish_positions(targets)
            if done and self._is_near_zero_pose() and self._is_settled():
                self._enter_passive_mode()
            return

        if self._phase == _PHASE_STANDUP:
            targets, ramp_complete = self._standup_targets(now - self._phase_start)
            self._publish_positions(targets)
            if ramp_complete and self._is_near_targets(
                self._default_targets, _STANDUP_TARGET_TOL
            ) and self._is_settled():
                self._phase = _PHASE_WAIT
                self._phase_start = now
                self.get_logger().info(
                    "[gait] standup complete — waiting for /joint_states_aggregated"
                )
            return

        if self._phase == _PHASE_WAIT:
            self._integrate_height()
            targets = list(self._default_targets)
            self._publish_positions(targets)
            if self._joint_state_seen and _has_motion_command(
                self._current_motion_cmd(now)
            ):
                self._phase = _PHASE_TROT
                self._phase_start = now
                self._oscillator = 0.0
                actual = [self._latest_joint_pos[n] for n in self._joint_names]
                if all(v is not None for v in actual):
                    self._last_published_targets = [float(v) for v in actual]
                self.get_logger().info("[gait] motion command received — entering TROT")
            return

        self._integrate_height()
        if not _has_motion_command(self._current_motion_cmd(now)):
            self._phase = _PHASE_WAIT
            self._phase_start = now
            self._motion_started_at = None
            self._publish_positions(list(self._default_targets))
            self.get_logger().info(
                "[gait] motion command cleared — returning to standing idle",
                throttle_duration_sec=1.0,
            )
            return
        targets, clipped_any = self._trot_targets()
        self._publish_positions(targets)
        self._update_limit_safety(clipped_any)
        self._update_tracking_error_safety(self._last_published_targets or targets)


def main() -> None:
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
