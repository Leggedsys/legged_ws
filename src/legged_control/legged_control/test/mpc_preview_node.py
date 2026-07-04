"""mpc_preview_node — offline MPC visualization without hardware.

Publishes fake sensor data so mpc_node can run standalone, then relays
/joint_commands → /joint_states so robot_state_publisher shows the
commanded configuration in RViz.

Starts automatically: posture_command=true is sent after STARTUP_DELAY.
The fake /joint_states_aggregated reflects the commanded positions so
mpc_node's convergence checks see realistic tracking.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

_YAML_JOINTS = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

_STARTUP_DELAY = 2.0   # seconds before posture_command=true


class MPCPreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("mpc_preview_node")
        self._start = time.monotonic()
        self._posture_sent = False
        # Starts at all-zeros (flat); updated each time /joint_commands arrives
        self._fake_pos: list[float] = [0.0] * 12

        # Fake sensor publishers
        self._pub_agg  = self.create_publisher(JointState, "/joint_states_aggregated", 10)
        self._pub_est  = self.create_publisher(Float32MultiArray, "/state_estimate", 10)
        self._pub_post = self.create_publisher(Bool, "/posture_command", 10)
        # RViz: relay commanded joints → robot_state_publisher
        self._pub_vis  = self.create_publisher(JointState, "/joint_states", 10)

        self.create_subscription(JointState, "/joint_commands", self._on_cmd, 10)

        self.create_timer(1.0 / 200.0, self._tick)
        self.get_logger().info(
            f"mpc_preview ready  (posture=true in {_STARTUP_DELAY:.0f} s)"
        )

    def _on_cmd(self, msg: JointState) -> None:
        """Forward commanded positions to RViz; track as fake actual."""
        self._fake_pos = list(msg.position)

        vis = JointState()
        vis.header.stamp = self.get_clock().now().to_msg()
        vis.name     = list(msg.name)
        vis.position = list(msg.position)
        self._pub_vis.publish(vis)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._start
        stamp   = self.get_clock().now().to_msg()

        # Trigger standup once
        if elapsed >= _STARTUP_DELAY and not self._posture_sent:
            self._pub_post.publish(Bool(data=True))
            self._posture_sent = True
            self.get_logger().info("mpc_preview: posture_command=true → standup")

        # Fake /joint_states_aggregated — reflects commanded positions so
        # mpc_node's is_near / is_settled convergence checks behave correctly
        js = JointState()
        js.header.stamp = stamp
        js.name     = list(_YAML_JOINTS)
        js.position = list(self._fake_pos)
        js.velocity = [0.0] * 12
        self._pub_agg.publish(js)

        # Fake /state_estimate: [lin_vel(3), ang_vel(3), proj_gravity(3), health(1)]
        # proj_gravity = [0, 0, -1] → level body, no roll/pitch
        est = Float32MultiArray()
        est.data = [
            0.0, 0.0, 0.0,   # lin_vel (body frame)
            0.0, 0.0, 0.0,   # ang_vel (body frame)
            0.0, 0.0, -1.0,  # projected_gravity (level)
            1.0,             # health flag
        ]
        self._pub_est.publish(est)


def main() -> None:
    rclpy.init()
    node = MPCPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
