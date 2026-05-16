"""height_scan_node

Subscribes:
  /camera/depth/image_rect_raw  (sensor_msgs/Image, 16UC1, mm units)
  /camera/depth/camera_info     (sensor_msgs/CameraInfo)

Publishes:
  /height_scan  (std_msgs/Float32MultiArray, 325 floats)

Height scan grid (in base_link frame):
  x: [0.10, 1.30] m (25 cols, step 0.05 m, positive = forward)
  y: [-0.30, 0.30] m (13 rows, step 0.05 m, positive = left)
  index = y_idx * 25 + x_idx  (x-major)
  value = -terrain_z_in_base_link, clipped to [-1, 1]
    positive = ground (normal stance), negative = raised obstacle

Requires a static TF base_link -> camera_link.  During initial hardware
integration, confirm the TF is published by the launch file.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
import struct
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 — needed to register PointStamped transforms

_X_MIN, _X_MAX = 0.10, 1.30
_Y_MIN, _Y_MAX = -0.30, 0.30
_N_X, _N_Y = 25, 13
_RES = 0.05
_N_CELLS = _N_X * _N_Y  # 325

# Default fallback for empty grid cells — loaded from robot.yaml at init
_NOMINAL_HEIGHT = 0.30


def _build_height_scan(points_base_link: np.ndarray) -> np.ndarray:
    grid = np.full(_N_CELLS, np.nan, dtype=np.float32)

    if len(points_base_link) == 0:
        return np.zeros(_N_CELLS, dtype=np.float32)

    xs, ys, zs = points_base_link[:, 0], points_base_link[:, 1], points_base_link[:, 2]

    mask = (xs >= _X_MIN) & (xs <= _X_MAX) & (ys >= _Y_MIN) & (ys <= _Y_MAX)
    xs, ys, zs = xs[mask], ys[mask], zs[mask]

    if len(xs) == 0:
        return np.zeros(_N_CELLS, dtype=np.float32)

    xi = np.clip(np.round((xs - _X_MIN) / _RES).astype(int), 0, _N_X - 1)
    yi = np.clip(np.round((ys - _Y_MIN) / _RES).astype(int), 0, _N_Y - 1)
    idx = yi * _N_X + xi

    for i, z in zip(idx, zs):
        if np.isnan(grid[i]) or z > grid[i]:
            grid[i] = z

    result = np.where(
        np.isnan(grid),
        _NOMINAL_HEIGHT,  # no data → assume flat ground at stance height
        np.where(grid > 0.0, -1.0, np.clip(-grid, -1.0, 1.0)),
    )

    return result.astype(np.float32)


def _deproject_pixel(
    u: np.ndarray, v: np.ndarray, z_m: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
) -> np.ndarray:
    x = (u - cx) * z_m / fx
    y = (v - cy) * z_m / fy
    return np.stack([x, y, z_m], axis=1)


class HeightScanNode(Node):
    def __init__(self) -> None:
        super().__init__("height_scan_node")
        self._load_nominal()
        self._fx = self._fy = self._cx = self._cy = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Float32MultiArray, "/height_scan", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "/height_scan_cloud", 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self._on_info, 1)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._on_depth, 10)
        self.get_logger().info("height_scan_node ready — waiting for camera_info")

    def _load_nominal(self) -> None:
        global _NOMINAL_HEIGHT
        try:
            share = get_package_share_directory("legged_control")
            with open(os.path.join(share, "config", "robot.yaml")) as f:
                cfg = yaml.safe_load(f)
            hs_cfg = cfg.get("height_scan", {})
            _NOMINAL_HEIGHT = float(hs_cfg.get("default_height", 0.30))
        except Exception:
            pass

    def _on_info(self, msg: CameraInfo) -> None:
        if self._fx is None:
            K = msg.k
            self._fx, self._fy, self._cx, self._cy = K[0], K[4], K[2], K[5]
            self.get_logger().info(
                f"camera_info received: fx={self._fx:.1f} fy={self._fy:.1f}"
            )

    def _pub_cloud(self, hs: np.ndarray, depth_msg: Image) -> None:
        points = []
        for yi in range(_N_Y):
            for xi in range(_N_X):
                idx = yi * _N_X + xi
                x = _X_MIN + xi * _RES
                y = _Y_MIN + yi * _RES
                z = -float(hs[idx])  # terrain_z = -hs_value
                points.append((x, y, z))

        cloud = PointCloud2()
        cloud.header.stamp = depth_msg.header.stamp
        cloud.header.frame_id = "base_link"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_dense = True
        cloud.data = bytes(b"".join(
            struct.pack("fff", x, y, z) for x, y, z in points
        ))
        self._cloud_pub.publish(cloud)

    def _on_depth(self, msg: Image) -> None:
        if self._fx is None:
            return

        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        z_mm = depth.astype(np.float32)
        valid = z_mm > 0
        vs, us = np.where(valid)
        z_m = z_mm[valid] / 1000.0

        pts_cam = _deproject_pixel(
            us.astype(np.float32), vs.astype(np.float32), z_m,
            self._fx, self._fy, self._cx, self._cy,
        )

        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, rclpy.time.Time()
            )
        except Exception:
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        translation = np.array([t.x, t.y, t.z])
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
        ])
        pts_base = (R @ pts_cam.T).T + translation

        hs = _build_height_scan(pts_base.astype(np.float32))
        out = Float32MultiArray()
        out.data = hs.tolist()
        self._pub.publish(out)
        self._pub_cloud(hs, msg)


def main() -> None:
    rclpy.init()
    node = HeightScanNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except RuntimeError:
                # rclpy/TF2 deserialization bug on some Gazebo TF messages
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
