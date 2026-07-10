"""Stub ROS-only modules so pure node logic is importable without a ROS install.

The legged_control nodes import rclpy / ament_index_python / message packages at
module load time. On a dev machine without ROS sourced these imports fail, which
would block unit-testing the pure helper functions (obs assembly, action decode,
kinematics, etc.). We register lightweight stand-ins in sys.modules before the
test modules import the production code. Only import-time symbols are stubbed;
tests exercise the real pure-Python logic.
"""

import sys
import types
from unittest.mock import MagicMock


def _module(name: str) -> types.ModuleType:
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


def _install_ros_stubs() -> None:
    rclpy = _module("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.ok = lambda *a, **k: False
    rclpy.shutdown = lambda *a, **k: None
    _module("rclpy.parameter")

    node_mod = _module("rclpy.node")

    class _Node:  # real class so `class Foo(Node)` works at import time
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, _name):  # create_publisher/subscription/timer/get_logger/...
            return lambda *a, **k: MagicMock()

    node_mod.Node = _Node

    _module("ament_index_python")
    ament_pkg = _module("ament_index_python.packages")
    ament_pkg.get_package_share_directory = lambda *a, **k: "/nonexistent"

    msg_attrs = {
        "geometry_msgs": [],
        "geometry_msgs.msg": ["Twist"],
        "sensor_msgs": [],
        "sensor_msgs.msg": ["JointState", "Joy", "Imu"],
        "nav_msgs": [],
        "nav_msgs.msg": ["Odometry"],
        "std_msgs": [],
        "std_msgs.msg": ["Bool", "Float32", "Float32MultiArray", "Int8"],
        "rcl_interfaces": [],
        "rcl_interfaces.msg": [
            "Parameter",
            "ParameterType",
            "ParameterValue",
            "SetParametersResult",
        ],
        "rcl_interfaces.srv": ["SetParameters"],
    }
    for mod_name, attrs in msg_attrs.items():
        mod = _module(mod_name)
        for attr in attrs:
            setattr(mod, attr, type(attr, (), {}))


_install_ros_stubs()
