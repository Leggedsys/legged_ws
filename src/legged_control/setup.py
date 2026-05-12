from setuptools import setup, find_packages

package_name = "legged_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/robot.yaml",
                "config/robot_sim.yaml",
                "config/position_control_sim.rviz",
                "config/passive_mode.rviz",
                "config/policy.yaml",
            ],
        ),
        (
            "share/" + package_name + "/models",
            ["models/policy.pt"],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/robot.launch.py",
                "launch/position_control_sim.launch.py",
                "launch/gazebo_physics.launch.py",
                "launch/gazebo_position_control.launch.py",
                "launch/gazebo_policy.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "passive_monitor_node  = legged_control.passive_monitor_node:main",
            "motor_bus_node        = legged_control.motor_bus_node:main",
            "joint_aggregator      = legged_control.joint_aggregator:main",
            "teleop_node           = legged_control.teleop_node:main",
            "gait_node             = legged_control.gait_node:main",
            "fake_motor_bus_node   = legged_control.fake_motor_bus_node:main",
            "urdf_joint_state_bridge = legged_control.urdf_joint_state_bridge:main",
            "gazebo_control_bridge = legged_control.gazebo_control_bridge:main",
            "state_estimator_node  = legged_control.state_estimator_node:main",
            "height_scan_node      = legged_control.height_scan_node:main",
            "policy_node           = legged_control.policy_node:main",
        ],
    },
)
