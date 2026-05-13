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
                "config/passive_mode.rviz",
                "config/policy.yaml",
                "config/test.rviz",
            ],
        ),
        (
            "share/" + package_name + "/models",
            ["models/policy.pt"],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/real.launch.py",
                "launch/robot.launch.py",
                "launch/test.launch.py",
                "launch/gazebo_physics.launch.py",
                "launch/gazebo_sim.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "obs_monitor_node      = legged_control.test.obs_monitor_node:main",
            "vel_viz_node          = legged_control.test.vel_viz_node:main",
            "motor_bus_node        = legged_control.real.motor_bus_node:main",
            "joint_aggregator      = legged_control.real.joint_aggregator:main",
            "motor_command_bridge  = legged_control.real.motor_command_bridge:main",
            "teleop_node           = legged_control.processing.teleop_node:main",
            "urdf_joint_state_bridge = legged_control.real.urdf_joint_state_bridge:main",
            "gazebo_control_bridge = legged_control.sim.gazebo_control_bridge:main",
            "state_estimator_node  = legged_control.processing.state_estimator_node:main",
            "height_scan_node      = legged_control.processing.height_scan_node:main",
            "policy_node           = legged_control.policy_node:main",
            "policy_node_sim       = legged_control.sim.policy_node_sim:main",
        ],
    },
)
