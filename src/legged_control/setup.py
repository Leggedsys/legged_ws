from setuptools import setup, find_packages

package_name = 'legged_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot.yaml']),
        ('share/' + package_name + '/launch', ['launch/locomotion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'joint_aggregator = legged_control.joint_aggregator:main',
            'teleop_node      = legged_control.teleop_node:main',
            'policy_node      = legged_control.policy_node:main',
        ],
    },
)
