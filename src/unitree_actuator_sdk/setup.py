from pathlib import Path
import platform

from setuptools import Extension, setup

package_name = 'unitree_actuator_sdk'
python_package = 'unitree_motor_ros2'
base_dir = Path(__file__).resolve().parent
native_dir = base_dir / python_package / 'native'
vendor_dir = base_dir / python_package / 'vendor'


def unitree_sdk_library() -> str:
    machine = platform.machine().lower()
    if machine in ('x86_64', 'amd64'):
        return 'UnitreeMotorSDK_Linux64'
    if machine in ('aarch64', 'arm64'):
        return 'UnitreeMotorSDK_Arm64'
    raise RuntimeError(f'Unsupported architecture: {platform.machine()}')


ext_modules = [
    Extension(
        f'{python_package}._unitree_actuator_sdk',
        sources=[str(vendor_dir / 'wrapper.cpp')],
        include_dirs=[str(vendor_dir / 'include'), '/usr/include'],
        library_dirs=[str(native_dir)],
        libraries=[unitree_sdk_library()],
        language='c++',
        extra_compile_args=['-O3', '-std=c++14'],
        extra_link_args=['-Wl,-rpath,$ORIGIN/native'],
    )
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[python_package],
    package_data={
        python_package: [
            'native/libUnitreeMotorSDK_*.so',
        ],
    },
    ext_modules=ext_modules,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Unitree Robotics',
    maintainer_email='support@unitree.com',
    description='ROS 2 Python node package for the prebuilt Unitree actuator SDK.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'go_m8010_6_node = unitree_motor_ros2.go_m8010_6_node:main',
        ],
    },
)
