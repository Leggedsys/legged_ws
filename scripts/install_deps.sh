#!/bin/bash
set -e

echo "=== installing system dependencies ==="
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-imu-filter-madgwick \
    ros-humble-joy \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers

echo "=== setting up udev rules ==="
sudo usermod -a -G dialout $USER

sudo tee /etc/udev/rules.d/99-odin-usb.rules << 'UEOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
UEOF
sudo udevadm control --reload && sudo udevadm trigger

echo "=== done. re-login or reboot for group changes ==="
