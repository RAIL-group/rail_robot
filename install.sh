#!/bin/bash

set -e # Exit if a commands exists with a non-zero status

# Install ros2 packages
sudo apt update
sudo apt install -y \
    openssh-server \
    ros-humble-kobuki-ros-interfaces \
    ros-humble-kobuki-velocity-smoother \
    ros-humble-sophus \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# clone required submodules from .gitmodules file
git submodule update --init --recursive

# Install dependencies
rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y

# Copy udev rules
sudo cp ./udev/* /etc/udev/rules.d
sudo service udev reload
sudo service udev restart

# Install
colcon build --symlink-install
echo "Installation complete"
