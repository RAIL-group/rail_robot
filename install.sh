#!/bin/bash

set -e # Exit if a commands exists with a non-zero status

# Install ros2 packages
sudo apt update
sudo apt install -y \
    openssh-server \
    ros-humble-ros2-control \
    ros-humble-kobuki-ros-interfaces \
    ros-humble-kobuki-velocity-smoother \
    ros-humble-sophus \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \

# clone required submodules from .gitmodules file
git submodule update --init --recursive

# Install dependencies
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep is not initialized. Initializing rosdep..."
    rosdep init
else
    echo "rosdep is already initialized."
fi
rosdep update
rosdep install -i --from-path src --rosdistro humble -y

# Copy udev rules
sudo cp ./udev/* /etc/udev/rules.d
sudo service udev reload
sudo service udev restart

# Install
colcon build --symlink-install
echo "Installation complete"
