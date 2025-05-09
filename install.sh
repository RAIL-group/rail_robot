#!/bin/bash

set -e # Exit if a commands exists with a non-zero status

# Install ros2 packages
sudo apt update
sudo apt install -y \
    ros-dev-tools \
    ros-humble-ros2-control \
    ros-humble-kobuki-ros-interfaces \
    ros-humble-kobuki-velocity-smoother \
    ros-humble-sophus \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp

# clone required submodules from .gitmodules file
git submodule update --init --recursive

# Install dependencies
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep is not initialized. Initializing rosdep..."
    sudo rosdep init
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

if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -Fxq "source ~/rail_robot/install/setup.bash" ~/.bashrc; then
    echo "source ~/rail_robot/install/setup.bash" >> ~/.bashrc
fi

if ! grep -Fxq "export ROS_DOMAIN_ID=42" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo 'export ROS_IP=$(hostname -I | cut -d " " -f1)' >> ~/.bashrc
    echo 'export ROS_HOSTNAME=$(hostname -I | cut -d " " -f1)' >> ~/.bashrc
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
fi

echo "Installation complete. Use 'source ~/.bashrc' to source the rail_robot package"
