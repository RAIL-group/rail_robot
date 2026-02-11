# ROS2 package for RAIL Group's LoCoBot

ROS 2 Humble package for the RAIL Group's LoCoBot. This project provides a complete software stack for a mobile robot based on the Kobuki base, equipped with an Intel RealSense camera, RPLidar, and a custom pan-and-tilt camera tower.

## System Requirements

- **OS:** Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 2:** Humble Hawksbill
- **Hardware (Optional for simulation):**
  - Kobuki Base
  - RPLidar A1/A2
  - Intel RealSense D435/D455

## Installation

1. **Clone the repository with submodules:**
   ```bash
   git clone --recursive https://github.com/RAIL-group/rail_robot.git
   cd rail_robot
   ```

2. **Run the installation script:**
   This script installs ROS 2 dependencies, initializes `rosdep`, sets up `udev` rules, and builds the workspace.
   ```bash
   chmod +x install.sh
   ./install.sh
   ```

3. **Source the workspace:**
   The installation script adds the necessary source commands to your `~/.bashrc`. Simply restart your terminal or run:
   ```bash
   source ~/.bashrc
   ```

## Quick Start

The most convenient way to launch the robot is using the `rail_robot_core.launch.py` file, which can handle simulation, hardware, SLAM, and Navigation.

### 1. Launch on Physical Hardware
```bash
ros2 launch rail_robot rail_robot_core.launch.py
```

### 2. Launch in Simulation (Gazebo)
```bash
ros2 launch rail_robot rail_robot_core.launch.py hardware_type:=gz_classic
```

## Detailed Usage

### Rviz Visualization
To visualize the robot and its sensor data in Rviz:
```bash
ros2 launch rail_robot rail_robot_rviz.launch.py
```

### Hardware Bringup
To launch the physical robot drivers (Kobuki, Lidar, Camera):
```bash
ros2 launch rail_robot rail_robot_hardware.launch.py
```
*Arguments:*
- `use_camera`: Whether to launch the RealSense camera (default: `true`).

### Simulation (Gazebo)
To launch only the simulation environment:
```bash
ros2 launch rail_robot rail_robot_simulation.launch.py
```
*Arguments:*
- `world_filepath`: Path to the Gazebo world file (default: `floor.world`).
- `robot_name`: Namespace for the robot (default: `robot`).

### SLAM and Navigation
- **SLAM (Mapping):**
  ```bash
  ros2 launch rail_robot rail_robot_slam.launch.py
  ```
- **Navigation:**
  ```bash
  ros2 launch rail_robot rail_robot_navigation.launch.py
  ```

## Package Overview

- **`rail_robot`**: Main integration package containing URDF models, launch files, and configurations.
- **`rail_msgs`**: Custom message and service definitions (e.g., `SaveLocation`, `GoToLocation`).
- **`pano_camera`**: Package for panoramic camera processing and imaging.
- **`planner`**: High-level planning utilities and scripts.
- **`kobuki`**: (Submodule) Drivers and nodes for the Kobuki base.
- **`realsense-ros`**: (Submodule) ROS 2 wrapper for Intel RealSense cameras.
- **`sllidar_ros2`**: (Submodule) ROS 2 driver for RPLidar.
