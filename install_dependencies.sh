#!/bin/bash

# ROS2 Localization Workspace - Dependencies Installation Script
# This script installs packages that are not included in ROS2 Humble desktop

set -e  # Exit on any error

echo " Installing ROS2 Localization Workspace Dependencies..."
echo "=================================================="

# Update package list
echo " Updating package list..."
sudo apt update

# ROS2 bag recording tools
echo " Installing ROS2 bag recording tools..."
sudo apt install -y \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-storage-sqlite3 \
    ros-humble-rosbag2-compression \
    ros-humble-rosbag2-compression-zstd

# Robot localization (EKF)
echo " Installing robot localization..."
sudo apt install -y ros-humble-robot-localization

# Serial communication for IMU
echo " Installing serial communication tools..."
sudo apt install -y python3-serial

# Build tools (if not already installed)
echo " Installing build tools..."
sudo apt install -y python3-colcon-common-extensions

echo ""
echo " All dependencies installed successfully!"
echo ""
echo " Next steps:"
echo "1. Build the workspace: colcon build"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Launch the system: ros2 launch gnss_imu_robot_localization bringup.launch.py"
echo ""
echo " For data recording, use:"
echo "   ros2 launch gnss_imu_robot_localization bringup.launch.py enable_bag_recording:=true"
echo ""
