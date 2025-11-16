#!/bin/bash

# ROS2 SLAM Dependencies Installation Script
# This script installs all required dependencies for the slam_demo package

echo "=== ROS2 SLAM Dependencies Installation ==="
echo "Make sure you have sudo privileges before continuing"

# Check ROS2 version
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        export ROS_DISTRO=humble
    else
        echo "ERROR: Could not detect ROS2 version. Please make sure ROS2 is installed."
        exit 1
    fi
fi

echo "Detected ROS2 distribution: $ROS_DISTRO"

# Update package index
echo "Updating package index..."
sudo apt update

# Install priority SLAM dependencies
echo "Installing SLAM Toolbox..."
sudo apt install -y ros-$ROS_DISTRO-slam-toolbox

echo "Installing Cartographer ROS..."
sudo apt install -y ros-$ROS_DISTRO-cartographer-ros ros-$ROS_DISTRO-cartographer-ros-msgs

# Install Navigation2 dependencies (often used with SLAM)
echo "Installing Navigation2..."
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-nav2-msgs

# Install visualization and TF2 dependencies
echo "Installing visualization and TF2 packages..."
sudo apt install -y ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-tf2-tools

# Install common robot simulation packages that work well with SLAM
echo "Installing robot simulation packages..."
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-gazebo ros-$ROS_DISTRO-turtlebot3-description

# Install geometry and sensor dependencies that might be missing
echo "Installing geometry and sensor packages..."
sudo apt install -y ros-$ROS_DISTRO-geometry2 ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-nav-msgs

# Install launch dependencies
echo "Installing launch and core packages..."
sudo apt install -y ros-$ROS_DISTRO-launch-ros ros-$ROS_DISTRO-rclpy

echo "=== Installation Complete! ==="
echo ""
echo "To verify installation, try the following commands:"
echo "  ros2 pkg list | grep slam-toolbox"
echo "  ros2 pkg list | grep cartographer"
echo "  ros2 pkg list | grep navigation2"
echo ""
echo "Then build your workspace:"
echo "  cd ~/ROS2_Learning/ws09_slam"
echo "  colcon build"
echo "  source install/setup.bash"