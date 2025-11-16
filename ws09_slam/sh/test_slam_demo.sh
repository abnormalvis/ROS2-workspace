#!/bin/bash

# SLAM Demo Package Test Script
# Tests the installation and functionality of slam_demo package

echo "=== Testing SLAM Demo Package ==="

# Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local workspace
if [ -f /home/idris/ROS2_Learning/ws09_slam/install/setup.bash ]; then
    source /home/idris/ROS2_Learning/ws09_slam/install/setup.bash
fi

echo "1. Checking if slam_demo package is available..."
ros2 pkg list | grep slam_demo
if [ $? -eq 0 ]; then
    echo "✓ slam_demo package is installed"
else
    echo "✗ slam_demo package is NOT installed"
    exit 1
fi

echo ""
echo "2. Checking launch files..."
ls /home/idris/ROS2_Learning/ws09_slam/install/slam_demo/share/slam_demo/launch/
if [ $? -eq 0 ]; then
    echo "✓ Launch files are installed"
else
    echo "✗ Launch files are missing"
    exit 1
fi

echo ""
echo "3. Checking config files..."
ls /home/idris/ROS2_Learning/ws09_slam/install/slam_demo/share/slam_demo/config/
if [ $? -eq 0 ]; then
    echo "✓ Config files are installed"
else
    echo "✗ Config files are missing"
    exit 1
fi

echo ""
echo "4. Checking RViz files..."
ls /home/idris/ROS2_Learning/ws09_slam/install/slam_demo/share/slam_demo/rviz/
if [ $? -eq 0 ]; then
    echo "✓ RViz config files are installed"
else
    echo "✗ RViz config files are missing"
    exit 1
fi

echo ""
echo "5. Testing launch file syntax..."
echo "Testing slam_toolbox_demo.launch.py..."
ros2 launch --show-args slam_demo slam_toolbox_demo.launch.py > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ slam_toolbox launch file is valid"
else
    echo "✗ slam_toolbox launch file has syntax errors"
    exit 1
fi

echo "Testing cartographer_demo.launch.py..."
ros2 launch --show-args slam_demo cartographer_demo.launch.py > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ cartographer launch file is valid"
else
    echo "✗ cartographer launch file has syntax errors"
    exit 1
fi

echo ""
echo "6. Showing launch file parameters..."
echo "=== SLAM Toolbox Parameters ==="
ros2 launch --show-args slam_demo slam_toolbox_demo.launch.py

echo ""
echo "=== Cartographer Parameters ==="
ros2 launch --show-args slam_demo cartographer_demo.launch.py

echo ""
echo "=== All Tests Passed! ==="
echo "The slam_demo package has been successfully installed and configured."
echo ""
echo "To use this package:"
echo "1. Update your ROS2 environment: source /opt/ros/humble/setup.bash"
echo "2. Source your workspace: source /home/idris/ROS2_Learning/ws09_slam/install/setup.bash"
echo "3. Run SLAM Toolbox demo: ros2 launch slam_demo slam_toolbox_demo.launch.py"
echo "4. Run Cartographer demo: ros2 launch slam_demo cartographer_demo.launch.py"
echo ""
echo "Note: Make sure you have the required SLAM dependencies installed:"
echo "- slam-toolbox"
echo "- cartographer-ros"
echo ""
echo "You can install them by running:"
echo "sudo apt install ros-humble-slam-toolbox ros-humble-cartographer-ros"