tf_listener
=============

A small ROS2 C++ package that demonstrates how to listen to TF transforms using tf2_ros::Buffer and tf2_ros::TransformListener.

Files created:
- package.xml: package metadata
- CMakeLists.txt: build rules
- src/tf_listener_node.cpp: example node that periodically prints the transform between two frames
- launch/tf_listener_launch.py: simple launch file

Build
-----
From the workspace root (where you have setup for ROS2 and colcon):

```bash
# source your ROS2 installation, e.g.:
# source /opt/ros/<distro>/setup.bash
# then from workspace root:
colcon build --packages-select tf_listener
```

Run
---

```bash
# source the install overlay
source install/setup.bash
# run the node directly
ros2 run tf_listener tf_listener_node
# or use launch
ros2 launch tf_listener tf_listener_launch.py
```

Notes
-----
- Make sure you have a TF broadcaster publishing the frames you want to listen to (example children/parent frames).
- Adjust `parent_frame` and `child_frame` via node parameters or the launch file.
