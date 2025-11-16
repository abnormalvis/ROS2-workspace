from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 启动仿真环境 ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=...
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
    this_pkg_path = get_package_share_directory("ignition_sim_pkg")
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_path, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": "-v 4 -r " + os.path.join(this_pkg_path, "world", "visualize_lidar.sdf")
            }.items()
        )
    
    # 2.rviz2启动
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(this_pkg_path, "rviz", "sim.rviz")],
    )
    
    # 3.桥接
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",#速度接口
            "/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",#里程计接口
            "/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",#坐标系变换接口
        ],
        remappings=[
            ("/model/vehicle_blue/tf", "/tf"),
            ("/model/vehicle_blue/cmd_vel", "/cmd_vel")
        ]
    )
    return LaunchDescription([gz_sim, rviz2_node, bridge])
