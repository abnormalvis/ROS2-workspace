#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取功能包路径
    slam_demo_dir = get_package_share_directory('slam_demo')

    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(slam_demo_dir, 'config', 'slam_toolbox_params.yaml')
    )

    # 声明launch参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_demo_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for slam_toolbox'
    )

    # 创建slam_toolbox节点
    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # 创建rviz节点
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(slam_demo_dir, 'rviz', 'slam_demo.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 创建并返回launch描述
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_params_file)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(start_rviz_cmd)

    return ld