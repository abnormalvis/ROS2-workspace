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
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration(
        'configuration_directory',
        default=os.path.join(slam_demo_dir, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='cartographer_config.lua'
    )

    # 声明launch参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the occupancy grid'
    )

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period'
    )

    declare_configuration_directory = DeclareLaunchArgument(
        'configuration_directory',
        default_value=os.path.join(slam_demo_dir, 'config'),
        description='Configuration directory containing cartographer configuration files'
    )

    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer_config.lua',
        description='Configuration basename for cartographer'
    )

    # 创建cartographer节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ]
    )

    # 创建occupancy grid节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }]
    )

    # 创建rviz节点
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(slam_demo_dir, 'rviz', 'cartographer_demo.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 创建并返回launch描述
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_resolution)
    ld.add_action(declare_publish_period_sec)
    ld.add_action(declare_configuration_directory)
    ld.add_action(declare_configuration_basename)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(start_rviz_cmd)

    return ld