from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            arguments=[os.path.join(get_package_share_directory('stage_ros2'), 'worlds', 'sim.world')]
        )
    ])