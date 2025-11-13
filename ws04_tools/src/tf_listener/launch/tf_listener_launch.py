from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package='tf_listener',
        executable='tf_listener_node',
        name='tf_listener',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'child_frame'
        }]
    )

    ld.add_action(node)
    return ld
