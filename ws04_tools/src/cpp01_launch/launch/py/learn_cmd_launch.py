from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
# from ament_index_python.packages import get_package_share_directory
# import os.path

"""
需求: 启动turtlesim节点, 并调用指令打印乌龟的位姿信息
"""

def generate_launch_description():
    turtle1 = Node(package="turtlesim", executable="turtlesim_node",
    respawn=True
    )
    
    cmd = ExecuteProcess(
        # cmd=["ros2 topic echo /turtle1/pose"],
        #Create a FindExecutable substitution.
        cmd = [FindExecutable(name="ros2"),"topic", "echo", "turtle1/pose"],
        output="both",
        shell=True,
    )
    
    return LaunchDescription([turtle1, cmd])