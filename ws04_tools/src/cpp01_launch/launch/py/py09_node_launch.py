"""
需求: 演示Node的使用。

构造函数参数说明：
    - package 被执行的程序所属的功能包
    - executable 可执行程序
    - name 节点名称
    - namespace 设置命名空间
    - exec_name 设置程序标签
    - parameters 设置参数
    - remappings 实现话题重映射
    - ros_arguments 为节点传参
    - arguments 为节点传参
    - respawn 是否自动重启
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
    turtle1 = Node(
        package="turtlesim",       
        executable="turtlesim_node",
        exec_name="t2_label",
        ros_arguments=["--remap", "__ns:=/t2"]
    )
    
    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="haha",
        parameters=[
            os.path.join(
                get_package_share_directory("cpp01_launch"), "config", "haha.yaml"
            )
        ],
        # parameters=["background_r": 255, "background_g": 255, "background_b": 255]
    )
    
    return LaunchDescription([turtle1, turtle2])