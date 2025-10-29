"""
需求: 在launch文件启动时动态地设置turtlesim_node的背景色
实现：
    1.声明参数（变量）
    2.调用参数（变量）
    3.执行launch文件时动态导入参数
终端调用: ros2 launch cpp01_launch py04_args_launch.py back_r:=0
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """
    生成launch描述文件
    """
    # 1.声明参数（变量）
    bg_r = DeclareLaunchArgument("back_r", default_value="255")

    bg_g = DeclareLaunchArgument("back_g", default_value="125")
    
    bg_b = DeclareLaunchArgument("back_b", default_value="0")
    
    # 2.调用参数（变量）
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[{"background_r": LaunchConfiguration("back_r")}, {"background_g": LaunchConfiguration("back_g")}, {"background_b": LaunchConfiguration("back_b")}],
        respawn=True,
    )
    
    return LaunchDescription([bg_r, bg_g, bg_b, turtle])