"""Launch Ignition Gazebo with an example world and start ros_gz_bridge parameter_bridge."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ignition_sim_pkg')
    world_file = os.path.join(pkg_share, 'world', 'demo.sdf')
    bridge_cfg = os.path.join(pkg_share, 'config', 'bridge.conf')

    # Command to start Ignition Gazebo (ign gazebo). Using -r to run.
    ign_cmd = ['ign', 'gazebo', '-r', world_file]

    # Command to start ros_gz_bridge parameter_bridge with a simple demo mapping file.
    # Note: depending on your ros_gz_bridge version the exact syntax for parameter_bridge
    # may vary. This example uses a single topic mapping defined in bridge.conf.
    bridge_cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--config', bridge_cfg]

    return [
        ExecuteProcess(cmd=ign_cmd, output='screen'),
        ExecuteProcess(cmd=bridge_cmd, output='screen')
    ]


def generate_launch_description():
    ld = LaunchDescription()
    # Use OpaqueFunction so we can resolve package path at launch time
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
