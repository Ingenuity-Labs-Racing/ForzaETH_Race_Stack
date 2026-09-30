"""
Runs SLAM Toolbox in mapping mode. Start sim.launch.py first.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('slam_toolbox_sandbox')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(pkg, 'config', 'mapping.yaml')),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
            parameters=[LaunchConfiguration('params'), {'use_sim_time': True}],
        ),
    ])
