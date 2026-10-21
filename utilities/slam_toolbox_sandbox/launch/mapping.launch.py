"""
Runs SLAM Toolbox in mapping mode, plus the ground-truth error node. Start sim.launch.py first.

The odometry starts in Gazebo world coordinates, so the SLAM map frame is the world
frame and the error against ground truth is meaningful while mapping too.
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
        Node(
            package='slam_toolbox_sandbox', executable='pose_error', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
