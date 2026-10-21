"""
Runs SLAM Toolbox in localization mode on a saved map, plus the ground-truth error node.
Start a fresh sim.launch.py first, so the car is at its spawn pose.

Frames: the odometry starts in Gazebo world coordinates, so a map recorded in this
sandbox has map frame = world frame = Victoria map frame, and map_origin_* stays 0.
(Only if odometry ever starts at zero again does map_origin_* need to be the spawn
pose used while mapping.) The car's spawn pose (x, y, yaw) is converted into the
map frame to give SLAM Toolbox its start pose.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP = os.path.expanduser('~/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_gz')
VICTORIA_START = ('-7.6016', '-2.5328', '-0.0638')


def launch_setup(context):
    def num(name):
        return float(LaunchConfiguration(name).perform(context))

    ox, oy, oyaw = num('map_origin_x'), num('map_origin_y'), num('map_origin_yaw')
    x, y, yaw = num('x'), num('y'), num('yaw')
    # spawn pose expressed in the map frame: inverse(origin) * spawn
    dx, dy = x - ox, y - oy
    start = [
        math.cos(oyaw) * dx + math.sin(oyaw) * dy,
        -math.sin(oyaw) * dx + math.cos(oyaw) * dy,
        math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw)),
    ]

    slam = Node(
        package='slam_toolbox', executable='localization_slam_toolbox_node', name='slam_toolbox', output='screen',
        parameters=[
            LaunchConfiguration('params').perform(context),
            {
                'use_sim_time': True,
                'map_file_name': LaunchConfiguration('map').perform(context),
                'map_start_pose': start,
            },
        ],
    )
    pose_error = Node(
        package='slam_toolbox_sandbox', executable='pose_error', output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_origin_x': ox, 'map_origin_y': oy, 'map_origin_yaw': oyaw,
            'csv_path': LaunchConfiguration('csv').perform(context),
        }],
    )
    return [slam, pose_error]


def generate_launch_description():
    pkg = get_package_share_directory('slam_toolbox_sandbox')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(pkg, 'config', 'localization.yaml')),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP, description='serialized map, without file extension'),
        DeclareLaunchArgument('x', default_value=VICTORIA_START[0], description='spawn pose used in sim.launch.py'),
        DeclareLaunchArgument('y', default_value=VICTORIA_START[1]),
        DeclareLaunchArgument('yaw', default_value=VICTORIA_START[2]),
        DeclareLaunchArgument('map_origin_x', default_value='0.0', description='world pose of the SLAM map frame'),
        DeclareLaunchArgument('map_origin_y', default_value='0.0'),
        DeclareLaunchArgument('map_origin_yaw', default_value='0.0'),
        DeclareLaunchArgument('csv', default_value='', description='optional path to log errors as CSV'),
        OpaqueFunction(function=launch_setup),
    ])
