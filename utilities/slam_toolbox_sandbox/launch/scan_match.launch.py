"""
Runs the scan-to-map localizer on the fixed ground-truth Victoria map, plus the ground-truth
error node. Start a fresh sim.launch.py first.

The default map is maps/victoria_outline: the walls of stack_master/maps/Victoria (exactly the
wall pixels the Gazebo world was built from) with the rest of the filled-in off-track area
marked unknown, made by outline_map. Its frame is the Gazebo world frame, so the spawn pose is
also the initial pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from slam_toolbox_sandbox.instance_guard import LOCALIZERS, fail_if_running

DEFAULT_MAP = os.path.expanduser('~/ws/src/race_stack/utilities/slam_toolbox_sandbox/maps/victoria_outline.yaml')
VICTORIA_START = ('-7.6016', '-2.5328', '-0.0638')


def launch_setup(context):
    fail_if_running('localizer', LOCALIZERS)
    def num(name):
        return float(LaunchConfiguration(name).perform(context))

    localizer = Node(
        package='slam_toolbox_sandbox', executable='scan_match_localizer', output='screen',
        parameters=[LaunchConfiguration('params').perform(context), {
            'use_sim_time': True,
            'map': LaunchConfiguration('map').perform(context),
            'x': num('x'), 'y': num('y'), 'yaw': num('yaw'),
        }],
    )
    pose_error = Node(
        package='slam_toolbox_sandbox', executable='pose_error', output='screen',
        parameters=[{'use_sim_time': True, 'csv_path': LaunchConfiguration('csv').perform(context)}],
    )
    return [localizer, pose_error]


def generate_launch_description():
    pkg = get_package_share_directory('slam_toolbox_sandbox')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(pkg, 'config', 'scan_match.yaml')),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP, description='map .yaml (map_server format)'),
        DeclareLaunchArgument('x', default_value=VICTORIA_START[0], description='spawn pose used in sim.launch.py'),
        DeclareLaunchArgument('y', default_value=VICTORIA_START[1]),
        DeclareLaunchArgument('yaw', default_value=VICTORIA_START[2]),
        DeclareLaunchArgument('csv', default_value='', description='optional path to log errors as CSV'),
        OpaqueFunction(function=launch_setup),
    ])
