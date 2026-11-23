"""
Runs Nav2 AMCL on the ground-truth Victoria map, plus the ground-truth error node.
The comparison to localization.launch.py (SLAM Toolbox). Start a fresh sim.launch.py first.

AMCL reads a plain map image. The default is maps/victoria_outline: the walls of
stack_master/maps/Victoria (exactly the wall pixels the Gazebo world was built from) with the rest
of the filled-in off-track area marked unknown, made by outline_map. On the filled map, a beam
that overshoots a wall still scores as a perfect hit. The map frame is the Gazebo world frame,
so the spawn pose is also AMCL's initial pose and the error node needs no map origin.
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

    params = LaunchConfiguration('params').perform(context)
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[params, {'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map').perform(context)}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params, {
            'use_sim_time': True,
            'set_initial_pose': True,
            'initial_pose.x': num('x'),
            'initial_pose.y': num('y'),
            'initial_pose.z': 0.0,
            'initial_pose.yaw': num('yaw'),
        }],
    )
    # map_server and amcl are lifecycle nodes: this configures and activates them
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
    )
    pose_error = Node(
        package='slam_toolbox_sandbox', executable='pose_error', output='screen',
        parameters=[{'use_sim_time': True, 'csv_path': LaunchConfiguration('csv').perform(context)}],
    )
    return [map_server, amcl, lifecycle, pose_error]


def generate_launch_description():
    pkg = get_package_share_directory('slam_toolbox_sandbox')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(pkg, 'config', 'amcl.yaml')),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP, description='map .yaml (map_server format)'),
        DeclareLaunchArgument('x', default_value=VICTORIA_START[0], description='spawn pose used in sim.launch.py'),
        DeclareLaunchArgument('y', default_value=VICTORIA_START[1]),
        DeclareLaunchArgument('yaw', default_value=VICTORIA_START[2]),
        DeclareLaunchArgument('csv', default_value='', description='optional path to log errors as CSV'),
        OpaqueFunction(function=launch_setup),
    ])
