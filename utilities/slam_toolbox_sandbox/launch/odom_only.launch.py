"""
"Odometry only": no localizer, the reference for the odometry noise test.

map -> odom is fixed at identity (the odometry starts in world = map coordinates), so the
estimate pose_error measures is the odometry itself, and its error is how far the odometry has
drifted. With perfect odometry (sim.launch.py odom_noise:=0) it scores about 0. Start a fresh
sim.launch.py first.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from slam_toolbox_sandbox.instance_guard import LOCALIZERS, fail_if_running


def launch_setup(context):
    fail_if_running('localizer', LOCALIZERS)
    map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
    )
    pose_error = Node(
        package='slam_toolbox_sandbox', executable='pose_error', output='screen',
        parameters=[{'use_sim_time': True, 'csv_path': LaunchConfiguration('csv').perform(context)}],
    )
    return [map_to_odom, pose_error]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('csv', default_value='', description='optional path to log errors as CSV'),
        OpaqueFunction(function=launch_setup),
    ])
