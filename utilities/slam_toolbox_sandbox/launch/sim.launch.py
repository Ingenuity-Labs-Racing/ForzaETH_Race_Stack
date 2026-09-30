"""
Starts Gazebo with the track world, spawns the car and bridges its topics to ROS.

Default spawn pose is the start of the Victoria centerline (from global_waypoints.json).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg = get_package_share_directory('slam_toolbox_sandbox')
    world = os.path.join(pkg, 'worlds', LaunchConfiguration('world').perform(context) + '.sdf')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    # -r: start running immediately, -s: server only (no Gazebo window)
    gz_cmd = ['ign', 'gazebo', '-r', '-v', '3', world]
    if not gui:
        gz_cmd.insert(2, '-s')
    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-world', 'track',
            '-file', os.path.join(pkg, 'models', 'car.sdf'),
            '-name', 'car',
            '-x', LaunchConfiguration('x').perform(context),
            '-y', LaunchConfiguration('y').perform(context),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw').perform(context),
        ],
    )

    # [ : Gazebo -> ROS,  ] : ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        remappings=[('/model/car/tf', '/tf')],
        parameters=[{'use_sim_time': True}],
    )

    # Must match the lidar <pose> in models/car.sdf (relative to the model origin)
    laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.1', '--z', '0.14', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        parameters=[{'use_sim_time': True}],
    )

    actions = [gazebo, TimerAction(period=3.0, actions=[spawn]), bridge, laser_tf]
    if rviz:
        actions.append(Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'sandbox.rviz')],
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='victoria', description='world file name in worlds/ without .sdf'),
        DeclareLaunchArgument('x', default_value='-7.6016'),
        DeclareLaunchArgument('y', default_value='-2.5328'),
        DeclareLaunchArgument('yaw', default_value='-0.0638'),
        DeclareLaunchArgument('gui', default_value='false', description='open the Gazebo window (needs working OpenGL)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
