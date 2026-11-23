"""
Starts Gazebo with the track world, spawns the car and bridges its topics to ROS.

Default spawn pose is the start of the Victoria centerline (from global_waypoints.json).

odom_noise:=<level> (default 0 = perfect odometry) corrupts the odometry the localizers get:
Gazebo's exact odometry goes to /odom_true and the odom_noise node publishes a drifting /odom
and odom->base_link instead (level 1 = roughly a real car, see odom_noise.py). odom_seed picks
the random part. /ground_truth always stays exact.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from slam_toolbox_sandbox.instance_guard import SIM, fail_if_running


def launch_setup(context):
    fail_if_running('sim', SIM)
    pkg = get_package_share_directory('slam_toolbox_sandbox')
    world = os.path.join(pkg, 'worlds', LaunchConfiguration('world').perform(context) + '.sdf')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    foxglove = LaunchConfiguration('foxglove').perform(context).lower() == 'true'
    noise = float(LaunchConfiguration('odom_noise').perform(context))

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

    # [ : Gazebo -> ROS,  ] : ROS -> Gazebo. With odometry noise, the exact odometry and its TF are
    # moved aside (/odom_true, /odom_true/tf) and odom_noise publishes /odom and odom->base_link.
    remap = [('/odom', '/odom_true'), ('/model/car/tf', '/odom_true/tf')] if noise > 0 else [('/model/car/tf', '/tf')]
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/wheel_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        remappings=remap,
        parameters=[{'use_sim_time': True}],
    )

    # Must match the lidar <pose> in models/car.sdf (relative to the model origin)
    laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.1', '--z', '0.14', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        parameters=[{'use_sim_time': True}],
    )

    # Ground truth is in the Gazebo world frame. The odometry starts in world coordinates,
    # so the SLAM map frame coincides with it - this links the two for visualization.
    world_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}],
    )

    actions = [gazebo, TimerAction(period=3.0, actions=[spawn]), bridge, laser_tf, world_tf]
    if noise > 0:
        actions.append(Node(
            package='slam_toolbox_sandbox', executable='odom_noise', output='screen',
            parameters=[{'level': noise, 'seed': int(LaunchConfiguration('odom_seed').perform(context)),
                         'use_sim_time': True}],
        ))
    if foxglove or rviz:
        # Draws the car at its true pose on /car_marker (only needed when something displays it)
        actions.append(Node(
            package='slam_toolbox_sandbox', executable='car_marker', output='screen',
            parameters=[{'scale': float(LaunchConfiguration('car_scale').perform(context)),
                         'use_sim_time': True}],
        ))
    if foxglove:
        # Streams all topics to the Foxglove app on the host: ws://localhost:8765
        actions.append(Node(
            package='foxglove_bridge', executable='foxglove_bridge', output='screen',
            parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': True}],
        ))
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
        DeclareLaunchArgument('rviz', default_value='false', description='RViz in the VNC display (fallback for Foxglove)'),
        DeclareLaunchArgument('foxglove', default_value='true', description='bridge for the Foxglove app: ws://localhost:8765'),
        DeclareLaunchArgument('car_scale', default_value='1.0', description='size of the drawn car, 1.0 = true size'),
        DeclareLaunchArgument('odom_noise', default_value='0',
                              description='odometry noise level: 0 = perfect, 1 = roughly a real car, 2 = double, ...'),
        DeclareLaunchArgument('odom_seed', default_value='1', description='seed of the random part of the odometry noise'),
        OpaqueFunction(function=launch_setup),
    ])
