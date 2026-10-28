# AutoDRIVE simulator counterpart of f1tenth_stack/launch/bringup_launch.py:
# the vesc and urg_node drivers are replaced by the AutoDRIVE bridge and the autodrive_adapter node,
# joystick, ackermann mux and static transforms are the same as on the car.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    adapter_config = os.path.join(
        get_package_share_directory('autodrive_adapter'),
        'config',
        'autodrive_adapter.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    map_path_la = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Stack map directory whose racetrack is built in the simulator, empty keeps the built-in track')

    ld = LaunchDescription([joy_la, mux_la, map_path_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    autodrive_bridge_node = Node(
        package='autodrive_roboracer',
        executable='autodrive_bridge',
        name='autodrive_bridge',
        parameters=[{'map_path': LaunchConfiguration('map_path')}],
        emulate_tty=True,
        output='screen',
    )
    autodrive_adapter_node = Node(
        package='autodrive_adapter',
        executable='autodrive_adapter_node',
        name='autodrive_adapter',
        parameters=[adapter_config]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')]
    )
    static_tf_node_bl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    static_tf_node_mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    static_tf_node_bi = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.07', '0.0', '0.05', '0.0', '0.0', '0.7071068', '0.7071068', 'base_link', 'imu']
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(autodrive_bridge_node)
    ld.add_action(autodrive_adapter_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node_bl)
    ld.add_action(static_tf_node_mo)
    ld.add_action(static_tf_node_bi)
    return ld
