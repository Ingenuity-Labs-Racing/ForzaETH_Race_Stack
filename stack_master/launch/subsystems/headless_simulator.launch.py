"""Launch the existing simulator without RViz, xacro, or GUI dependencies."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from PIL import Image


def setup(context):
    stack = Path(get_package_share_directory('stack_master'))
    map_path = Path(LaunchConfiguration('map_yaml_path').perform(context))
    # The existing simulator requires a 2D grayscale map. Do not alter map assets.
    with Image.open(map_path.with_suffix('.png')) as image:
        if image.mode not in ('1', 'L', 'I', 'F'):
            raise RuntimeError('Headless simulator needs a grayscale map; use glc_ot_ez or fix the map loader separately.')
    bridge = Node(package='f1tenth_gym_ros', executable='gym_bridge', name='bridge',
                  parameters=[str(stack / 'config/SIM/sim.yaml'),
                              {'map_path': str(map_path), 'sim_params': str(stack / 'config/SIM/sim_params.yaml'),
                               'num_agent': 1, 'kb_teleop': False}], output='screen')
    map_server = Node(package='nav2_map_server', executable='map_server', name='map_server',
                      parameters=[{'yaml_filename': str(map_path), 'topic': 'map',
                                   'frame_id': 'map', 'use_sim_time': False}], output='screen')
    lifecycle = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_localization',
                     parameters=[{'autostart': True, 'node_names': ['map_server'],
                                  'use_sim_time': False}], output='screen')
    return [RegisterEventHandler(OnProcessExit(target_action=bridge,
                on_exit=[EmitEvent(event=Shutdown(reason='Simulator bridge exited'))])),
            map_server, lifecycle, bridge]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_yaml_path'), OpaqueFunction(function=setup)])
