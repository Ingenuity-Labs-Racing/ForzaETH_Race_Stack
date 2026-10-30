"""One command for a headless two-lap AGMPC, NMPC or MAP experiment."""
from pathlib import Path
from core.controller_registry import get_controller_spec
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def default_source_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / '.gitmodules').is_file() and (parent / 'controller/controller').is_dir():
            return str(parent)
    return str(Path.home() / 'ws/src/race_stack')


def setup(context):
    root = Path(LaunchConfiguration('source_root').perform(context)).resolve()
    requested = LaunchConfiguration('evaluation_config').perform(context)
    config_path = Path(requested) if requested else root / 'controller/evaluation/tuning_params.yaml'
    if not config_path.is_absolute():
        config_path = root / config_path
    with config_path.open(encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or cfg.get('laps') != 2:
        raise RuntimeError('Evaluation requires laps: 2')
    spec = get_controller_spec(cfg.get('controller'))
    if 'GB_TRACK' not in spec.supported_behaviors:
        raise RuntimeError('Selected controller does not support global tracking')
    controller = spec.name
    map_name = cfg['map_name']
    if Path(map_name).name != map_name or not map_name:
        raise RuntimeError('map_name must be a map directory name')
    parameters = spec.parameters(cfg)
    controller_path = config_path
    if spec.lookup_table:
        table = parameters.get('lookup_table', 'NUC2_hangar_pacejka')
        if not isinstance(table, str) or not table or Path(table).name != table:
            raise RuntimeError('lookup_table must be a lookup table name')
        table_path = Path(get_package_share_directory('steering_lookup')) / 'cfg' / (table + '_lookup_table.csv')
        if not table_path.is_file():
            raise RuntimeError(f'Missing MAP steering lookup table: {table_path}')
    stack = Path(get_package_share_directory('stack_master'))
    base = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(stack / 'launch/base_system_launch.xml')),
        launch_arguments={'sim': 'true', 'racecar_version': 'SIM',
                          'map_name': map_name, 'visualize': 'false'}.items())
    trial = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(stack / 'launch/time_trials_launch.xml')),
        launch_arguments={'racecar_version': 'SIM', 'ctrl_algo': controller,
                          'controller_config': str(controller_path),
                          'LU_table': parameters.get('lookup_table', 'NUC2_hangar_pacejka'),
                          'initial_state': 'GB_TRACK',
                          'evaluation_mode': 'true', 'test_on_car': 'false'}.items())
    runner = Node(package='controller', executable='controller_evaluation',
                  name='controller_evaluation', output='screen',
                  parameters=[{'source_root': str(root), 'evaluation_config': str(config_path),
                               'output_directory': LaunchConfiguration('output_directory').perform(context)}])
    return [RegisterEventHandler(OnProcessExit(target_action=runner,
                on_exit=[EmitEvent(event=Shutdown(reason='Controller evaluation finished'))])),
            base, trial, runner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_root', default_value=default_source_root()),
        DeclareLaunchArgument('evaluation_config', default_value=''),
        DeclareLaunchArgument('output_directory', default_value=''),
        OpaqueFunction(function=setup)])
