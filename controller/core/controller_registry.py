"""Controller definitions: adding an adapter requires an entry here, not ROS-node edits."""
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path


def resolve(target):
    module, name = target.split(':')
    return getattr(import_module(module), name)


@dataclass(frozen=True)
class ControllerSpec:
    name: str
    factory: str
    loader: str
    config_package: str
    config_path: str
    supported_behaviors: tuple = ('GB_TRACK',)
    required_inputs: tuple = ('pose', 'velocity', 'waypoints', 'state')
    report_capabilities: tuple = ()
    config_parameter: str = ''
    lookup_table: bool = False
    fallback: tuple = ()
    remote_parameters: tuple = ()
    reference_description: str = 'First time-sampled controller reference'
    speed_label: str = 'Reference'
    steering_label: str = 'Reference'
    waypoint_timeout_cycles: int = 0

    def parameters(self, document):
        # Multi-controller documents select one self-contained tuning block.
        if isinstance(document, dict) and 'controllers' in document:
            profiles = document['controllers']
            if not isinstance(profiles, dict) or not isinstance(profiles.get(self.name), dict):
                raise ValueError(f'Missing controllers.{self.name} parameter block')
            document = profiles[self.name]
        return resolve(self.loader)(document)

    def create(self, parameters, context):
        return resolve(self.factory)(parameters, context)

    def default_path(self, package_share, racecar_version):
        return Path(package_share) / self.config_path.format(racecar_version=racecar_version)

    def report_metadata(self):
        return dict(report_capabilities=list(self.report_capabilities),
                    reference_definition=self.reference_description,
                    reference_speed_label=self.speed_label,
                    reference_steering_label=self.steering_label)


REGISTRY = {}
ALIASES = {}


def register(spec, aliases=()):
    names = (spec.name,) + tuple(aliases)
    if any(name.upper() in REGISTRY or name.upper() in ALIASES for name in names):
        raise ValueError('Controller name already registered')
    REGISTRY[spec.name] = spec
    for alias in aliases:
        ALIASES[alias.upper()] = spec.name


def get_controller_spec(name):
    key = str(name).upper()
    key = ALIASES.get(key, key)
    try:
        return REGISTRY[key]
    except KeyError as exc:
        raise ValueError(f'Unknown controller {name!r}; choose from {sorted(REGISTRY)}') from exc


register(ControllerSpec(
    'AGMPC', 'controller.agmpc.adapter:AGMPCAdapter', 'controller.agmpc.adapter:load_config',
    'controller', 'agmpc/agmpc_params.yaml', report_capabilities=('solve_time', 'constraints'),
    config_parameter='agmpc_config'))
register(ControllerSpec(
    'NMPC', 'controller.nmpc.adapter:NMPCAdapter', 'controller.nmpc.adapter:load_config',
    'controller', 'nmpc/nmpc_params.yaml', report_capabilities=('solve_time', 'constraints'),
    config_parameter='nmpc_config'))
register(ControllerSpec(
    'MAP', 'controller.map.adapter:MAPAdapter', 'controller.pursuit:load_config',
    'stack_master', 'config/{racecar_version}/l1_params.yaml',
    supported_behaviors=('GB_TRACK', 'TRAILING', 'OVERTAKE'),
    required_inputs=('pose', 'velocity', 'waypoints', 'state', 'frenet', 'track'),
    config_parameter='map_config', waypoint_timeout_cycles=10, lookup_table=True, fallback=(('FTGONLY', 'FTG'),),
    reference_description='Nearest planner waypoint; steering = atan(wheelbase * curvature)',
    speed_label='Planner desired', steering_label='Planner-derived'))
register(ControllerSpec(
    'PP', 'controller.pp.adapter:PPAdapter', 'controller.pursuit:load_config',
    'stack_master', 'config/{racecar_version}/l1_params.yaml',
    supported_behaviors=('GB_TRACK', 'TRAILING', 'OVERTAKE'),
    required_inputs=('pose', 'velocity', 'waypoints', 'state', 'frenet', 'track'),
    waypoint_timeout_cycles=10, fallback=(('FTGONLY', 'FTG'),),
    reference_description='Nearest planner waypoint; steering = atan(wheelbase * curvature)',
    speed_label='Planner desired', steering_label='Planner-derived'))
register(ControllerSpec(
    'FTG', 'controller.ftg.adapter:FTGAdapter', 'controller.ftg.adapter:load_config',
    'stack_master', 'config/state_machine_params.yaml',
    supported_behaviors=('GB_TRACK', 'TRAILING', 'OVERTAKE', 'FTGONLY'),
    required_inputs=('scan', 'state'),
    remote_parameters=('debug', 'safety_radius', 'max_lidar_dist', 'max_speed', 'range_offset', 'track_width'),
    reference_description='No waypoint reference: reactive LiDAR controller'))
