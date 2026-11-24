"""Reactive LiDAR controller adapter."""
import numpy as np
from core.controller_interface import ControllerAdapter, ControllerResult
from .controller import FTG_Controller


def load_config(document):
    try:
        parameters = dict(document['state_machine']['ros__parameters'] if 'state_machine' in document else document)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('FTG configuration must be a parameter mapping or state_machine.ros__parameters') from exc
    for key in ('debug', 'safety_radius', 'max_lidar_dist', 'max_speed', 'range_offset', 'track_width'):
        if key not in parameters:
            raise ValueError(f'Missing FTG parameter {key}')
    return parameters


class FTGAdapter(ControllerAdapter):
    def __init__(self, config, context):
        self.config = config
        self.engine = FTG_Controller(mapping=context.mapping, node_name='ftg_controller',
                                     use_global_arguments=False, **{
            key: config[key] for key in ('debug', 'safety_radius', 'max_lidar_dist',
                                        'max_speed', 'range_offset', 'track_width')})

    def reset(self, steering=0.):
        self.engine.velocity = 0
        self.engine.scan = None
        self.engine.radians_per_elem = None

    def step(self, inputs):
        ranges = np.asarray(inputs.scan.ranges, dtype=float)
        if not len(ranges) or np.any(np.isnan(ranges)):
            raise ValueError('Invalid LiDAR ranges')
        speed, steering = self.engine.process_lidar(ranges)
        return ControllerResult(float(speed), float(steering), True,
                                diagnostics=dict(valid=True, reason='', failure_stage=''))

    def close(self):
        self.engine.destroy_node()
