"""Shared adapter mechanics for pursuit controllers; their algorithms remain independent."""
import math
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from core.controller_interface import ControllerAdapter, ControllerResult, normalize_behavior
from core.reference import planner_reference

MAP_NUMERIC_PARAMETERS = (
    't_clip_min', 't_clip_max', 'm_l1', 'q_l1', 'speed_lookahead', 'lat_err_coeff',
    'acc_scaler_for_steer', 'dec_scaler_for_steer', 'start_scale_speed',
    'end_scale_speed', 'downscale_factor', 'speed_lookahead_for_steer',
    'trailing_gap', 'trailing_p_gain', 'trailing_i_gain', 'trailing_d_gain',
    'blind_trailing_speed',
)


def load_config(document):
    try:
        parameters = dict(document['controller']['ros__parameters'] if 'controller' in document else document)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Pursuit configuration must be a parameter mapping or controller.ros__parameters') from exc
    for key in MAP_NUMERIC_PARAMETERS:
        value = parameters.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'Parameter {key} must be finite and numeric')
        parameters[key] = float(value)
    if not isinstance(parameters.get('prioritize_dyn'), bool):
        raise ValueError('prioritize_dyn must be boolean')
    return parameters


class PursuitAdapter(ControllerAdapter):
    def __init__(self, config, context):
        self.config, self.context = config, context
        self.engine = self.create_engine()

    def engine_parameters(self):
        keys = MAP_NUMERIC_PARAMETERS + ('prioritize_dyn',)
        return {key: self.config[key] for key in keys}

    def reset(self, steering=0.):
        if hasattr(self.engine, 'reset'):
            self.engine.reset()
        else:
            self.engine = self.create_engine()
        self.engine.curr_steering_angle = float(steering)

    def update_parameters(self, parameters):
        super().update_parameters(parameters)
        for key, value in parameters.items():
            if hasattr(self.engine, key):
                setattr(self.engine, key, value)

    def step(self, inputs):
        reference = planner_reference(inputs.pose, inputs.waypoints, self.context.wheelbase)
        pose = np.asarray(inputs.pose).reshape(1, 3)
        speed, _, _, steering, point, distance, index = self.engine.main_loop(
            'StateType.' + normalize_behavior(inputs.behavior), pose, inputs.waypoints,
            inputs.speed, inputs.opponent, inputs.frenet, inputs.acceleration, inputs.track_length)
        reference.update(valid=True, reason='', failure_stage='',
                         lookahead_point=[float(v) for v in point],
                         lookahead_distance=float(distance), lookahead_index=int(index))
        return ControllerResult(float(speed), float(steering), True, diagnostics=reference)

    def parameter_descriptors(self):
        # Filled from the existing ROS descriptors without changing their ranges.
        return self._descriptors()

    def _descriptors(self):
        param_dicts = [{'name': 't_clip_min',
                        'default': self.config["t_clip_min"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.5, step=0.01)])},
                       {'name': 't_clip_max',
                        'default': self.config["t_clip_max"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)])},
                       {'name': 'm_l1',
                        'default': self.config["m_l1"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.001)])},
                       {'name': 'q_l1',
                        'default': self.config["q_l1"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.001)])},
                       {'name': 'speed_lookahead',
                        'default': self.config["speed_lookahead"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])},
                       {'name': 'lat_err_coeff',
                        'default': self.config["lat_err_coeff"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])},
                       {'name': 'acc_scaler_for_steer',
                        'default': self.config["acc_scaler_for_steer"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.5, step=0.01)])},
                       {'name': 'dec_scaler_for_steer',
                        'default': self.config["dec_scaler_for_steer"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.5, step=0.01)])},
                       {'name': 'start_scale_speed',
                        'default': self.config["start_scale_speed"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)])},
                       {'name': 'end_scale_speed',
                        'default': self.config["end_scale_speed"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)])},
                       {'name': 'downscale_factor',
                        'default': self.config["downscale_factor"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.5, step=0.01)])},
                       {'name': 'speed_lookahead_for_steer',
                        'default': self.config["speed_lookahead_for_steer"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.2, step=0.01)])},
                       {'name': 'prioritize_dyn',
                        'default': self.config['prioritize_dyn'],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)},
                       {'name': 'trailing_gap',
                        'default': self.config["trailing_gap"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=3.0, step=0.1)])},
                       {'name': 'trailing_p_gain',
                        'default': self.config["trailing_p_gain"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=3.0, step=0.01)])},
                       {'name': 'trailing_i_gain',
                        'default': self.config["trailing_i_gain"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.5, step=0.001)])},
                       {'name': 'trailing_d_gain',
                        'default': self.config["trailing_d_gain"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)])},
                       {'name': 'blind_trailing_speed',
                        'default': self.config["blind_trailing_speed"],
                        'descriptor': ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, floating_point_range=[FloatingPointRange(from_value=0.0, to_value=3.0, step=0.01)])}]
        return {item["name"]: item["descriptor"] for item in param_dicts}
