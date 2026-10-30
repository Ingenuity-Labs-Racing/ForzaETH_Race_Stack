"""Shared reference construction and tracking mechanics, independent of solver type."""

from dataclasses import dataclass
import time
import math
import numpy as np
import yaml



REQUIRED = ('horizon', 'dt', 'wheelbase', 'state_weights',
            'input_weights', 'terminal_weight_multiplier', 'speed_min',
            'speed_max', 'steering_min', 'steering_max', 'solver_plugin',
            'steering_rate_min', 'steering_rate_max')


def validate_config(config):
    missing = [key for key in REQUIRED if config.get(key) is None]
    if missing:
        raise ValueError('Missing Predictive controller parameters: ' + ', '.join(missing))
    if isinstance(config['horizon'], bool) or not isinstance(config['horizon'], int) or config['horizon'] < 1:
        raise ValueError('horizon must be a positive integer')
    for key in ('dt', 'wheelbase', 'terminal_weight_multiplier'):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    for key in ('max_solve_time', 'input_timeout'):
        value = config.get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not np.isfinite(value) or value <= 0):
            raise ValueError(f'{key} must be null or finite and positive')
    for key, size in (('state_weights', 4), ('input_weights', 2)):
        weights = np.asarray(config[key], dtype=float)
        if weights.shape != (size,) or not np.all(np.isfinite(weights)) or np.any(weights < 0):
            raise ValueError(f'{key} must contain {size} finite nonnegative weights')
        if key == 'input_weights' and np.any(weights <= 0):
            raise ValueError('input_weights must be positive for a strictly convex QP')
    bounds = np.array([config[key] for key in ('speed_min', 'speed_max', 'steering_min', 'steering_max')])
    if not np.all(np.isfinite(bounds)) or not (0 <= bounds[0] < bounds[1]):
        raise ValueError('speed bounds must be finite with 0 <= min < max')
    if not (-np.pi / 2 < bounds[2] <= 0 <= bounds[3] < np.pi / 2) or bounds[2] == bounds[3]:
        raise ValueError('steering bounds must straddle zero and lie inside +/- pi/2')
    rates = np.array([config['steering_rate_min'], config['steering_rate_max']])
    if not np.all(np.isfinite(rates)) or not (rates[0] <= 0 <= rates[1]) or rates[0] == rates[1]:
        raise ValueError('steering rate bounds must be finite and straddle zero')
    if not isinstance(config['solver_plugin'], str) or not config['solver_plugin'].strip():
        raise ValueError('solver_plugin must be a nonempty string')
    delay = config.get('latency_compensation_s', 0.0)
    if not np.isfinite(delay) or delay < 0:
        raise ValueError('latency_compensation_s must be finite and nonnegative')


@dataclass(frozen=True)
class PredictiveResult:
    speed: float = 0.0
    steering: float = 0.0
    valid: bool = False
    reason: str = ''
    steering_rate: float = 0.0


class PredictiveTracker:
    def __init__(self, config, solver_factory):
        self.config = config
        self.solver = None
        self.error = ''
        self.reset()
        try:
            validate_config(config)
            self.solver = solver_factory(config)
        except Exception as exc:
            self.error = str(exc)

    @classmethod
    def from_file(cls, path):
        try:
            with open(path, encoding='utf-8') as stream:
                config = yaml.safe_load(stream)
            if not isinstance(config, dict):
                raise ValueError('Predictive controller configuration must be a YAML mapping')
        except Exception as exc:
            controller = cls({})
            controller.error = f'Cannot load Predictive controller configuration: {exc}'
            return controller
        return cls(config)

    def reset(self, steering=0.0):
        # Commanded angle only: no actuator lag or yaw-rate steering estimator.
        self.steering_angle = float(steering)
        self.last_diagnostics = {}

    def build_reference(self, pose, speed, waypoints):
        """Time-sample x/y/heading/speed/curvature along the local path.

        Stack columns: x, y, speed, track ratio, s, curvature, heading, acceleration.
        Use geometric arc length so track s wrapping needs no special handling.
        Refuse to extrapolate a moving reference beyond the available path.
        """
        cfg = self.config
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 7:
            raise ValueError('Need at least two local waypoints with heading and curvature')
        if not np.all(np.isfinite(points[:, [0, 1, 2, 5, 6]])) or np.any(points[:, 2] < 0):
            raise ValueError('Waypoint reference contains invalid values or reverse speeds')
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
        keep = np.r_[True, np.diff(arc) > 1e-9]
        arc, points = arc[keep], points[keep]
        if len(arc) < 2:
            raise ValueError('Local path has no length')
        # Start at the waypoint nearest to the vehicle in Euclidean distance.
        index = np.argmin(np.sum((points[:, :2] - pose[:2])**2, axis=1))
        self.last_reference_index = int(index)
        distance = arc[index]
        samples = [distance]
        for k in range(cfg['horizon']):
            desired_speed = np.interp(distance, arc, points[:, 2])
            advance_speed = (max(speed, 0.0) + desired_speed) / 2 if k == 0 else desired_speed
            distance += advance_speed * cfg['dt']
            if distance > arc[-1] + 1e-9:
                raise ValueError('Local waypoints do not cover the Predictive controller time horizon')
            samples.append(min(distance, arc[-1]))
        samples = np.asarray(samples)
        ref = np.vstack([np.interp(samples, arc, points[:, 0]),
                         np.interp(samples, arc, points[:, 1]),
                         np.interp(samples, arc, np.unwrap(points[:, 6])),
                         np.arctan(cfg['wheelbase'] * np.interp(samples, arc, points[:, 5]))])
        velocities = np.interp(samples, arc, points[:, 2])
        curvature = np.interp(samples, arc, points[:, 5])
        controls = np.vstack([velocities[:-1], velocities[:-1] * curvature[:-1],
                              np.diff(ref[3]) / cfg['dt']])
        return ref, controls

    def compensate_pose(self, pose, speed, pose_age_s=0.0):
        """Predict pose under constant speed and last commanded steering angle.

        The configured delay covers solve/delivery; pose age is added separately.
        A zero configured delay disables the entire compensation path.
        """
        delay = self.config.get('latency_compensation_s', 0.0)
        predicted = np.asarray(pose, dtype=float).reshape(3).copy()
        if delay == 0:
            return predicted
        if not np.isfinite(delay) or delay < 0 or not np.isfinite(pose_age_s) or pose_age_s < 0:
            raise ValueError('Latency compensation and pose age must be finite and nonnegative')
        delay += pose_age_s
        distance = speed * delay
        turn = distance * np.tan(self.steering_angle) / self.config['wheelbase']
        # Exact constant-curvature motion; sinc handles zero steering safely.
        chord = distance * np.sinc(turn / (2 * np.pi))
        direction = predicted[2] + turn / 2
        predicted[0] += chord * np.cos(direction)
        predicted[1] += chord * np.sin(direction)
        predicted[2] = np.arctan2(np.sin(predicted[2] + turn), np.cos(predicted[2] + turn))
        if not np.all(np.isfinite(predicted)):
            raise ValueError('Latency-compensated pose is invalid')
        return predicted

    def step(self, pose, speed, waypoints, control_dt=None, pose_age_s=0.0):
        self.last_diagnostics = {'valid': False, 'failure_stage': 'inputs'}
        solve_start = None
        if self.solver is None:
            self.last_diagnostics.update(reason=self.error, failure_stage='configuration')
            return PredictiveResult(reason=self.error)
        try:
            pose = np.asarray(pose, dtype=float).reshape(3)
            if not np.all(np.isfinite(pose)) or not np.isfinite(speed):
                raise ValueError('Pose or velocity is invalid')
            if speed < -1e-6:
                raise ValueError('Reverse motion is not supported by this adapter')
            control_dt = self.config['dt'] if control_dt is None else control_dt
            if not np.isfinite(control_dt) or control_dt <= 0:
                raise ValueError('Control timestep must be finite and positive')
            if not np.isfinite(self.steering_angle):
                raise ValueError('Commanded steering angle is invalid')
            pose = self.compensate_pose(pose, speed, pose_age_s)
            self.last_diagnostics.update(prediction_pose_x=float(pose[0]), prediction_pose_y=float(pose[1]),
                                         prediction_pose_yaw=float(pose[2]), failure_stage='reference')
            reference, controls = self.build_reference(pose, speed, waypoints)
            self.last_diagnostics.update(reference_local_index=self.last_reference_index,
                reference_x=float(reference[0, 0]), reference_y=float(reference[1, 0]),
                reference_yaw=float(reference[2, 0]), reference_steering=float(reference[3, 0]),
                reference_speed=float(controls[0, 0]), reference_steering_rate=float(controls[2, 0]),
                failure_stage='solve')
            start = solve_start = time.perf_counter()
            command = np.asarray(self.solver.solve(np.r_[pose, self.steering_angle], reference, controls)).reshape(2)
            self.last_diagnostics.update(solve_time_s=time.perf_counter() - start)
            solve_limit = self.config.get('max_solve_time')
            if solve_limit is not None and time.perf_counter() - start > solve_limit:
                raise ValueError('Predictive controller solve exceeded max_solve_time')
            self.last_diagnostics['failure_stage'] = 'command_validation'
            if not np.all(np.isfinite(command)):
                raise ValueError('Predictive controller returned nonfinite commands')
            low = np.array([self.config['speed_min'], self.config['steering_rate_min']])
            high = np.array([self.config['speed_max'], self.config['steering_rate_max']])
            if np.any(command < low - 1e-6) or np.any(command > high + 1e-6):
                raise ValueError('Predictive controller returned commands outside configured bounds')
            command = np.clip(command, low, high)
            old_angle = self.steering_angle
            self.steering_angle = float(np.clip(old_angle + command[1] * control_dt,
                                               self.config['steering_min'], self.config['steering_max']))
            applied_rate = (self.steering_angle - old_angle) / control_dt
            self.last_diagnostics.update(valid=True, reason='', failure_stage='',
                                         requested_steering_rate=float(command[1]), steering_rate=float(applied_rate))
            return PredictiveResult(float(command[0]), self.steering_angle, True,
                             steering_rate=float(applied_rate))
        except Exception as exc:
            diagnostics = dict(self.last_diagnostics)
            if solve_start is not None and 'solve_time_s' not in diagnostics:
                diagnostics['solve_time_s'] = time.perf_counter() - solve_start
            self.reset()
            diagnostics.update(valid=False, reason=str(exc))
            self.last_diagnostics = diagnostics
            return PredictiveResult(reason=str(exc))


def planner_reference(pose, waypoints, wheelbase):
    """Unmodified nearest local planner waypoint; curvature gives geometric steering."""
    pose = np.asarray(pose, dtype=float).reshape(3)
    points = np.asarray(waypoints, dtype=float)
    if (not np.all(np.isfinite(pose)) or points.ndim != 2
            or len(points) == 0 or points.shape[1] < 7):
        raise ValueError('Missing pose or planner waypoints')
    if not np.all(np.isfinite(points[:, [0, 1, 2, 5, 6]])):
        raise ValueError('Nonfinite planner reference')
    if not math.isfinite(wheelbase) or wheelbase <= 0:
        raise ValueError('Reference wheelbase must be positive and finite')
    index = int(np.argmin(np.sum((points[:, :2] - pose[:2]) ** 2, axis=1)))
    point = points[index]
    return dict(reference_local_index=index, reference_x=float(point[0]),
                reference_y=float(point[1]), reference_yaw=float(point[6]),
                reference_speed=float(point[2]),
                reference_steering=math.atan(wheelbase * point[5]))
