"""Pure lap-progress and report calculations, independent of ROS."""
import math
import numpy as np

METRIC_DEFINITIONS = {
    'position_error_m': 'Euclidean distance from measured pose to the recorded first controller reference.',
    'heading_error_deg': 'Measured heading minus recorded reference heading, wrapped to [-180, 180] degrees.',
    'speed_tracking_error_mps': 'Measured speed minus recorded reference speed.',
    'speed_command_error_mps': 'Published speed command minus recorded reference speed.',
    'steering_command_error_rad': 'Published steering angle minus recorded reference angle; not actuator feedback.',
    'statistics': 'Sample-based over finite values; absolute percentiles; no interpolation across missing samples.',
    'saturation': 'Fraction of finite samples at either configured bound within 1e-5.',
    'solve_duration': 'Solver invocation and first-command extraction; excludes reference building.',
    'lap_detection': 'Signed unwrapped Frenet progress from the reset position; a full track length per lap.',
}


from core.controller_registry import get_controller_spec
from controller.pursuit import MAP_NUMERIC_PARAMETERS
from core.reference import planner_reference


def controller_parameters(document, mode):
    return get_controller_spec(mode).parameters(document)


class LapProgress:
    """Track full circuits from any starting s; reject implausible jumps."""
    def __init__(self, track_length, start_s, laps=2):
        if not math.isfinite(track_length) or track_length <= 0 or laps < 1:
            raise ValueError('Invalid lap progress configuration')
        self.length = float(track_length)
        self.previous_s = float(start_s) % self.length
        self.progress = 0.0
        self.completed = 0
        self.target = int(laps)
        self.boundaries = []
        self.last_elapsed = 0.0

    def update(self, s, elapsed):
        s = float(s)
        if not math.isfinite(s):
            raise ValueError('Nonfinite track position')
        delta = (s - self.previous_s + self.length / 2) % self.length - self.length / 2
        if abs(delta) > min(2.0, self.length / 4):
            raise ValueError('Implausible progress jump; possible simulator reset')
        previous = self.progress
        self.progress += delta
        while self.completed < self.target and self.progress >= (self.completed + 1) * self.length:
            crossing = (self.completed + 1) * self.length
            fraction = (crossing - previous) / delta if delta > 0 else 1.0
            crossing_time = self.last_elapsed + np.clip(fraction, 0, 1) * (elapsed - self.last_elapsed)
            self.boundaries.append(float(crossing_time))
            self.completed += 1
        self.previous_s, self.last_elapsed = s % self.length, float(elapsed)
        return self.completed


def finite_values(rows, key):
    return np.array([float(row[key]) for row in rows
                     if row.get(key) is not None and math.isfinite(float(row[key]))])


def error_stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {'samples': 0, 'mean': None, 'mean_absolute': None, 'rmse': None,
                'p95_absolute': None, 'max_absolute': None}
    return dict(samples=int(len(values)), mean=float(np.mean(values)),
                mean_absolute=float(np.mean(np.abs(values))), rmse=float(np.sqrt(np.mean(values ** 2))),
                p95_absolute=float(np.percentile(np.abs(values), 95)), max_absolute=float(np.max(np.abs(values))))


def distribution(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return dict(samples=0, mean=None, median=None, p95=None, maximum=None)
    return dict(samples=int(len(values)), mean=float(np.mean(values)), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), maximum=float(np.max(values)))


def add_errors(sample):
    """Keep physical measured pose distinct from compensated prediction pose."""
    row = dict(sample)
    pairs = [('speed_tracking_error_mps', 'measured_speed', 'reference_speed'),
             ('speed_command_error_mps', 'command_speed', 'reference_speed'),
             ('steering_command_error_rad', 'command_steering', 'reference_steering')]
    for result, first, second in pairs:
        row[result] = None if row.get(first) is None or row.get(second) is None else row[first] - row[second]
    if all(row.get(k) is not None for k in ('pose_x', 'pose_y', 'reference_x', 'reference_y')):
        row['position_error_m'] = math.hypot(row['pose_x'] - row['reference_x'], row['pose_y'] - row['reference_y'])
    else:
        row['position_error_m'] = None
    if row.get('pose_yaw') is not None and row.get('reference_yaw') is not None:
        error = row['pose_yaw'] - row['reference_yaw']
        row['heading_error_deg'] = math.degrees(math.atan2(math.sin(error), math.cos(error)))
    else:
        row['heading_error_deg'] = None
    return row


def summarise_rows(rows, config, controller='AGMPC', capabilities=None):
    if capabilities is None:
        capabilities = get_controller_spec(controller).report_capabilities
    metrics = {key: error_stats(finite_values(rows, key)) for key in (
        'position_error_m', 'heading_error_deg', 'speed_tracking_error_mps',
        'speed_command_error_mps', 'steering_command_error_rad')}
    intervals = np.diff(finite_values(rows, 'elapsed_s'))
    cycles = finite_values(rows, 'cycle_id')
    gaps = np.maximum(np.diff(cycles) - 1, 0)
    saturation = {}
    for key, low, high in [('command_speed', 'speed_min', 'speed_max'),
                           ('command_steering', 'steering_min', 'steering_max'),
                           ('steering_rate', 'steering_rate_min', 'steering_rate_max')]:
        values = finite_values(rows, key)
        saturation[key] = None if ('constraints' not in capabilities or not len(values) or low not in config or high not in config) else float(100 * np.mean(
            np.isclose(values, config[low], atol=1e-5, rtol=0) |
            np.isclose(values, config[high], atol=1e-5, rtol=0)))
    return dict(samples=len(rows), errors=metrics, sample_interval_s=distribution(intervals),
                missing_cycle_count=int(np.sum(gaps)), solve_time_s=distribution(finite_values(rows, 'solve_time_s')) if 'solve_time' in capabilities else None,
                saturation_percent=saturation,
                invalid_outputs=sum(not row.get('valid', False) for row in rows),
                solver_failures=sum(row.get('failure_stage') == 'solve' and not row.get('valid', False) for row in rows) if 'solve_time' in capabilities else None,
                missing_input_events=sum(row.get('failure_stage') == 'inputs' for row in rows))


def build_summary(rows, config, metadata, boundaries, requested_laps, reason, stop_confirmed):
    complete = len(boundaries) >= requested_laps and reason == 'completed'
    result = dict(schema_version=1, experiment=metadata, requested_laps=requested_laps,
                  completed_laps=len(boundaries), success=complete, termination_reason=reason,
                  stop_confirmed=bool(stop_confirmed), metric_definitions=METRIC_DEFINITIONS,
                  total_duration_s=boundaries[-1] if complete else (rows[-1]['elapsed_s'] if rows else 0.0),
                  lap_boundaries_s=list(boundaries), overall=summarise_rows(rows, config, metadata.get('controller', 'AGMPC'), metadata.get('report_capabilities')), laps=[],
                  collision_count=None,
                  limitations=['No measured steering angle; steering plot compares commands with references.',
                               'Simulator collision flags are not published by the existing bridge; timeout/stall detection is used.',
                               'First lap starts from rest; second lap is continuous.',
                               'Position error is waypoint error, not perpendicular distance to a path segment.'])
    for lap in range(1, requested_laps + 1):
        subset = [row for row in rows if row['lap'] == lap]
        if not subset and lap > len(boundaries):
            continue
        completed = lap <= len(boundaries)
        start = boundaries[lap - 2] if lap > 1 and lap - 2 < len(boundaries) else 0.0
        record = dict(lap=lap, completed=completed,
                      duration_s=boundaries[lap - 1] - start if completed else None,
                      partial_duration_s=None if completed else max(0.0, rows[-1]['elapsed_s'] - start) if rows else 0.0)
        record.update(summarise_rows(subset, config, metadata.get('controller', 'AGMPC'), metadata.get('report_capabilities')))
        result['laps'].append(record)
    return result
