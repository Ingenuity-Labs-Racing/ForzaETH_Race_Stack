"""Run one continuous simulation trial, record two laps, then create reports."""
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from f110_msgs.msg import WpntArray
import yaml

from core.controller_registry import get_controller_spec

from .metrics import LapProgress, add_errors, build_summary, controller_parameters
from .plotting import generate_plots

FIELDS = (
    'cycle_id', 'timestamp_ns', 'pose_stamp_ns', 'elapsed_s', 'lap', 'progress_m',
    'pose_x', 'pose_y', 'pose_yaw', 'measured_speed', 'frenet_s',
    'prediction_pose_x', 'prediction_pose_y', 'prediction_pose_yaw',
    'reference_local_index', 'reference_x', 'reference_y', 'reference_yaw',
    'reference_speed', 'reference_steering', 'reference_steering_rate',
    'command_speed', 'command_steering', 'requested_steering_rate', 'steering_rate',
    'solve_time_s', 'pose_age_s', 'valid', 'reason', 'failure_stage',
    'position_error_m', 'heading_error_deg', 'speed_tracking_error_mps',
    'speed_command_error_mps', 'steering_command_error_rad',
)


def load_experiment(path, source_root):
    with open(path, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict):
        raise ValueError('Evaluation YAML must be a mapping')
    spec = get_controller_spec(cfg.get('controller'))
    if 'GB_TRACK' not in spec.supported_behaviors:
        raise ValueError('Controller does not support global tracking')
    cfg['controller'] = spec.name
    if not isinstance(cfg.get('controllers'), dict):
        raise ValueError('Evaluation requires controller parameters inside controllers.<name>')
    spec.parameters(cfg)
    if cfg.get('laps') != 2:
        raise ValueError('This mode requires exactly two continuous laps')
    if not isinstance(cfg.get('start_waypoint'), int) or cfg['start_waypoint'] < 0:
        raise ValueError('start_waypoint must be a nonnegative integer')
    for key in ('startup_timeout_s', 'reset_timeout_s', 'trial_timeout_s',
                'stall_timeout_s', 'data_timeout_s', 'stop_timeout_s'):
        if not isinstance(cfg.get(key), (int, float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if not (source_root / 'controller').is_dir():
        raise ValueError('source_root must point to the mounted race_stack source')
    return cfg


class TrialRunner(Node):
    def __init__(self):
        super().__init__('controller_evaluation', automatically_declare_parameters_from_overrides=True)
        self.source_root = Path(self.get_parameter('source_root').value).resolve()
        self.cfg = load_experiment(self.get_parameter('evaluation_config').value, self.source_root)
        self.evaluation_path = Path(self.get_parameter('evaluation_config').value).resolve()
        self.evaluation_bytes = self.evaluation_path.read_bytes()
        self.spec = get_controller_spec(self.cfg['controller'])
        self.controller_cfg = controller_parameters(self.cfg, self.cfg['controller'])
        self.controller_bytes = yaml.safe_dump(self.controller_cfg, sort_keys=False).encode('utf-8')
        output = self.get_parameter('output_directory').value
        parent = Path(output) if output else self.source_root / self.cfg['output_directory']
        if not parent.is_absolute():
            parent = self.source_root / parent
        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + uuid.uuid4().hex[:6]
        self.directory = parent / name
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / 'experiment.yaml').write_bytes(self.evaluation_bytes)
        (self.directory / 'controller_params.yaml').write_bytes(self.controller_bytes)
        self.metadata = dict(map=self.cfg['map_name'], controller=self.cfg['controller'],
                             controller_config_sha256=hashlib.sha256(self.controller_bytes).hexdigest(),
                             evaluation_config_sha256=hashlib.sha256(self.evaluation_bytes).hexdigest(),
                             controller_parameters=self.controller_cfg,
                             latency_compensation_enabled=self.controller_cfg.get('latency_compensation_s', 0) > 0,
                             experiment_config=self.cfg, source_root=str(self.source_root),
                             simulator_parameters=yaml.safe_load((self.source_root / 'stack_master/config/SIM/sim_params.yaml').read_text()),
                             lookup_table=self.controller_cfg.get('lookup_table', 'NUC2_hangar_pacejka') if self.spec.lookup_table else None,
                             **self.spec.report_metadata(),
                             initial_pose=None)
        if self.spec.lookup_table or self.spec.steering_label == 'Planner-derived':
            params = self.metadata['simulator_parameters']
            self.metadata['reference_wheelbase_m'] = float(params['lf'] + params['lr'])
        self.rows = []
        self.planned = []
        self.track_length = None
        self.last_sample = None
        self.last_sample_wall = time.monotonic()
        self.phase = 'WAIT_READY'
        self.born = time.monotonic()
        self.sim_verified = False
        self.sim_future = None
        self.future = None
        self.progress = None
        self.start_stamp_ns = None
        self.start_wall = None
        self.last_progress = 0.0
        self.last_motion = None
        self.reset_settled = 0
        self.finished = False
        self.reason = None
        self.stop_confirmed = False
        self.last_cycle_id = None
        self.file = (self.directory / 'samples.csv').open('w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS, extrasaction='ignore')
        self.writer.writeheader()
        self.file.flush()
        self.enable_client = self.create_client(SetBool, '/controller/evaluation/enable')
        self.reset_client = self.create_client(Trigger, '/controller/evaluation/reset')
        self.global_client = self.create_client(GetParameters, '/global_parameters/get_parameters')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(WpntArray, '/global_waypoints_scaled', self.on_path, 10)
        self.create_subscription(String, '/controller/evaluation/sample', self.on_sample, 100)
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'Evaluation output: {self.directory}')

    def on_path(self, message):
        if self.planned or len(message.wpnts) < 3:
            return
        points = list(message.wpnts)
        self.track_length = float(points[-1].s_m)
        if math.hypot(points[-1].x_m - points[0].x_m, points[-1].y_m - points[0].y_m) < 1e-5:
            points = points[:-1]
        if self.cfg['start_waypoint'] >= len(points) or self.track_length <= 0:
            self.request_finish('invalid_reference_path')
            return
        self.planned = [dict(index=i, s=float(p.s_m), x=float(p.x_m), y=float(p.y_m),
                             heading=float(p.psi_rad), curvature=float(p.kappa_radpm),
                             reference_speed=float(p.vx_mps)) for i, p in enumerate(points)]
        self.write_path()

    def write_path(self):
        with (self.directory / 'planned_path.csv').open('w', newline='') as stream:
            fields = ('index', 's', 'x', 'y', 'heading', 'curvature', 'reference_speed')
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.planned)

    def on_sample(self, message):
        if self.finished:
            return
        try:
            sample = json.loads(message.data)
            if sample.get('schema_version') != 1:
                raise ValueError('Unsupported diagnostics schema')
        except (ValueError, TypeError):
            self.request_finish('invalid_diagnostics')
            return
        self.last_sample = sample
        self.last_sample_wall = time.monotonic()
        if self.phase == 'WAIT_RESET':
            target = self.planned[self.cfg['start_waypoint']]
            if all(sample.get(k) is not None for k in ('pose_x', 'pose_y', 'pose_yaw', 'measured_speed', 'frenet_s')):
                angle = sample['pose_yaw'] - target['heading']
                settled = (math.hypot(sample['pose_x'] - target['x'], sample['pose_y'] - target['y']) < .05
                           and abs(math.atan2(math.sin(angle), math.cos(angle))) < .1
                           and abs(sample['measured_speed']) < .02
                           and sample.get('status') == 'paused')
                self.reset_settled = self.reset_settled + 1 if settled else 0
            return
        if self.phase not in ('ENABLING', 'RUNNING') or sample.get('status') == 'paused':
            return
        if self.last_cycle_id is not None and sample['cycle_id'] <= self.last_cycle_id:
            self.request_finish('controller_restarted')
            return
        self.last_cycle_id = sample['cycle_id']
        elapsed = (sample['timestamp_ns'] - self.start_stamp_ns) / 1e9
        if elapsed < 0:
            self.request_finish('clock_went_backwards')
            return
        lap = min(self.progress.completed + 1, 2)
        if sample.get('frenet_s') is None:
            self.request_finish('missing_frenet_position')
            return
        try:
            completed = self.progress.update(sample['frenet_s'], elapsed)
        except ValueError:
            self.request_finish('unexpected_progress_jump')
            return
        row = add_errors(sample)
        row.update(elapsed_s=elapsed, lap=lap, progress_m=self.progress.progress)
        self.rows.append(row)
        self.writer.writerow(row)
        self.file.flush()
        if self.progress.progress > self.last_progress + .05:
            self.last_progress = self.progress.progress
            self.last_motion = time.monotonic()
        if not sample.get('valid', False):
            self.request_finish('controller_failure: ' + sample.get('reason', 'unknown'))
        elif completed >= 2:
            self.request_finish('completed')

    def set_enabled(self, value):
        request = SetBool.Request()
        request.data = value
        return self.enable_client.call_async(request)

    def tick(self):
        if self.finished:
            return
        now = time.monotonic()
        if self.phase == 'FINISHING':
            if self.future is not None and self.future.done():
                try:
                    self.stop_confirmed = bool(self.future.result().success)
                except Exception:
                    self.stop_confirmed = False
                self.finalize()
            elif now - self.finish_wall > self.cfg['stop_timeout_s']:
                self.finalize()
            return
        if self.phase == 'WAIT_READY':
            if now - self.born > self.cfg['startup_timeout_s']:
                self.request_finish('startup_timeout')
                return
            if self.sim_future is None and self.global_client.service_is_ready():
                request = GetParameters.Request()
                request.names = ['sim', 'map_path']
                self.sim_future = self.global_client.call_async(request)
            if self.sim_future is not None and self.sim_future.done() and not self.sim_verified:
                try:
                    response = self.sim_future.result()
                except Exception:
                    self.request_finish('simulation_parameter_service_failed')
                    return
                if not response or len(response.values) != 2 or not response.values[0].bool_value:
                    self.request_finish('simulation_required')
                    return
                if Path(response.values[1].string_value).name != self.cfg['map_name']:
                    self.request_finish('map_configuration_mismatch')
                    return
                self.sim_verified = True
            if (self.sim_verified and self.planned and self.last_sample
                    and self.enable_client.service_is_ready() and self.reset_client.service_is_ready()):
                if self.last_sample.get('controller') != self.cfg['controller']:
                    self.request_finish('controller_configuration_mismatch')
                    return
                if self.last_sample.get('configuration_error'):
                    self.request_finish('controller_configuration_error')
                    return
                if self.count_publishers('/drive') != 1:
                    self.request_finish('conflicting_drive_publishers')
                    return
                self.future = self.set_enabled(False)
                self.phase = 'DISABLING'
        elif self.phase in ('DISABLING', 'RESET_CONTROLLER', 'ENABLING'):
            if now - self.born > self.cfg['startup_timeout_s'] and self.phase != 'ENABLING':
                self.request_finish('startup_timeout')
                return
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception:
                    self.request_finish('controller_service_failed')
                    return
                if response is None or not response.success:
                    self.request_finish('controller_service_rejected')
                    return
                if self.phase == 'DISABLING':
                    self.future = self.reset_client.call_async(Trigger.Request())
                    self.phase = 'RESET_CONTROLLER'
                elif self.phase == 'RESET_CONTROLLER':
                    target = self.planned[self.cfg['start_waypoint']]
                    self.metadata['initial_pose'] = {key: target[key] for key in ('x', 'y', 'heading')}
                    message = PoseWithCovarianceStamped()
                    message.header.frame_id = 'map'
                    message.header.stamp = self.get_clock().now().to_msg()
                    message.pose.pose.position.x, message.pose.pose.position.y = target['x'], target['y']
                    message.pose.pose.orientation.z = math.sin(target['heading'] / 2)
                    message.pose.pose.orientation.w = math.cos(target['heading'] / 2)
                    self.pose_pub.publish(message)
                    self.reset_wall = now
                    self.phase = 'WAIT_RESET'
                else:
                    self.phase = 'RUNNING'
                    self.get_logger().info('Trial running: two continuous laps, no intermediate reset')
        elif self.phase == 'WAIT_RESET':
            if now - self.reset_wall > self.cfg['reset_timeout_s']:
                self.request_finish('simulator_reset_timeout')
            elif self.reset_settled >= 3:
                self.progress = LapProgress(self.track_length, self.last_sample['frenet_s'], 2)
                self.start_stamp_ns = self.get_clock().now().nanoseconds
                self.start_wall = self.last_motion = now
                self.future = self.set_enabled(True)
                self.phase = 'ENABLING'
        if self.phase in ('RUNNING', 'ENABLING'):
            if now - self.last_sample_wall > self.cfg['data_timeout_s']:
                self.request_finish('controller_data_timeout')
            elif now - self.start_wall > self.cfg['trial_timeout_s']:
                self.request_finish('trial_timeout')
            elif now - self.last_motion > self.cfg['stall_timeout_s']:
                self.request_finish('stalled')

    def request_finish(self, reason):
        if self.finished or self.phase == 'FINISHING':
            return
        self.reason = reason
        self.phase = 'FINISHING'
        self.finish_wall = time.monotonic()
        self.future = None
        # Never command a controller until the global simulation flag is verified.
        if self.sim_verified and self.enable_client.service_is_ready():
            self.future = self.set_enabled(False)
        else:
            self.finalize()

    def finalize(self):
        if self.finished:
            return
        self.finished = True
        self.file.close()
        self.write_path()
        boundaries = self.progress.boundaries if self.progress else []
        summary = build_summary(self.rows, self.controller_cfg, self.metadata, boundaries,
                                2, self.reason or 'interrupted', self.stop_confirmed)
        summary['report_generation_success'] = False
        path = self.directory / 'summary.json'
        path.write_text(json.dumps(summary, indent=2, allow_nan=False))
        try:
            generate_plots(self.directory)
            summary['report_generation_success'] = True
        except Exception as exc:
            summary['plot_error'] = str(exc)
            self.get_logger().error(f'Plot generation failed: {exc}')
        path.write_text(json.dumps(summary, indent=2, allow_nan=False))
        self.get_logger().info(f"Trial {summary['termination_reason']}; completed {len(boundaries)}/2 laps. Results: {self.directory}")


def main():
    rclpy.init()
    runner = None
    try:
        runner = TrialRunner()
        while rclpy.ok() and not runner.finished:
            rclpy.spin_once(runner, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if runner is not None:
            if not runner.finished:
                if rclpy.ok():
                    runner.request_finish('interrupted')
                    until = time.monotonic() + runner.cfg['stop_timeout_s']
                    while not runner.finished and rclpy.ok() and time.monotonic() < until:
                        rclpy.spin_once(runner, timeout_sec=.1)
                if not runner.finished:
                    runner.reason = runner.reason or 'interrupted'
                    runner.finalize()
            runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
