"""ROS controller host. Algorithms, configuration, and capabilities come from the registry."""
import os
import json
import math
import time
from pathlib import Path
from dataclasses import replace
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ament_index_python import get_package_share_directory
from ackermann_msgs.msg import AckermannDriveStamped
from f110_msgs.msg import ObstacleArray, WpntArray, GapData
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler
from frenet_conversion.frenet_converter import FrenetConverter
from stack_master.parameter_event_handler import ParameterEventHandler
from core.controller_interface import (ControllerInputs, ControllerResult, ControllerContext,
                                       ControllerAdapter, normalize_behavior)
from core.controller_registry import get_controller_spec


from typing import List, Union
from rcl_interfaces.msg import ParameterValue, ParameterType

def return_param_value(p: ParameterValue) -> Union[None, bool, int, str, float, List]:
    """Returns the value of the parameter based on the parameter type.

    Args:
        p (ParameterValue): Input ParameterValue object

    Raises:
        TypeError: If p.type is not in the enumerated values defined by ParameterType

    Returns:
        Union[None, bool, int, str, float, List]: The relevant value of the parameter.
    """
    if p.type == ParameterType.PARAMETER_NOT_SET:
        return None
    elif p.type == ParameterType.PARAMETER_BOOL:
        return p.bool_value
    elif p.type == ParameterType.PARAMETER_INTEGER:
        return p.integer_value
    elif p.type == ParameterType.PARAMETER_DOUBLE:
        return p.double_value
    elif p.type == ParameterType.PARAMETER_STRING:
        return p.string_value
    elif p.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return p.byte_array_value
    elif p.type == ParameterType.PARAMETER_BOOL_ARRAY:
        return p.bool_array_value
    elif p.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return p.integer_array_value
    elif p.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return p.double_array_value
    elif p.type == ParameterType.PARAMETER_STRING_ARRAY:
        return p.string_array_value
    else:
        raise TypeError(
            f"Parameter has unexpected {p.type=}, the highest expected number is 9. Check rclpy docs.")

class UnavailableController(ControllerAdapter):
    def __init__(self, error, config):
        self.error, self.config = str(error), config
    def reset(self, steering=0.):
        pass
    def step(self, inputs):
        return ControllerResult(reason=self.error, diagnostics={'failure_stage': 'configuration'})


class Controller(Node):
    def __init__(self):
        super().__init__('controller_manager', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.map_path = self.get_remote_parameter('global_parameters', 'map_path')
        self.racecar_version = self.get_remote_parameter('global_parameters', 'racecar_version')
        self.sim = self.get_remote_parameter('global_parameters', 'sim')
        self.state_machine_rate = self.get_remote_parameter('state_machine', 'rate_hz')
        self.rate, self.state = 40, 'GB_TRACK'
        self.spec = get_controller_spec(self.option('mode', 'MAP'))
        self.mode = self.spec.name
        self.mapping = bool(self.option('mapping', False))
        self.LUT_name = self.option('LU_table', 'NUC2_hangar_pacejka')
        self.evaluation_mode = bool(self.option('evaluation_mode', False))
        if self.evaluation_mode and not self.sim:
            raise RuntimeError('Controller evaluation is simulation-only')
        if self.evaluation_mode and 'GB_TRACK' not in self.spec.supported_behaviors:
            raise RuntimeError('Selected controller does not support time trials')
        self.evaluation_enabled = not self.evaluation_mode
        self.input_times, self.pose_stamp_ns = {}, None
        self.position_in_map = self.position_in_map_frenet = None
        self.speed_now = self.waypoint_array_in_map = self.track_length = None
        self.opponent, self.obstacles, self.scan = None, [], None
        self.acc_now = np.zeros(10)
        self.waypoint_safety_counter = 0
        self.last_steering_command = 0.
        self.evaluation_detail, self.evaluation_cycle, self.evaluation_pose_age = {}, 0, None
        self.converter, self.active_adapter = None, None
        self.adapters = {}
        self.adapter_context = ControllerContext(self.rate, self.vehicle_wheelbase(), self.LUT_name,
                                         self.mapping, self.get_logger().info, self.get_logger().warning)
        # Fetch fallback configuration before timers start; never spin for a remote
        # service from inside a mutually-exclusive control callback.
        self.remote_settings = {}
        specs = [self.spec] + [get_controller_spec(name) for _, name in self.spec.fallback]
        for spec in specs:
            for name in spec.remote_parameters:
                if name not in self.remote_settings:
                    self.remote_settings[name] = self.get_remote_parameter('state_machine', name)
        self.adapter = self.create_adapter(self.spec, self.selected_config_path())
        self.adapters[self.mode] = self.adapter
        self.prioritize_dyn = self.adapter.config.get('prioritize_dyn', False)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.steering_pub = self.create_publisher(Marker, 'steering', 10)
        self.lookahead_pub = self.create_publisher(Marker, 'lookahead_point', 10)
        self.trailing_pub = self.create_publisher(Marker, 'trailing_opponent_marker', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'my_waypoints', 10)
        self.l1_pub = self.create_publisher(Point, 'l1_distance', 10)
        self.gap_data = self.create_publisher(GapData, '/trailing/gap_data', 10)
        self.waypoint_array_buf = MarkerArray()
        self.markers_buf = [Marker() for _ in range(1000)]
        if self.evaluation_mode:
            self.evaluation_pub = self.create_publisher(String, '/controller/evaluation/sample', 100)
        self.create_subscription(String, '/state', self.state_cb, 10)
        self.create_subscription(WpntArray, '/global_waypoints', self.track_length_cb, 10)
        self.create_subscription(WpntArray, '/local_waypoints', self.local_waypoint_cb, 10)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self.obstacle_cb, 10)
        self.create_subscription(Odometry, '/car_state/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/car_state/pose', self.car_state_cb, 10)
        self.create_subscription(Odometry, '/car_state/frenet/odom', self.car_state_frenet_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/vesc/sensors/imu/raw', self.imu_cb, 10)
        self.init_dynamic_parameters()
        if self.evaluation_mode:
            self.create_service(SetBool, '/controller/evaluation/enable', self.evaluation_enable_cb)
            self.create_service(Trigger, '/controller/evaluation/reset', self.evaluation_reset_cb)
        self.create_timer(1 / self.rate, self.control_loop)
        self.get_logger().info(f'Controller ready: {self.mode}')
        if self.adapter.error:
            self.get_logger().error(self.adapter.error)

    def option(self, name, default):
        return self.get_parameter(name).value if self.has_parameter(name) else default

    def selected_config_path(self):
        requested = self.option('controller_config', '')
        if not requested and self.spec.config_parameter:
            requested = self.option(self.spec.config_parameter, '')
        return Path(requested) if requested else self.spec.default_path(
            get_package_share_directory(self.spec.config_package), self.racecar_version)

    def vehicle_wheelbase(self):
        base = Path(get_package_share_directory('stack_master')) / 'config' / self.racecar_version
        if self.sim:
            params = yaml.safe_load((base / 'sim_params.yaml').read_text())
            return float(params['lf'] + params['lr'])
        params = yaml.safe_load((base / 'vesc.yaml').read_text())
        return float(params['vesc_to_odom_node']['ros__parameters']['wheelbase'])

    def create_adapter(self, spec, path=None):
        config = {}
        try:
            path = path or spec.default_path(get_package_share_directory(spec.config_package),
                                             self.racecar_version)
            document = yaml.safe_load(Path(path).read_text())
            config = spec.parameters(document)
            # An explicit unified tuning block is authoritative, including FTG settings.
            if not isinstance(document, dict) or 'controllers' not in document:
                for name in spec.remote_parameters:
                    config[name] = self.remote_settings[name]
            context = replace(self.adapter_context,
                              lookup_table=config.get('lookup_table', self.adapter_context.lookup_table))
            return spec.create(config, context)
        except Exception as exc:
            return UnavailableController(exc, config)

    def init_dynamic_parameters(self):
        descriptors = self.adapter.parameter_descriptors()
        self.dynamic_names = tuple(descriptors)
        for name, descriptor in descriptors.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, self.adapter.config[name], descriptor)
        if descriptors:
            self.param_handler = ParameterEventHandler(self)
            self.callback_handle = self.param_handler.add_parameter_event_callback(self.parameters_changed)
            self.adapter.update_parameters({name: self.get_parameter(name).value for name in descriptors})

    def parameters_changed(self, event):
        if event.node == self.get_fully_qualified_name():
            self.adapter.update_parameters({name: self.get_parameter(name).value for name in self.dynamic_names})
            self.prioritize_dyn = self.adapter.config.get('prioritize_dyn', False)

    def state_cb(self, data):
        self.state = normalize_behavior(data.data)
        self.input_times['state'] = time.monotonic()

    def selected_adapter(self):
        if self.state in self.spec.supported_behaviors:
            return self.spec, self.adapter
        fallback = dict(self.spec.fallback).get(self.state)
        if fallback:
            spec = get_controller_spec(fallback)
            if spec.name not in self.adapters:
                self.adapters[spec.name] = self.create_adapter(spec)
            return spec, self.adapters[spec.name]
        raise ValueError(f'{self.mode} does not support behavior {self.state}')

    def run_controller(self):
        stage = 'state'
        try:
            spec, adapter = self.selected_adapter()
            if adapter.error:
                return ControllerResult(reason=adapter.error, diagnostics={'failure_stage': 'configuration'})
            if adapter is not self.active_adapter:
                if self.active_adapter is not None:
                    self.active_adapter.reset()
                adapter.reset(self.last_steering_command)
                self.active_adapter = adapter
            stage = 'inputs'
            now = time.monotonic()
            timeout = adapter.config.get('input_timeout')
            if any(key not in self.input_times or
                   (timeout is not None and now - self.input_times[key] > timeout)
                   for key in spec.required_inputs):
                raise ValueError('Required controller inputs are missing or stale')
            if spec.waypoint_timeout_cycles and now - self.input_times['waypoints'] > (
                    spec.waypoint_timeout_cycles / self.state_machine_rate):
                raise ValueError('Local waypoints stopped arriving')
            pose_age = now - self.input_times.get('pose', now)
            if adapter.config.get('latency_compensation_s', 0.) > 0 and self.pose_stamp_ns:
                pose_age = (self.get_clock().now().nanoseconds - self.pose_stamp_ns) / 1e9
                if pose_age < 0 or (timeout is not None and pose_age > timeout):
                    raise ValueError('Invalid or stale pose timestamp')
            self.evaluation_pose_age = pose_age
            inputs = ControllerInputs(
                pose=self.position_in_map, speed=self.speed_now, waypoints=self.waypoint_array_in_map,
                behavior=self.state, control_dt=1 / self.rate, pose_age_s=pose_age,
                frenet=self.position_in_map_frenet, acceleration=self.acc_now,
                opponent=self.opponent, obstacles=self.obstacles, track_length=self.track_length, scan=self.scan)
            if inputs.pose is not None and not np.all(np.isfinite(inputs.pose)):
                raise ValueError('Nonfinite pose')
            if inputs.speed is not None and not np.isfinite(inputs.speed):
                raise ValueError('Nonfinite speed')
            if inputs.frenet is not None and not np.all(np.isfinite(inputs.frenet)):
                raise ValueError('Nonfinite Frenet input')
            stage = 'control'
            result = adapter.step(inputs)
            if not result.valid:
                adapter.reset()
                return result
            if not np.all(np.isfinite([result.speed, result.steering])):
                raise ValueError('Controller returned nonfinite commands')
            stage = 'inputs'
            if timeout is not None and any(time.monotonic() - self.input_times[key] > timeout
                                           for key in spec.required_inputs):
                raise ValueError('Inputs became stale during control calculation')
            result.diagnostics['active_controller'] = spec.name
            return result
        except Exception as exc:
            if self.active_adapter is not None:
                self.active_adapter.reset()
            return ControllerResult(reason=str(exc), diagnostics={'failure_stage': stage})

    def control_loop(self):
        if self.evaluation_mode and not self.evaluation_enabled:
            self.evaluation_publish_zero()
            return
        result = self.run_controller()
        self.evaluation_detail = dict(result.diagnostics, valid=result.valid, reason=result.reason)
        if not result.valid:
            self.get_logger().warning(f'{self.mode} stopped: {result.reason}', throttle_duration_sec=1.)
        self.publish_command(result.speed if result.valid else 0., result.steering if result.valid else 0.)
        if result.valid:
            self.publish_visuals(result.diagnostics)

    def publish_command(self, speed, steering):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.drive.speed, message.drive.steering_angle = float(speed), float(steering)
        self.last_steering_command = float(steering)
        self.drive_pub.publish(message)
        if self.evaluation_mode:
            self.publish_evaluation_sample(message)

    def evaluation_publish_zero(self):
        self.publish_command(0., 0.)

    def evaluation_enable_cb(self, request, response):
        if request.data and self.adapter.error:
            response.success, response.message = False, self.adapter.error
            return response
        self.evaluation_enabled = bool(request.data)
        if not request.data:
            self.reset_evaluation_controller()
            self.evaluation_detail = {}
            self.evaluation_publish_zero()
        response.success, response.message = True, 'enabled' if request.data else 'paused with zero command'
        return response

    def evaluation_reset_cb(self, request, response):
        if self.evaluation_enabled:
            response.success, response.message = False, 'Disable the controller before resetting'
            return response
        self.reset_evaluation_controller()
        self.last_steering_command = 0.
        self.evaluation_detail = {}
        response.success, response.message = True, f'{self.mode} state reset'
        return response

    def reset_evaluation_controller(self):
        for adapter in self.adapters.values():
            adapter.reset()
        self.active_adapter = None
        self.acc_now.fill(0.)

    def publish_evaluation_sample(self, command):
        def clean(value):
            if isinstance(value, np.ndarray):
                return clean(value.tolist())
            if isinstance(value, dict):
                return {key: clean(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [clean(item) for item in value]
            if isinstance(value, (float, np.floating)):
                return float(value) if math.isfinite(value) else None
            if isinstance(value, np.integer):
                return int(value)
            return value
        pose = self.position_in_map.reshape(-1) if self.position_in_map is not None else [None] * 3
        self.evaluation_cycle += 1
        stamp = command.header.stamp
        sample = dict(
            schema_version=1, cycle_id=self.evaluation_cycle,
            timestamp_ns=stamp.sec * 1000000000 + stamp.nanosec, pose_stamp_ns=self.pose_stamp_ns,
            pose_x=pose[0], pose_y=pose[1], pose_yaw=pose[2], measured_speed=self.speed_now,
            frenet_s=self.position_in_map_frenet[0] if self.position_in_map_frenet is not None else None,
            command_speed=command.drive.speed, command_steering=command.drive.steering_angle,
            pose_age_s=self.evaluation_pose_age, status='running' if self.evaluation_enabled else 'paused',
            controller=self.mode, behavior=self.state, configuration_error=self.adapter.error)
        sample.update(self.evaluation_detail)
        message = String()
        message.data = json.dumps(clean(sample), allow_nan=False)
        self.evaluation_pub.publish(message)

    def publish_visuals(self, detail):
        if 'lookahead_point' not in detail:
            return
        self.set_lookahead_marker(detail['lookahead_point'], 100)
        self.visualize_steering(self.last_steering_command)
        self.l1_pub.publish(Point(x=float(detail['lookahead_index']), y=detail['lookahead_distance']))
        if self.opponent is not None and self.converter is None and hasattr(self, 'waypoints'):
            self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2])
        self.visualize_trailing_opponent()

    def destroy_node(self):
        for adapter in self.adapters.values():
            adapter.close()
        return super().destroy_node()

    def get_remote_parameter(self, remote_node_name, param_name):
        cli = self.create_client(GetParameters, remote_node_name + '/get_parameters')
        while not cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('service not available, waiting again...')
        req = GetParameters.Request()
        req.names = [param_name]
        future = cli.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    res = future.result()
                    return return_param_value(res.values[0])
                except Exception as e:
                    self.get_logger().warn('Service call failed %r' % (e,))
                break

    def scan_cb(self, data: LaserScan):
        self.scan = data
        self.input_times["scan"] = time.monotonic()

    def track_length_cb(self, data: WpntArray):
        if not data.wpnts:
            return
        self.input_times["track"] = time.monotonic()
        self.track_length = data.wpnts[-1].s_m
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad] for wpnt in data.wpnts])

    def obstacle_cb(self, data: ObstacleArray):
        self.obstacles = data.obstacles
        self.input_times["obstacles"] = time.monotonic()
        if len(data.obstacles) > 0 and \
                self.position_in_map_frenet is not None and len(self.position_in_map_frenet) and \
                self.track_length is not None:

            self.opponent_s = None
            static_flag = False  # If we have a Static and a Dynamic obstacle we prefer the dynamic
            closest_opp = self.track_length
            for obstacle in data.obstacles:
                opponent_dist = (obstacle.s_start - self.position_in_map_frenet[0]) % self.track_length
                if opponent_dist < closest_opp or (static_flag and not obstacle.is_static):
                    closest_opp = opponent_dist
                    opponent_static = obstacle.is_static
                    opponent_s = obstacle.s_center
                    opponent_d = obstacle.d_center
                    opponent_vs = obstacle.vs
                    opponent_visible = obstacle.is_visible
                    if opponent_static:
                        static_flag = self.prioritize_dyn  # Chosen Obstacle is static
                    else:
                        static_flag = False  # Chosen obstacle is dynamic
                    self.opponent = [opponent_s, opponent_d, opponent_vs, opponent_static, opponent_visible]
        else:
            self.opponent = None

    def odom_cb(self, data: Odometry):
        self.speed_now = data.twist.twist.linear.x
        self.input_times['velocity'] = time.monotonic()

    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        rot = Rotation.from_quat([data.pose.orientation.x, data.pose.orientation.y,
                                  data.pose.orientation.z, data.pose.orientation.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]
        self.position_in_map = np.array([x, y, theta])[np.newaxis]
        stamp = data.header.stamp
        self.pose_stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        self.input_times['pose'] = time.monotonic()

    def local_waypoint_cb(self, data: WpntArray):
        self.waypoint_list_in_map = []
        for waypoint in data.wpnts:
            waypoint_in_map = [waypoint.x_m, waypoint.y_m]
            speed = waypoint.vx_mps
            if waypoint.d_right + waypoint.d_left != 0:
                self.waypoint_list_in_map.append([waypoint_in_map[0],
                                                  waypoint_in_map[1],
                                                  speed,
                                                  min(waypoint.d_left,
                                                      waypoint.d_right) / (waypoint.d_right + waypoint.d_left),
                                                  waypoint.s_m,
                                                  waypoint.kappa_radpm,
                                                  waypoint.psi_rad,
                                                  waypoint.ax_mps2])
            else:
                self.waypoint_list_in_map.append([waypoint_in_map[0],
                                                  waypoint_in_map[1],
                                                  speed,
                                                  0,
                                                  waypoint.s_m,
                                                  waypoint.kappa_radpm,
                                                  waypoint.psi_rad,
                                                  waypoint.ax_mps2])
        self.waypoint_array_in_map = np.array(self.waypoint_list_in_map)
        self.waypoint_safety_counter = 0
        self.input_times['waypoints'] = time.monotonic()

    def imu_cb(self, data):
        self.input_times["acceleration"] = time.monotonic()
        # save acceleration in a rolling buffer
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = -data.linear_acceleration.y  # vesc is rotated 90 deg, so (-acc_y) == (long_acc)

    def car_state_frenet_cb(self, data: Odometry):
        s = data.pose.pose.position.x
        d = data.pose.pose.position.y
        vs = data.twist.twist.linear.x
        vd = data.twist.twist.linear.y
        self.position_in_map_frenet = np.array([s, d, vs, vd])
        self.input_times['frenet'] = time.monotonic()

    def visualize_steering(self, theta):

        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "car_state/base_link"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = 0
        lookahead_marker.id = 50
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0.01
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = 0.0
        lookahead_marker.pose.position.y = 0.0
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.steering_pub.publish(lookahead_marker)

    def set_waypoint_markers(self, waypoints):
        wpnt_id = 0

        for waypoint in waypoints:
            waypoint_marker = self.markers_buf[wpnt_id]
            waypoint_marker.header.frame_id = "map"
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.type = 2
            waypoint_marker.scale.x = 0.1
            waypoint_marker.scale.y = 0.1
            waypoint_marker.scale.z = 0.1
            waypoint_marker.color.r = 0.0
            waypoint_marker.color.g = 0.0
            waypoint_marker.color.b = 1.0
            waypoint_marker.color.a = 1.0
            waypoint_marker.pose.position.x = waypoint[0]
            waypoint_marker.pose.position.y = waypoint[1]
            waypoint_marker.pose.position.z = 0.0
            waypoint_marker.pose.orientation.x = 0.0
            waypoint_marker.pose.orientation.y = 0.0
            waypoint_marker.pose.orientation.z = 0.0
            waypoint_marker.pose.orientation.w = 1.0
            waypoint_marker.id = wpnt_id + 1
            wpnt_id += 1
        self.waypoint_array_buf.markers = self.markers_buf[:wpnt_id]
        self.waypoint_pub.publish(self.waypoint_array_buf)

    def set_lookahead_marker(self, lookahead_point, id):
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = 0.0
        lookahead_marker.pose.orientation.y = 0.0
        lookahead_marker.pose.orientation.z = 0.0
        lookahead_marker.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_trailing_opponent(self):
        if (self.state == "TRAILING" and (self.opponent is not None)):
            on = True
        else:
            on = False
        opponent_marker = Marker()
        opponent_marker.header.frame_id = "map"
        opponent_marker.header.stamp = self.get_clock().now().to_msg()
        opponent_marker.type = 2
        opponent_marker.scale.x = 0.3
        opponent_marker.scale.y = 0.3
        opponent_marker.scale.z = 0.3
        opponent_marker.color.r = 1.0
        opponent_marker.color.g = 0.0
        opponent_marker.color.b = 0.0
        opponent_marker.color.a = 1.0
        if self.opponent is not None:
            pos = self.converter.get_cartesian([self.opponent[0]], [self.opponent[1]])
            opponent_marker.pose.position.x = pos[0][0]
            opponent_marker.pose.position.y = pos[1][0]
            opponent_marker.pose.position.z = 0.0

        opponent_marker.pose.orientation.x = 0.0
        opponent_marker.pose.orientation.y = 0.0
        opponent_marker.pose.orientation.z = 0.0
        opponent_marker.pose.orientation.w = 1.0
        if not on:
            opponent_marker.action = Marker.DELETE
        self.trailing_pub.publish(opponent_marker)



def main():
    rclpy.init()
    node = None
    try:
        node = Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
