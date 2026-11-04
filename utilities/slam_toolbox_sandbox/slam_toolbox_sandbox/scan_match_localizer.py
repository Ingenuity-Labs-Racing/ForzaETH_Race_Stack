#!/usr/bin/env python3
"""
Localization by scan-to-map matching on a fixed map. The map is loaded once and never changes
(unlike SLAM Toolbox's localization mode, which adds every scan to its pose graph).

For each scan: predict the pose from odometry, then refine it with Gauss-Newton so the lidar
points land on the walls of the map (minimizes the squared distance from each point to the
nearest wall). A weak prior keeps the odometry guess where the scan says nothing, e.g. along
a straight. Matches that fit the walls badly are rejected and odometry is used instead.

Publishes map -> odom (TF), /map once, /scan_match/pose and /scan_match/inlier_ratio.

    ros2 launch slam_toolbox_sandbox scan_match.launch.py
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt, map_coordinates
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from slam_toolbox_sandbox.png_to_world import load_occupancy


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# 2D poses as (x, y, yaw)
def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def pose_from_tf(tf):
    t = tf.transform
    return (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))


class ScanMatchLocalizer(Node):
    def __init__(self):
        super().__init__('scan_match_localizer')
        map_yaml = self.declare_parameter('map', '').value
        self.init_pose = (self.declare_parameter('x', 0.0).value,
                          self.declare_parameter('y', 0.0).value,
                          self.declare_parameter('yaw', 0.0).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.beam_step = self.declare_parameter('beam_step', 4).value
        self.min_range = self.declare_parameter('min_range', 0.1).value
        self.max_range = self.declare_parameter('max_range', 20.0).value
        self.max_match_dist = self.declare_parameter('max_match_dist', 0.5).value
        self.inlier_dist = self.declare_parameter('inlier_dist', 0.1).value
        self.min_inlier_ratio = self.declare_parameter('min_inlier_ratio', 0.3).value
        self.max_correction = self.declare_parameter('max_correction', 0.5).value
        self.max_iterations = self.declare_parameter('max_iterations', 10).value
        self.translation_weight = self.declare_parameter('translation_weight', 0.01).value
        self.rotation_weight = self.declare_parameter('rotation_weight', 0.1).value
        self.tolerance = Duration(seconds=self.declare_parameter('transform_tolerance', 0.1).value)

        self.load_map(map_yaml)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/scan_match/pose', 10)
        self.ratio_pub = self.create_publisher(Float64, '/scan_match/inlier_ratio', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)
        self.create_timer(0.01, self.process_scan)
        self.create_timer(0.02, self.publish_tf)
        self.create_timer(2.0, self.log_stats)

        self.pending = None      # newest scan whose odometry hasn't arrived yet
        self.map_to_odom = None  # set on the first scan, from init_pose
        self.laser_pose = None   # laser in base_link, static
        self.scans = 0
        self.rejects = 0
        self.last_ratio = 0.0

    def load_map(self, map_yaml):
        occupied, free, info = load_occupancy(map_yaml)
        self.res = info['resolution']
        self.origin = info['origin'][:2]
        # image row 0 is the top of the map: flip so row i is y = origin_y + (i + 0.5) * res
        occupied = occupied[::-1]
        free = free[::-1]
        dist = distance_transform_edt(~occupied) * self.res
        self.dist = np.minimum(dist, self.max_match_dist)
        self.grad_y, self.grad_x = np.gradient(self.dist, self.res)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.res
        grid.info.height, grid.info.width = occupied.shape
        grid.info.origin.position.x, grid.info.origin.position.y = self.origin
        grid.info.origin.orientation.w = 1.0
        data = np.full(occupied.shape, -1, dtype=np.int8)
        data[free] = 0
        data[occupied] = 100
        grid.data = data.ravel().tolist()
        # published once; transient local so late subscribers (Foxglove) still get it
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.map_pub.publish(grid)
        self.get_logger().info(f'Loaded fixed map {map_yaml}: {grid.info.width}x{grid.info.height} cells')

    def lookup(self, field, px, py):
        rows = (py - self.origin[1]) / self.res - 0.5
        cols = (px - self.origin[0]) / self.res - 0.5
        return map_coordinates(field, [rows, cols], order=1, mode='nearest')

    def initialpose_cb(self, msg):
        p = msg.pose.pose
        self.init_pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.map_to_odom = None
        self.get_logger().info(f'Re-initialized at x={self.init_pose[0]:.2f} y={self.init_pose[1]:.2f} '
                               f'yaw={self.init_pose[2]:.2f}')

    def scan_points(self, scan):
        ranges = np.asarray(scan.ranges)[::self.beam_step]
        angles = scan.angle_min + scan.angle_increment * self.beam_step * np.arange(ranges.size)
        ok = np.isfinite(ranges) & (ranges > self.min_range) & (ranges < self.max_range)
        lx, ly = ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])
        # laser frame -> base_link
        c, s = math.cos(self.laser_pose[2]), math.sin(self.laser_pose[2])
        return self.laser_pose[0] + c * lx - s * ly, self.laser_pose[1] + s * lx + c * ly

    def match(self, guess, bx, by):
        """Gauss-Newton on (x, y, yaw); returns the refined pose and the inlier ratio."""
        pose = np.array(guess)
        prior = np.array([self.translation_weight, self.translation_weight, self.rotation_weight])
        n = bx.size
        for _ in range(self.max_iterations):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            wx, wy = pose[0] + c * bx - s * by, pose[1] + s * bx + c * by
            r = self.lookup(self.dist, wx, wy)
            gx, gy = self.lookup(self.grad_x, wx, wy), self.lookup(self.grad_y, wx, wy)
            J = np.stack([gx, gy, gx * (-s * bx - c * by) + gy * (c * bx - s * by)], axis=1)
            delta_prior = pose - np.array(guess)
            delta_prior[2] = wrap(delta_prior[2])
            H = J.T @ J / n + np.diag(prior)
            g = J.T @ r / n + prior * delta_prior
            step = -np.linalg.solve(H, g)
            pose += step
            pose[2] = wrap(pose[2])
            if abs(step[0]) + abs(step[1]) < 1e-4 and abs(step[2]) < 1e-4:
                break
        c, s = math.cos(pose[2]), math.sin(pose[2])
        r = self.lookup(self.dist, pose[0] + c * bx - s * by, pose[1] + s * bx + c * by)
        return tuple(pose), float(np.mean(r < self.inlier_dist))

    def scan_cb(self, scan):
        # Never wait for TF in a callback: that blocks the TF listener on the same executor.
        # The newest scan is matched by process_scan once its odometry has arrived.
        self.pending = scan

    def process_scan(self):
        scan = self.pending
        if scan is None:
            return
        stamp = Time.from_msg(scan.header.stamp)
        try:
            if self.laser_pose is None:
                self.laser_pose = pose_from_tf(self.tf_buffer.lookup_transform(
                    self.base_frame, scan.header.frame_id, Time()))
            odom_pose = pose_from_tf(self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, stamp))
        except TransformException as e:
            if self.get_clock().now() - stamp > Duration(seconds=0.5):
                self.get_logger().warn(f'dropping scan, no TF: {e}', throttle_duration_sec=2.0)
                self.pending = None
            return
        self.pending = None
        if self.map_to_odom is None:
            self.map_to_odom = compose(self.init_pose, inverse(odom_pose))

        guess = compose(self.map_to_odom, odom_pose)
        bx, by = self.scan_points(scan)
        if bx.size < 10:
            return
        pose, ratio = self.match(guess, bx, by)
        self.scans += 1
        self.last_ratio = ratio
        self.ratio_pub.publish(Float64(data=ratio))

        jump = math.hypot(pose[0] - guess[0], pose[1] - guess[1])
        if ratio < self.min_inlier_ratio or jump > self.max_correction:
            self.rejects += 1  # keep the odometry prediction
            return
        self.map_to_odom = compose(pose, inverse(odom_pose))

        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = scan.header.stamp
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.pose_pub.publish(msg)

    def publish_tf(self):
        if self.map_to_odom is None:
            return
        # stamped ahead, like SLAM Toolbox and AMCL, so lookups at the latest sensor time succeed
        tf = TransformStamped()
        tf.header.stamp = (self.get_clock().now() + self.tolerance).to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x, tf.transform.translation.y = self.map_to_odom[0], self.map_to_odom[1]
        tf.transform.rotation.z = math.sin(self.map_to_odom[2] / 2)
        tf.transform.rotation.w = math.cos(self.map_to_odom[2] / 2)
        self.tf_broadcaster.sendTransform(tf)

    def log_stats(self):
        if self.scans:
            self.get_logger().info(f'scans {self.scans}, rejected {self.rejects} '
                                   f'(inlier ratio now {self.last_ratio:.2f})')


def main():
    rclpy.init()
    node = ScanMatchLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
