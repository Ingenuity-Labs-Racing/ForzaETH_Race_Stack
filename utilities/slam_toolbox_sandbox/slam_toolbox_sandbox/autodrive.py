#!/usr/bin/env python3
"""
Autopilot for the sandbox car: pure pursuit along the track centerline, using Gazebo's
ground truth pose (so it drives the same line no matter what SLAM is doing).

    ros2 run slam_toolbox_sandbox autodrive --ros-args -p speed:=0.8 -p laps:=1.2

Parameters:
    speed      [m/s]  cruise speed (slows down automatically in sharp turns)
    laps              how far to drive, 0 = keep going until Ctrl+C
    lookahead  [m]    pure pursuit lookahead distance
    waypoints         global_waypoints.json with 'centerline_waypoints'
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

DEFAULT_WAYPOINTS = os.path.expanduser(
    '~/ws/src/race_stack/stack_master/maps/Victoria/global_waypoints.json')


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoDrive(Node):
    def __init__(self):
        super().__init__('autodrive')
        self.speed = self.declare_parameter('speed', 0.8).value
        self.laps = self.declare_parameter('laps', 1.0).value
        self.lookahead = self.declare_parameter('lookahead', 0.5).value
        path = self.declare_parameter('waypoints', DEFAULT_WAYPOINTS).value

        with open(path) as f:
            wpnts = json.load(f)['centerline_waypoints']['wpnts']
        self.path = [(w['x_m'], w['y_m']) for w in wpnts]
        self.n = len(self.path)

        self.pose = None
        self.idx = None
        self.progress = 0  # waypoints passed since start
        self.last_progress = 0
        self.last_progress_time = None
        self.reverse_until = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self.pose_cb, 10)
        self.create_timer(0.05, self.control)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'{self.n} waypoints, speed {self.speed} m/s, laps {self.laps or "unlimited"}')

    def pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def point(self, i):
        return self.path[i % self.n]

    def dist(self, i):
        px, py = self.point(i)
        return math.hypot(px - self.pose[0], py - self.pose[1])

    def control(self):
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.idx is None:
            self.idx = min(range(self.n), key=self.dist)
            self.last_progress_time = now

        # advance along the path (only forwards, within a short window)
        best = min(range(self.idx, self.idx + 20), key=self.dist)
        self.progress += best - self.idx
        self.idx = best % self.n

        if self.laps > 0 and self.progress >= self.laps * self.n:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'Done: {self.progress / self.n:.2f} laps')
            raise SystemExit

        # stuck detection: no progress for 3 s -> back up for 1 s
        if self.progress > self.last_progress:
            self.last_progress, self.last_progress_time = self.progress, now
        elif self.reverse_until is None and now - self.last_progress_time > 3.0:
            self.get_logger().warn('Stuck - backing up')
            self.reverse_until = now + 1.0
        cmd = Twist()
        if self.reverse_until is not None:
            if now < self.reverse_until:
                cmd.linear.x = -0.3
                self.cmd_pub.publish(cmd)
                return
            self.reverse_until = None
            self.last_progress_time = now

        # pure pursuit target: first path point at least `lookahead` away
        j = self.idx
        while self.dist(j) < self.lookahead and j < self.idx + self.n:
            j += 1
        tx, ty = self.point(j)
        x, y, yaw = self.pose
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        v = self.speed * max(0.2, math.cos(alpha))  # slow down when pointing away from the path
        cmd.linear.x = v
        cmd.angular.z = max(-2.5, min(2.5, 2.0 * v * math.sin(alpha) / self.lookahead))
        self.cmd_pub.publish(cmd)

    def report(self):
        if self.idx is not None:
            self.get_logger().info(f'{self.progress / self.n:.2f} laps')


def main():
    rclpy.init()
    node = AutoDrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    try:
        node.cmd_pub.publish(Twist())  # stop the car
    except Exception:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
