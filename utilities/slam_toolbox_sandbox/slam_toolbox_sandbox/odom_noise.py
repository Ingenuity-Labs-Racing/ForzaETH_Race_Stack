#!/usr/bin/env python3
"""
Corrupts the simulator's perfect odometry, so the localizers have drift to correct.

Started by sim.launch.py when odom_noise:=<level> is above 0. Gazebo's exact odometry then goes
to /odom_true (and its TF off /tf), and this node publishes /odom and the TF odom -> base_link
from a corrupted copy: each 50 Hz step of the car's motion, taken in the car's own frame, is
distorted and added up from the first true pose. The start is exact and the error grows with
the distance and time driven, like real VESC + IMU odometry.

Error terms at level 1, roughly a real car (VESC speed and IMU yaw rate through an EKF). Each is
multiplied by `level`; any of them can be set directly instead:
  scale_error      0.02            distance reads 2 % long (ERPM gain, tire radius)
  yaw_scale_error  0.02            turns read 2 % too sharp (steering / gyro scale)
  yaw_bias_dps     0.2  [deg/s]    gyro bias: the heading creeps even on a straight
  pos_noise        0.01 [m/sqrt(m)]   random walk in position, along and sideways
  yaw_noise_deg    0.3  [deg/sqrt(m)] random walk in heading
The scale errors and the bias always have the same sign. The random part is seeded (`seed`), so
one seed gives the same noise for every localizer. Nothing changes while the car stands still
(also not the gyro bias), so the start pose stays exact however long the car waits.
Measured at level 1, 1.0 m/s: after one 37 m Victoria lap the odometry alone is ~1.5 m and
10-13 deg off (seeds 1 and 2).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

# level-1 values, see the module docstring
LEVEL1 = {'scale_error': 0.02, 'yaw_scale_error': 0.02, 'yaw_bias_dps': 0.2, 'pos_noise': 0.01,
          'yaw_noise_deg': 0.3}
MOVING = 1e-4  # [m] or [rad] per step: below this the car counts as standing still


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')
        level = self.declare_parameter('level', 1.0).value
        seed = self.declare_parameter('seed', 1).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # NaN default = not set: use the level-1 value times `level`
        t = {k: self.declare_parameter(k, math.nan).value for k in LEVEL1}
        t = {k: v if not math.isnan(v) else LEVEL1[k] * level for k, v in t.items()}
        self.scale, self.yaw_scale = t['scale_error'], t['yaw_scale_error']
        self.yaw_bias = math.radians(t['yaw_bias_dps'])
        self.pos_noise, self.yaw_noise = t['pos_noise'], math.radians(t['yaw_noise_deg'])
        self.rng = np.random.default_rng(seed)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom_true', self.cb, 50)
        self.prev = None  # last true pose (x, y, yaw, t)
        self.est = None   # corrupted pose (x, y, yaw)
        self.get_logger().info(
            f'odometry noise level {level:g} (seed {seed}): distance {self.scale:+.1%}, turns {self.yaw_scale:+.1%}, '
            f'gyro bias {t["yaw_bias_dps"]:.2f} deg/s, random walk {self.pos_noise * 100:.1f} cm and '
            f'{t["yaw_noise_deg"]:.2f} deg per sqrt(m)')

    def cb(self, msg):
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.prev is None or t < self.prev[3]:  # first message, or the sim was reset
            self.est = [x, y, yaw]
        else:
            px, py, pyaw, pt = self.prev
            # true step in the car's previous frame
            c, s = math.cos(pyaw), math.sin(pyaw)
            dx, dy = c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py)
            dyaw = wrap(yaw - pyaw)
            ds = math.hypot(dx, dy)
            if ds > MOVING or abs(dyaw) > MOVING:
                n = self.rng.standard_normal(3) * math.sqrt(ds)
                dx = dx * (1 + self.scale) + self.pos_noise * n[0]
                dy = dy * (1 + self.scale) + self.pos_noise * n[1]
                dyaw = dyaw * (1 + self.yaw_scale) + self.yaw_bias * (t - pt) + self.yaw_noise * n[2]
            # apply the (corrupted) step in the corrupted pose's frame
            ex, ey, eyaw = self.est
            c, s = math.cos(eyaw), math.sin(eyaw)
            self.est = [ex + c * dx - s * dy, ey + s * dx + c * dy, wrap(eyaw + dyaw)]
        self.prev = (x, y, yaw, t)
        self.publish(msg)

    def publish(self, true_msg):
        ex, ey, eyaw = self.est
        qz, qw = math.sin(eyaw / 2), math.cos(eyaw / 2)
        out = Odometry()
        out.header.stamp = true_msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y = ex, ey
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
        out.pose.covariance = true_msg.pose.covariance
        out.twist = true_msg.twist  # body-frame velocities, as measured (left uncorrupted)
        self.pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y = ex, ey
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomNoise()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
