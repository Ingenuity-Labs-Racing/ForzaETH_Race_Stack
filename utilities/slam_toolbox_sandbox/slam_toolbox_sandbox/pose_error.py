#!/usr/bin/env python3
"""
Compares the SLAM pose (map -> base_link TF) with Gazebo's ground truth (/ground_truth).

Ground truth is in the Gazebo world frame, so it is first moved into the SLAM map
frame using map_origin_* (the world pose the car had when the map was recorded).

Publishes /slam_eval/position_error and /slam_eval/yaw_error (plot with rqt_plot),
logs a running summary every 2 s and a final summary on Ctrl+C.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseError(Node):
    def __init__(self):
        super().__init__('pose_error')
        self.ox = self.declare_parameter('map_origin_x', 0.0).value
        self.oy = self.declare_parameter('map_origin_y', 0.0).value
        self.oyaw = self.declare_parameter('map_origin_yaw', 0.0).value
        csv_path = self.declare_parameter('csv_path', '').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pos_pub = self.create_publisher(Float64, '/slam_eval/position_error', 10)
        self.yaw_pub = self.create_publisher(Float64, '/slam_eval/yaw_error', 10)
        self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)
        self.create_timer(2.0, self.log_summary)

        self.pos_errs = []
        self.yaw_errs = []
        self.tf_misses = 0
        self.first_gt_logged = False
        self.csv = open(csv_path, 'w') if csv_path else None
        if self.csv:
            self.csv.write('t,gt_x,gt_y,gt_yaw,slam_x,slam_y,slam_yaw,pos_err,yaw_err\n')

    def gt_cb(self, msg: Odometry):
        # ground truth: world frame -> map frame
        wx, wy = msg.pose.pose.position.x, msg.pose.pose.position.y
        wyaw = yaw_from_quat(msg.pose.pose.orientation)
        if not self.first_gt_logged:
            self.get_logger().info(f'First ground truth (world frame): x={wx:.3f} y={wy:.3f} yaw={wyaw:.3f} '
                                   '- should match the spawn pose')
            self.first_gt_logged = True
        dx, dy = wx - self.ox, wy - self.oy
        gx = math.cos(self.oyaw) * dx + math.sin(self.oyaw) * dy
        gy = -math.sin(self.oyaw) * dx + math.cos(self.oyaw) * dy
        gyaw = wrap(wyaw - self.oyaw)

        # SLAM estimate at the same timestamp, else the latest one
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time.from_msg(msg.header.stamp))
        except TransformException:
            self.tf_misses += 1
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except TransformException:
                return
        sx, sy = tf.transform.translation.x, tf.transform.translation.y
        syaw = yaw_from_quat(tf.transform.rotation)

        pos_err = math.hypot(sx - gx, sy - gy)
        yaw_err = abs(wrap(syaw - gyaw))
        self.pos_errs.append(pos_err)
        self.yaw_errs.append(yaw_err)
        self.pos_pub.publish(Float64(data=pos_err))
        self.yaw_pub.publish(Float64(data=yaw_err))
        if self.csv:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.csv.write(f'{t:.3f},{gx:.4f},{gy:.4f},{gyaw:.4f},{sx:.4f},{sy:.4f},{syaw:.4f},'
                           f'{pos_err:.4f},{yaw_err:.4f}\n')

    def summary(self):
        if not self.pos_errs:
            return 'no samples yet - is SLAM publishing map->odom?'
        n = len(self.pos_errs)
        rmse = math.sqrt(sum(e * e for e in self.pos_errs) / n)
        return (f'n={n} | position: now {self.pos_errs[-1]:.3f} m, RMSE {rmse:.3f} m, max {max(self.pos_errs):.3f} m'
                f' | yaw: now {math.degrees(self.yaw_errs[-1]):.2f} deg, max {math.degrees(max(self.yaw_errs)):.2f} deg'
                f' | TF lookups not time-matched {self.tf_misses}')

    def log_summary(self):
        self.get_logger().info(self.summary())


def main():
    rclpy.init()
    node = PoseError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('FINAL: ' + node.summary())
    if node.csv:
        node.csv.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
