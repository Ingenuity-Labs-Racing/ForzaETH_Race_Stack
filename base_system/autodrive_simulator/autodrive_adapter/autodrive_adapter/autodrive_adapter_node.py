import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from tf_transformations import quaternion_multiply

# base_link -> imu is yawed by +90 deg on the car (static_baselink_to_imu in f1tenth_stack/bringup_launch.py)
IMU_MOUNT_QUAT = [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]


class AutodriveAdapter(Node):
    """
    Makes the AutoDRIVE simulator look like the car hardware bringup (f1tenth_stack/bringup_launch.py)
    to the rest of the stack.

    Subscribes to:
        - `/autodrive/roboracer_1/lidar`: simulated LiDAR scan
        - `/autodrive/roboracer_1/imu`: simulated IMU, aligned with the car body, gravity removed
        - `/autodrive/roboracer_1/odom`: simulator odometry, only its body frame velocities are used
        - `/ackermann_cmd`: ackermann_mux output (controller or joystick)

    Publishes to:
        - `/scan`: LiDAR scan in the `laser` frame (urg_node)
        - `/odom`: dead reckoned wheel odometry in the `odom` frame (vesc_to_odom_node)
        - `/sensors/imu/raw`: IMU in the `imu` frame, mounted like the VESC IMU (vesc_driver)
        - `/autodrive/roboracer_1/throttle_command`, `/autodrive/roboracer_1/steering_command`: normalized [-1, 1]
    """

    def __init__(self):
        super().__init__('autodrive_adapter')
        self.speed_to_throttle_gain = self.declare_parameter('speed_to_throttle_gain', 0.0402).value
        self.steering_angle_to_command_gain = self.declare_parameter('steering_angle_to_command_gain', 1.9099).value

        # dead reckoning state, same as vesc_to_odom_node
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_stamp = None

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu/raw', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', 10)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', 10)

        self.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar', self.lidar_cb, 10)
        self.create_subscription(Imu, '/autodrive/roboracer_1/imu', self.imu_cb, 10)
        self.create_subscription(Odometry, '/autodrive/roboracer_1/odom', self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', self.ackermann_cb, 10)

    def lidar_cb(self, msg: LaserScan):
        # no-return rays are already inf, only the frame differs
        msg.header.frame_id = 'laser'
        self.scan_pub.publish(msg)

    def imu_cb(self, msg: Imu):
        # rotate body frame (x forward, y left) into the imu frame: x_imu = y_body, y_imu = -x_body
        msg.header.frame_id = 'imu'
        acc = msg.linear_acceleration
        acc.x, acc.y = acc.y, -acc.x
        gyro = msg.angular_velocity
        gyro.x, gyro.y = gyro.y, -gyro.x
        q = msg.orientation
        q.x, q.y, q.z, q.w = quaternion_multiply([q.x, q.y, q.z, q.w], IMU_MOUNT_QUAT)
        self.imu_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        speed = msg.twist.twist.linear.x
        yaw_rate = msg.twist.twist.angular.z
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_odom_stamp is not None:
            dt = (stamp - self.last_odom_stamp).nanoseconds * 1e-9
            self.x += speed * math.cos(self.yaw) * dt
            self.y += speed * math.sin(self.yaw) * dt
            self.yaw += yaw_rate * dt
        self.last_odom_stamp = stamp

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # same covariances as vesc_to_odom_node
        odom.pose.covariance[0] = 0.2
        odom.pose.covariance[7] = 0.2
        odom.pose.covariance[35] = 0.4
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

    def ackermann_cb(self, msg: AckermannDriveStamped):
        throttle = np.clip(self.speed_to_throttle_gain * msg.drive.speed, -1.0, 1.0)
        steering = np.clip(self.steering_angle_to_command_gain * msg.drive.steering_angle, -1.0, 1.0)
        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.steering_pub.publish(Float32(data=float(steering)))


def main(args=None):
    rclpy.init(args=args)
    node = AutodriveAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
