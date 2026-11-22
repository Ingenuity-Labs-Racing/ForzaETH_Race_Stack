#!/usr/bin/env python3
"""
Draws the car in Foxglove/RViz as a small 3D model at its true pose (/ground_truth), in place of
an arrow. Shapes and sizes copy models/car.sdf: green body, black wheels, a lidar on top and a
yellow strip across the front so the heading is visible from above.

    ros2 run slam_toolbox_sandbox car_marker --ros-args -p scale:=2.0

Parameters:
    scale   size multiplier, 1.0 = true size. Bigger is easier to spot when zoomed out, but the
            car then looks closer to the walls than it really is.

sim.launch.py starts it whenever Foxglove or RViz is on. Publishes /car_marker (MarkerArray).
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

GREEN = (0.24, 0.86, 0.52)  # same green the /ground_truth arrow used: green = true pose
BLACK = (0.1, 0.1, 0.1)
YELLOW = (1.0, 0.85, 0.1)

# Car parts relative to the model origin (= the /ground_truth pose), in metres, from models/car.sdf:
# (name, shape, centre xyz, size xyz, colour, turned sideways)
PARTS = [
    ('body', Marker.CUBE, (-0.08, 0.0, 0.06), (0.45, 0.25, 0.06), GREEN, False),
    ('front', Marker.CUBE, (0.135, 0.0, 0.06), (0.02, 0.25, 0.065), YELLOW, False),
    ('lidar', Marker.CYLINDER, (0.1, 0.0, 0.125), (0.05, 0.05, 0.07), BLACK, False),
    ('wheel_left', Marker.CYLINDER, (0.0, 0.15, 0.05), (0.1, 0.1, 0.04), BLACK, True),
    ('wheel_right', Marker.CYLINDER, (0.0, -0.15, 0.05), (0.1, 0.1, 0.04), BLACK, True),
    ('caster', Marker.SPHERE, (-0.25, 0.0, 0.025), (0.05, 0.05, 0.05), BLACK, False),
]

# Marker cylinders stand upright (axis = z). Wheels roll about y: turn them 90 deg about x.
SIDEWAYS = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))


def quat_mul(a, b):
    """Combines two rotations (quaternions as x, y, z, w): b is applied in the frame already turned by a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    """Rotates vector v by quaternion q."""
    x, y, z, _ = quat_mul(quat_mul(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return x, y, z


class CarMarker(Node):
    def __init__(self):
        super().__init__('car_marker')
        self.scale = self.declare_parameter('scale', 1.0).value
        self.pub = self.create_publisher(MarkerArray, '/car_marker', 10)
        self.create_subscription(Odometry, '/ground_truth', self.pose_cb, 10)

    def pose_cb(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        s = self.scale
        out = MarkerArray()
        for name, shape, centre, size, colour, sideways in PARTS:
            m = Marker()
            m.header = msg.header  # same frame (world) and timestamp as the pose
            m.ns, m.id, m.type, m.action = name, 0, shape, Marker.ADD
            dx, dy, dz = rotate(q, [s * c for c in centre])  # part offset, turned with the car
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p.x + dx, p.y + dy, p.z + dz
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = quat_mul(q, SIDEWAYS) if sideways else q
            m.scale.x, m.scale.y, m.scale.z = (s * c for c in size)
            m.color.r, m.color.g, m.color.b = colour
            m.color.a = 1.0
            out.markers.append(m)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CarMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
