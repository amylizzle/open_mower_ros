#!/usr/bin/env python3
"""Fake obstacle sensor for the simulator.

Publishes virtual round obstacles as a PointCloud2 on /obstacles/points, the topic the
costmap's obstacle layer listens to (see open_mower/params/costmap_common_params.yaml).

Add obstacles by clicking with rviz's "Publish Point" tool (/clicked_point), or list them
in the ~obstacles parameter as [[x, y], ...] in map coordinates for repeatable tests. Each
obstacle is a disk of ~radius metres (default 0.2).
Publish an empty message on ~clear to remove them all. Already marked costmap cells stay
until the costmap is cleared (rosservice call /move_base_flex/clear_costmaps).
"""

import math

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, Header

SPACING = 0.05     # [m] distance between points, matches the local costmap resolution
HEIGHT = 0.1       # [m] z of the points, inside the obstacle layer's height window


def disk_points(cx, cy, radius):
    steps = int(radius / SPACING)
    for i in range(-steps, steps + 1):
        for j in range(-steps, steps + 1):
            x, y = i * SPACING, j * SPACING
            if math.hypot(x, y) <= radius:
                yield (cx + x, cy + y, HEIGHT)


class SimObstacles:
    def __init__(self):
        self.obstacles = [tuple(o) for o in rospy.get_param("~obstacles", [])]
        self.pub = rospy.Publisher("/obstacles/points", PointCloud2, queue_size=1)
        rospy.Subscriber("/clicked_point", PointStamped, self.on_click)
        rospy.Subscriber("~clear", Empty, self.on_clear)
        rospy.Timer(rospy.Duration(0.2), self.publish)

    def on_click(self, msg):
        if msg.header.frame_id not in ("map", "/map"):
            rospy.logwarn("sim_obstacles: ignoring point in frame '%s', expected 'map'", msg.header.frame_id)
            return
        self.obstacles.append((msg.point.x, msg.point.y))
        rospy.loginfo("sim_obstacles: added obstacle at (%.2f, %.2f)", msg.point.x, msg.point.y)

    def on_clear(self, _msg):
        self.obstacles = []
        rospy.loginfo("sim_obstacles: cleared all obstacles")

    def publish(self, _event):
        radius = float(rospy.get_param("~radius", 0.2))  # read every time so it can be changed while running
        points = [p for (x, y) in self.obstacles for p in disk_points(x, y, radius)]
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))


if __name__ == "__main__":
    rospy.init_node("sim_obstacles")
    SimObstacles()
    rospy.spin()
