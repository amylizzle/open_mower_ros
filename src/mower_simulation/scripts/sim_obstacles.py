#!/usr/bin/env python3
"""Fake obstacle sensor for the simulator.

Reports virtual round obstacles, one PointCloud2 on /obstacles/new per obstacle. mower_comms_v2
keeps them and publishes them on /obstacles/points for the costmap.

Add obstacles by clicking with rviz's "Publish Point" tool (/clicked_point). Each obstacle is a
disk of ~radius metres (default 0.2). To remove them, publish an empty message on /obstacles/clear.
"""

import math

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

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
        self.pub = rospy.Publisher("/obstacles/new", PointCloud2, queue_size=10)
        rospy.Subscriber("/clicked_point", PointStamped, self.on_click)

    def on_click(self, msg):
        if msg.header.frame_id not in ("map", "/map"):
            rospy.logwarn("sim_obstacles: ignoring point in frame '%s', expected 'map'", msg.header.frame_id)
            return
        self.report(msg.point.x, msg.point.y)

    def report(self, x, y):
        radius = float(rospy.get_param("~radius", 0.2))  # read every time so it can be changed while running
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, list(disk_points(x, y, radius))))
        rospy.loginfo("sim_obstacles: obstacle at (%.2f, %.2f), radius %.2f", x, y, radius)


if __name__ == "__main__":
    rospy.init_node("sim_obstacles")
    SimObstacles()
    rospy.spin()
