#!/usr/bin/env python
import time

import rospy
import sensor_msgs.msg
import utils
import math
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from arc_utilities.ros_helpers import get_connected_publisher
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import argparse
import sys


class Highlighter:
    def __init__(self, frame, default_radius):
        self.my_frame = frame

        self.sel_pub = rospy.Publisher("/selected_pc", PointCloud2, queue_size=10)

        self.pc2_msg = PointCloud2()

        # Set default values
        self.center = np.array([0, 0, 0])
        self.RADIUS = default_radius

        # Highlight the starting area
        # self.highlight()

    def highlight(self):
        t0 = time.perf_counter()
        points = np.array(list(sensor_msgs.point_cloud2.read_points(self.pc2_msg)))[:, :3]
        dist = np.linalg.norm(points - self.center, axis=-1)
        sel_indices = np.argwhere(dist < self.RADIUS).squeeze(1)
        sel_pc = points[sel_indices]
        print(time.perf_counter() - t0)

        msg = utils.points_to_pc2_msg(sel_pc, self.my_frame)
        self.sel_pub.publish(msg)

    def run(self):
        rospy.Subscriber("/sel_data/cur_frame", PointCloud2, self.handle_new_frame)
        rospy.Subscriber("/sel_data/center", Float32MultiArray, self.handle_interactive_marker)
        rospy.Subscriber("/sel_data/radius", Float32, self.handle_slider)
        rospy.spin()

    def handle_interactive_marker(self, point):
        # Gets point which is the new center of the pointcloud
        self.center = np.array(point.data)
        self.highlight()

    def handle_slider(self, radius):
        # Gets the radius which is the range to select the points
        self.RADIUS = radius.data
        self.highlight()

    def handle_new_frame(self, new_pointcloud: PointCloud2):
        self.pc2_msg = new_pointcloud
        self.highlight()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('radius', type=float)

    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    rospy.init_node("highlighter")
    my_pointcloud = Highlighter("camera_depth_optical_frame", default_radius=args.radius)
    my_pointcloud.run()


if __name__ == '__main__':
    main()
