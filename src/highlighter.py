#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import utils
import math
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from arc_utilities.ros_helpers import get_connected_publisher
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2


class Pc:
    def __init__(self, frame):
        self.my_frame = frame

        self.sel_pub = get_connected_publisher("selected", PointCloud2, queue_size=10)

        # Read in the point cloud TODO Fix declaration!
        self.points = np.array([])

        # Publish the initial points of the cloud
        msg = utils.points_to_pc2_msg(self.points, self.my_frame)

        # Set default values
        self.CENTER = np.array([0, 0, 0])
        self.RADIUS = 1

        # Highlight the starting area
        # self.highlight()

    def highlight(self):
        sel_pc = []
        for point in sensor_msgs.point_cloud2.read_points(self.points):
            cur_point = np.array(point[0:3])
            dist = math.sqrt(np.sum((cur_point - self.CENTER) ** 2))

            if dist < self.RADIUS:
                sel_pc.append(cur_point)
        # If no points in the range, vstack throws an error
        if len(sel_pc) > 0:
            sel_pc = np.vstack(sel_pc)

        msg = utils.points_to_pc2_msg(sel_pc, self.my_frame)
        self.sel_pub.publish(msg)

    def run(self):
        rospy.Subscriber("/current_frame", PointCloud2, self.handle_new_frame)
        rospy.Subscriber("center", Float32MultiArray, self.handle_interactive_marker)
        rospy.Subscriber("radius", Float32, self.handle_slider)
        rospy.spin()

    def handle_interactive_marker(self, point):
        # Gets point which is the new center of the pointcloud
        self.CENTER = np.array(point.data)
        self.highlight()

    def handle_slider(self, radius):
        # Gets the radius which is the range to select the points
        self.RADIUS = radius.data
        self.highlight()

    def handle_new_frame(self, new_pointcloud):
        self.points = new_pointcloud
        self.highlight()


def main():
    rospy.init_node("highlighter")
    my_pointcloud = Pc("camera_depth_optical_frame")
    my_pointcloud.run()


if __name__ == '__main__':
    main()
