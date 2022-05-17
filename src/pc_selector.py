#!/usr/bin/env python
import argparse
import pathlib
from threading import Thread

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import PointCloud2
import rosbag
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import utils
import numpy as np
import time
import rospy
import sensor_msgs.msg
import sensor_msgs.point_cloud2
from arc_utilities.ros_helpers import get_connected_publisher


class TrajectoryPlaybackGUI(QWidget):
    def __init__(self, default_radius, filename: pathlib.Path):
        super().__init__()
        self.layout = QVBoxLayout(self)

        self.radius_pub = rospy.Publisher("/sel_data/radius", Float32, queue_size=10)
        self.frame_pub = rospy.Publisher("/sel_data/cur_frame", PointCloud2, queue_size=10)

        # Read in the bag
        bag = rosbag.Bag(filename.as_posix())
        self.bag_msgs = []
        for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
            self.bag_msgs.append(msg)
        bag.close()
        print("Finished Reading Bag")

        # Radius Label
        self.label = QLabel('0', self)
        self.label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label.setText("Radius Selector")
        self.layout.addSpacing(15)
        self.layout.addWidget(self.label)

        # Radius slider
        self.rad_slider = QSlider(Qt.Horizontal)
        self.rad_slider.setRange(0, 100)
        self.rad_slider.setValue(int(default_radius * 100))  # The radius passed in is a decimal, setval must be an int
        self.rad_slider.valueChanged.connect(self.radius_change)
        self.layout.addWidget(self.rad_slider)

        # Frame Label
        self.label = QLabel('0', self)
        self.label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label.setText("Frame Selector")
        self.layout.addSpacing(15)
        self.layout.addWidget(self.label)

        # Frame slider
        self.frame_slider = QSlider(Qt.Horizontal)
        self.frame_slider.setRange(0, len(self.bag_msgs) - 1)
        self.frame_slider.setValue(0)
        self.frame_slider.valueChanged.connect(self.frame_change)
        self.layout.addWidget(self.frame_slider)

        # Publish the default point cloud (frame 0)
        self.frame_pub.publish(self.bag_msgs[0])

        self.frame_idx = 0

        # This thread is used for the case where the user unselects the view of the entire frame, and then reselects it
        # The thread periodically publishes the frame, so it reappears without needing to change the selected frame.
        self.pc_pub_thread = Thread(target=self.pc_pub_worker)
        # self.pc_pub_thread.start()  # Uncomment this line if you want rviz to constantly publish the frame (slows
        # things down)

    # Callback for slider change
    def radius_change(self):
        self.radius_pub.publish(self.rad_slider.value() / 100.0)

    # Callback for slider change
    def frame_change(self):
        self.frame_idx = self.frame_slider.value()
        self.frame_pub.publish(self.bag_msgs[self.frame_idx])

    # TODO: This thread slows things down quite a bit, but ideally shouldn't matter when running realtime
    def pc_pub_worker(self):
        while True:
            self.frame_pub.publish(self.bag_msgs[self.frame_idx])
            rospy.sleep(1)


# Interactive marker class
class IM:
    def __init__(self, frame):
        self.center_pub = rospy.Publisher("/sel_data/center", Float32MultiArray, queue_size=10)
        self.my_frame = frame

    # When the im is moved, update the center and notify the highlighter ros node
    def processFeedback(self, feedback):
        p = feedback.pose.position

        new_center = Float32MultiArray()
        new_center.data = [p.x, p.y, p.z]

        self.center_pub.publish(new_center)

    # Referenced from https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel
    # /interactive_marker_tutorials/scripts/basic_controls.py
    def setup_and_run_im(self):
        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("interactive_marker")

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.my_frame
        int_marker.name = "interactive_marker"

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        server.applyChanges()


class Highlighter:
    def __init__(self, frame, default_radius):
        self.frame_id = frame

        self.sel_pub = rospy.Publisher("/selected_pc", PointCloud2, queue_size=10)
        self.pc2_msg = PointCloud2()

        # Set default values
        self.center = np.array([0, 0, 0])
        self.radius = default_radius

        # Sphere for visualization
        self.sphere_pub = rospy.Publisher("/sel_data/sel_sphere", Marker, queue_size=10)

        self.sphere_marker = Marker()
        self.sphere_marker.header.frame_id = self.frame_id
        self.sphere_marker.header.stamp = rospy.Time.now()
        self.sphere_marker.type = Marker.SPHERE
        self.sphere_marker.pose.position.x = 0
        self.sphere_marker.pose.position.y = 0
        self.sphere_marker.pose.position.z = 0
        self.sphere_marker.scale.x = self.radius * 2
        self.sphere_marker.scale.y = self.radius * 2
        self.sphere_marker.scale.z = self.radius * 2
        self.sphere_marker.color.r = 0.0
        self.sphere_marker.color.g = 0.5
        self.sphere_marker.color.b = 0.5
        self.sphere_marker.color.a = 0.3

        self.sphere_pub.publish(self.sphere_marker)

    def highlight(self):
        # t0 = time.perf_counter()
        points = np.array(list(sensor_msgs.point_cloud2.read_points(self.pc2_msg)))
        dist = np.linalg.norm(points[:, :3] - self.center, axis=-1)  # indices 0 to 3 is xyz, the 4th val is rgb
        sel_indices = np.argwhere(dist < self.radius).squeeze(1)
        sel_pc = points[sel_indices]
        # print("Highlight() Runtime:", time.perf_counter() - t0)

        msg = utils.points_to_pc2_msg(sel_pc, self.frame_id)
        self.sel_pub.publish(msg)

    def run(self):
        rospy.Subscriber("/sel_data/cur_frame", PointCloud2, self.handle_new_frame)
        rospy.Subscriber("/sel_data/center", Float32MultiArray, self.handle_new_center)
        rospy.Subscriber("/sel_data/radius", Float32, self.handle_new_radius)

    def handle_new_center(self, new_center):
        # Gets point which is the new center of the point cloud
        self.center = np.array(new_center.data)
        self.sphere_marker.pose.position.x = self.center[0]
        self.sphere_marker.pose.position.y = self.center[1]
        self.sphere_marker.pose.position.z = self.center[2]
        self.sphere_pub.publish(self.sphere_marker)

        self.highlight()

    def handle_new_radius(self, new_radius):
        # Gets the radius which is the range to select the points
        self.radius = new_radius.data

        self.sphere_marker.scale.x = self.radius * 2
        self.sphere_marker.scale.y = self.radius * 2
        self.sphere_marker.scale.z = self.radius * 2
        self.sphere_pub.publish(self.sphere_marker)

        self.highlight()

    def handle_new_frame(self, new_point_cloud: PointCloud2):
        self.pc2_msg = new_point_cloud
        self.highlight()


def main():
    rospy.init_node("pc_selector")

    parser = argparse.ArgumentParser()
    parser.add_argument('radius', type=float)
    parser.add_argument('bag_filename', type=pathlib.Path)

    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    marker = IM("camera_depth_optical_frame")
    marker.setup_and_run_im()

    sel_point_cloud = Highlighter("camera_depth_optical_frame", default_radius=args.radius)
    sel_point_cloud.run()

    app = QApplication([])
    gui = TrajectoryPlaybackGUI(args.radius, args.bag_filename)
    gui.show()
    app.exec()


if __name__ == "__main__":
    main()
