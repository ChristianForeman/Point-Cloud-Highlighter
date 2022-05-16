#!/usr/bin/env python
import argparse
import pathlib

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
            break
        bag.close()

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
        self.frame_slider.setRange(0, len(self.bag_msgs))
        self.frame_slider.setValue(0)
        self.frame_slider.valueChanged.connect(self.frame_change)
        self.layout.addWidget(self.frame_slider)

        # Publish the default point cloud (frame 0)
        self.frame_pub.publish(self.bag_msgs[0])

    # Callback for slider change
    def radius_change(self):
        self.radius_pub.publish(self.rad_slider.value() / 100.0)

    # Callback for slider change
    def frame_change(self):
        self.frame_pub.publish(self.bag_msgs[self.frame_slider.value()])


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
    def setup_im(self):
        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("interactive_marker")

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.my_frame
        int_marker.name = "interactive_marker"

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.15
        box_marker.scale.y = 0.15
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)

        # add the control to the interactive marker
        int_marker.controls.append(box_control)

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
        self.my_frame = frame

        self.sel_pub = rospy.Publisher("/selected_pc", PointCloud2, queue_size=10)
        self.pc2_msg = PointCloud2()

        # Set default values
        self.center = np.array([0, 0, 0])
        self.RADIUS = default_radius

        # Highlight the starting area
        self.highlight()

    def highlight(self):
        t0 = time.perf_counter()
        points = np.array(list(sensor_msgs.point_cloud2.read_points(self.pc2_msg)))[:, :3]
        dist = np.linalg.norm(points - self.center, axis=-1)
        sel_indices = np.argwhere(dist < self.RADIUS).squeeze(1)
        sel_pc = points[sel_indices]
        print("Highlight() Runtime:", time.perf_counter() - t0)

        msg = utils.points_to_pc2_msg(sel_pc, self.my_frame)
        self.sel_pub.publish(msg)

    def run(self):
        rospy.Subscriber("/sel_data/cur_frame", PointCloud2, self.handle_new_frame)
        rospy.Subscriber("/sel_data/center", Float32MultiArray, self.handle_interactive_marker)
        rospy.Subscriber("/sel_data/radius", Float32, self.handle_slider)

    def handle_interactive_marker(self, point):
        # Gets point which is the new center of the point cloud
        self.center = np.array(point.data)
        self.highlight()

    def handle_slider(self, radius):
        # Gets the radius which is the range to select the points
        self.RADIUS = radius.data
        self.highlight()

    def handle_new_frame(self, new_point_cloud: PointCloud2):
        self.pc2_msg = new_point_cloud
        self.highlight()


if __name__ == "__main__":
    rospy.init_node("pc_selector")

    parser = argparse.ArgumentParser()
    parser.add_argument('radius', type=float)
    parser.add_argument('bag_filename', type=pathlib.Path)

    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    marker = IM("camera_depth_optical_frame")
    marker.setup_im()

    sel_point_cloud = Highlighter("camera_depth_optical_frame", default_radius=args.radius)
    sel_point_cloud.run()

    app = QApplication([])
    gui = TrajectoryPlaybackGUI(args.radius, args.bag_filename)
    gui.show()
    app.exec()
