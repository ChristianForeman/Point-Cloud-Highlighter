#!/usr/bin/env python
import argparse
import pathlib

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from sensor_msgs.msg import PointCloud2
import rosbag
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import utils
import numpy as np
import rospy
import sensor_msgs.msg
import sensor_msgs.point_cloud2
from arc_utilities.ros_helpers import get_connected_publisher


class TrajectoryPlaybackGUI(QWidget):
    def __init__(self, frame, default_radius, filename: pathlib.Path):
        super().__init__()

        # Read in arguments
        self.frame_id = frame
        self.radius = default_radius
        self.bag_path = filename

        # Read in the bag
        bag = rosbag.Bag(filename.as_posix())
        self.bag_msgs = []
        for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
            self.bag_msgs.append(msg)
        bag.close()
        print("Finished Reading Bag")

        # Initialize the default center and point cloud
        self.center = np.array([0, 0, 0])
        self.points = np.array(list(sensor_msgs.point_cloud2.read_points(self.bag_msgs[0])))

        # Set up the sphere marker
        self.sphere_pub = rospy.Publisher("/sel_data/sel_sphere", Marker, queue_size=10)
        self.sphere_marker = Marker()
        self.setup_sphere_marker()

        # Set up the interactive marker
        self.setup_interactive_marker()

        # Set up the publisher for selected points
        self.sel_pub = rospy.Publisher("/selected_pc", PointCloud2, queue_size=10)

        # Frame_counter is used to skip every other message
        self.frame_counter = 0

        # Set up the UI
        self.layout = QVBoxLayout(self)
        self.setup_UI()

        self.frame_pub = rospy.Publisher("/sel_data/cur_frame", PointCloud2, queue_size=10)

        self.frame_idx = 0
        self.frame_pub.publish(self.bag_msgs[self.frame_idx])
        print("Published first frame")

    def setup_sphere_marker(self):
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

    def setup_interactive_marker(self):
        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("interactive_marker")

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
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
        server.insert(int_marker, self.center_change)

        # 'commit' changes and send to all clients
        server.applyChanges()

    def setup_UI(self):
        # Radius Label
        self.label = QLabel('0', self)
        self.label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.label.setText("Radius Selector")
        self.layout.addSpacing(15)
        self.layout.addWidget(self.label)

        # Radius slider
        self.rad_slider = QSlider(Qt.Horizontal)
        self.rad_slider.setRange(0, 100)
        self.rad_slider.setValue(int(self.radius * 100))  # The radius passed in is a decimal, setval must be an int
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

        self.frameNum = QLabel('0', self)
        self.frameNum.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.frameNum.setMinimumWidth(80)

        self.layout.addWidget(self.frameNum)

    def center_change(self, feedback):
        p = feedback.pose.position
        self.center = np.array([p.x, p.y, p.z])

        self.sphere_marker.pose.position.x = self.center[0]
        self.sphere_marker.pose.position.y = self.center[1]
        self.sphere_marker.pose.position.z = self.center[2]
        self.sphere_pub.publish(self.sphere_marker)

        self.select_pc()

    # Callback for slider change
    def radius_change(self):
        self.radius = self.rad_slider.value() / 100.0

        self.sphere_marker.scale.x = self.radius * 2
        self.sphere_marker.scale.y = self.radius * 2
        self.sphere_marker.scale.z = self.radius * 2
        self.sphere_pub.publish(self.sphere_marker)

        self.select_pc()

    # Callback for slider change
    def frame_change(self):
        self.frame_idx = self.frame_slider.value()
        self.frame_pub.publish(self.bag_msgs[self.frame_idx])
        self.points = np.array(list(sensor_msgs.point_cloud2.read_points(self.bag_msgs[self.frame_idx])))
        self.frameNum.setText(str(self.frame_idx))

    def select_pc(self):
        self.frame_counter += 1

        if self.frame_counter % 2 != 0:
            return

        self.frame_counter = 0

        dist = np.linalg.norm(self.points[:, :3] - self.center, axis=-1)  # indices 0 to 3 is xyz, the 4th val is rgb
        sel_indices = np.argwhere(dist < self.radius).squeeze(1)
        sel_pc = self.points[sel_indices]

        msg = utils.points_to_pc2_msg(sel_pc, self.frame_id)
        self.sel_pub.publish(msg)


def main():
    rospy.init_node("pc_selector")

    parser = argparse.ArgumentParser()
    parser.add_argument('radius', type=float)
    parser.add_argument('bag_filename', type=pathlib.Path)

    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    app = QApplication([])
    gui = TrajectoryPlaybackGUI("camera_depth_optical_frame", args.radius, args.bag_filename)
    gui.show()
    app.exec()


if __name__ == "__main__":
    main()
