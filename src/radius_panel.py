#!/usr/bin/env python
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from std_msgs.msg import Float32
from arc_utilities.ros_helpers import get_connected_publisher
from sensor_msgs.msg import PointCloud2
import rosbag
import sys


class TrajectoryPlaybackGUI(QWidget):
    def __init__(self, default_radius, filename):
        super().__init__()
        self.layout = QVBoxLayout(self)

        self.radius_pub = rospy.Publisher("radius", Float32, queue_size=10)
        self.frame_pub = get_connected_publisher("/current_frame", PointCloud2, queue_size=10)

        # Read in the bag
        bag = rosbag.Bag(filename)
        self.bag_msgs = []
        for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
            self.bag_msgs.append(msg)
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
        # self.rad_slider.setTickPosition(QSlider.TicksBelow)
        # self.rad_slider.setTickInterval(1)
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
        # self.frame_slider.setTickPosition(QSlider.TicksBelow)
        # self.frame_slider.setTickInterval(1)
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


if __name__ == "__main__":
    rospy.init_node("radius_panel")
    def_radius = float(sys.argv[1])
    bag_filename = sys.argv[2]

    app = QApplication([])
    gui = TrajectoryPlaybackGUI(def_radius, bag_filename)
    gui.show()
    app.exec()
