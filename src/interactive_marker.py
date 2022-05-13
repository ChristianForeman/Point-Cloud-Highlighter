#!/usr/bin/env python
from std_msgs.msg import Float32MultiArray

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


# Interactive marker class
class IM:
    def __init__(self, frame):
        self.center_pub = rospy.Publisher("center", Float32MultiArray, queue_size=10)
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
        server = InteractiveMarkerServer("DOF_marker")

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.my_frame
        int_marker.name = "DOFMarker"

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

        rospy.spin()


def main():
    rospy.init_node("interactive_marker")
    marker = IM("camera_depth_optical_frame")
    marker.setup_and_run_im()


if __name__ == '__main__':
    main()
