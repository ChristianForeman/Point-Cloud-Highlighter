import rosbag
import rospy
from arc_utilities.ros_helpers import get_connected_publisher
from sensor_msgs.msg import PointCloud2
import argparse


def main():
    rospy.init_node("read_bag")
    # TODO: Add argparse for the name of the bag, maybe make a bash script to run everything?

    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    args = parser.parse_args()

    print(args.filename)

    bag = rosbag.Bag(args.filename)
    bag_msgs = []
    for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
        bag_msgs.append(msg)
    bag.close()

    bag_pub = get_connected_publisher("/current_frame", PointCloud2, queue_size=10)

    # Publish an initial frame
    bag_pub.publish(bag_msgs[0])

    user_msg = "Please select a frame between 0 and " + str(len(bag_msgs) - 1) + ": "

    while True:
        desired_frame = input(user_msg)
        bag_pub.publish(bag_msgs[int(desired_frame)])


if __name__ == '__main__':
    main()
