#!/usr/bin/env python
"""
    EASY BAXTER
    https://github.com/ardabbour/easy-baxter/

    Abdul Rahman Dabbour
    Cognitive Robotics Laboratory
    Faculty of Engineering and Natural Sciences
    Sabanci University

    Publishes an external camera's feed camera feed to a ROS topic.
"""

from argparse import ArgumentParser

import cv2

import rospy
import roslib
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import easy_baxter as eb

PKG = 'easy_baxter'
roslib.load_manifest(PKG)
PKG_DIR = rospkg.RosPack().get_path(PKG)


def main(node, publisher, camera_index, camera_resolution):
    """Creates an external camera node."""

    # Initialize node
    rospy.init_node(node)

    # Smile!
    eb.display_image(PKG_DIR + '/smiley.jpg')

    # Create publisher
    pub = rospy.Publisher(publisher, Image, queue_size=1)

    # Set camera source
    cam_feed = cv2.VideoCapture(camera_index)
    cam_feed.set(3, camera_resolution[0])
    cam_feed.set(4, camera_resolution[1])

    # Initialize CV Bridge
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Capture camera input frame-by-frame
        ret, frame = cam_feed.read()

        if ret:
            # Convert frame to a ROS image message.
            frame = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish frame
            pub.publish(frame)


if __name__ == "__main__":
    PARSER = ArgumentParser()
    PARSER.add_argument("--node", "-n",
                        help="Node name",
                        default="external_camera_publisher",
                        type=str)
    PARSER.add_argument("--publisher", "-p",
                        help="Publisher path",
                        default="/easy_baxter/external_camera/raw",
                        type=str)
    PARSER.add_argument("--index", "-i",
                        help="Camera index no.",
                        default=1,
                        type=int)
    PARSER.add_argument("--resolution", "-r",
                        help="Camera resolution as a tuple",
                        default=(1920, 1080),
                        type=tuple)

    ARGS = PARSER.parse_args()

    main(ARGS.node, ARGS.publisher, ARGS.index, ARGS.resolution)
