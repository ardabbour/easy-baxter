#!/usr/bin/env python
"""
    EASY BAXTER
    https://github.com/ardabbour/easy-baxter/

    Abdul Rahman Dabbour
    Cognitive Robotics Laboratory
    Faculty of Engineering and Natural Sciences
    Sabanci University

    Allows user to calibrate HSV filtering for a camera connected through ROS.
"""

from argparse import ArgumentParser
from itertools import repeat

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import easy_baxter as eb


def nothing(_):
    """Does nothing."""

    pass


def increase_contrast(image):
    """Uses CLAHE (Contrast Limited Adaptive Histogram Equalization) to increase
    the contrast of an image. Found on Stack Overflow, written by Jeru Luke."""

    # Converting image to LAB Color model
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

    # Splitting the LAB image to different channels
    l, a, b = cv2.split(lab)

    # Applying CLAHE to L-channel---
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)

    # Merge the CLAHE enhanced L-channel with the a and b channel
    limg = cv2.merge((cl, a, b))

    # Converting image from LAB Color model to RGB model
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    return final


def main(node_name, subscriber):
    """Creates a camera calibration node and keeps it running."""

    # Initialize node
    rospy.init_node(node_name)

    # Initialize CV Bridge
    bridge = CvBridge()

    # Create a named window to calibrate HSV values in

    cv2.namedWindow('Mask 1', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask 2', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask 1 OR Mask 2', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask 1 AND Mask 2', cv2.WINDOW_NORMAL)

    cv2.resizeWindow('Mask 1', (640,480))
    cv2.resizeWindow('Mask 2', (640,480))
    cv2.resizeWindow('Mask 2 OR Mask 2', (640,480))
    cv2.resizeWindow('Mask 1 AND Mask 2', (640,480))

    # Creating track bars for Mask 1
    cv2.createTrackbar('H_low1', 'Mask 1', 0, 179, nothing)
    cv2.createTrackbar('S_low1', 'Mask 1', 0, 255, nothing)
    cv2.createTrackbar('V_low1', 'Mask 1', 0, 255, nothing)
    cv2.createTrackbar('H_high1', 'Mask 1', 179, 179, nothing)
    cv2.createTrackbar('S_high1', 'Mask 1', 255, 255, nothing)
    cv2.createTrackbar('V_high1', 'Mask 1', 255, 255, nothing)

    # Creating track bars for Mask 2
    cv2.createTrackbar('H_low2', 'Mask 2', 0, 179, nothing)
    cv2.createTrackbar('S_low2', 'Mask 2', 0, 255, nothing)
    cv2.createTrackbar('V_low2', 'Mask 2', 0, 255, nothing)
    cv2.createTrackbar('H_high2', 'Mask 2', 179, 179, nothing)
    cv2.createTrackbar('S_high2', 'Mask 2', 255, 255, nothing)
    cv2.createTrackbar('V_high2', 'Mask 2', 255, 255, nothing)

    # Subscribe to the specified ROS topic and process it continuously
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message(subscriber, Image)
        raw = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        raw = increase_contrast(raw)

        for _ in repeat(None, 4):
            raw = cv2.medianBlur(raw, 5)
        hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

        # To properly calibrate the HSV values for some colors like red,
        # we need two seperate masks joined with an OR operation.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))

        # Get info from trackbars and apply to windows
        h_low1 = cv2.getTrackbarPos('H_low1', 'Mask 1')
        s_low1 = cv2.getTrackbarPos('S_low1', 'Mask 1')
        v_low1 = cv2.getTrackbarPos('V_low1', 'Mask 1')
        h_high1 = cv2.getTrackbarPos('H_high1', 'Mask 1')
        s_high1 = cv2.getTrackbarPos('S_high1', 'Mask 1')
        v_high1 = cv2.getTrackbarPos('V_high1', 'Mask 1')

        h_low2 = cv2.getTrackbarPos('H_low2', 'Mask 2')
        s_low2 = cv2.getTrackbarPos('S_low2', 'Mask 2')
        v_low2 = cv2.getTrackbarPos('V_low2', 'Mask 2')
        h_high2 = cv2.getTrackbarPos('H_high2', 'Mask 2')
        s_high2 = cv2.getTrackbarPos('S_high2', 'Mask 2')
        v_high2 = cv2.getTrackbarPos('V_high2', 'Mask 2')

        # Masking algorithm for mask 1
        lower1 = np.array([h_low1, s_low1, v_low1])
        upper1 = np.array([h_high1, s_high1, v_high1])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask1 = cv2.dilate(mask1, kernel, iterations=2)
        mask1 = cv2.erode(mask1, kernel, iterations=3)
        result_mask1 = cv2.bitwise_and(raw, raw, mask=mask1)

        # Masking algorithm for mask 2
        lower2 = np.array([h_low2, s_low2, v_low2])
        upper2 = np.array([h_high2, s_high2, v_high2])
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask2 = cv2.dilate(mask2, kernel, iterations=2)
        mask2 = cv2.erode(mask2, kernel, iterations=3)
        result_mask2 = cv2.bitwise_and(raw, raw, mask=mask2)

        mask1_or_mask2 = cv2.bitwise_or(mask1, mask2)
        mask1_and_mask2 = cv2.bitwise_and(mask1, mask2)
        result_mask1_or_mask2 = cv2.bitwise_and(raw, raw, mask=mask1_or_mask2)
        result_mask1_and_mask2 = cv2.bitwise_and(
            raw, raw, mask=mask1_and_mask2)

        cv2.imshow('Mask 1', result_mask1)
        cv2.imshow('Mask 2', result_mask2)
        cv2.imshow('Mask 1 OR Mask 2', result_mask1_or_mask2)
        cv2.imshow('Mask 1 AND Mask 2', result_mask1_and_mask2)

        k = cv2.waitKey(10)
        if k == 27:
            rospy.signal_shutdown("Esc pressed. Goodbye.")


if __name__ == "__main__":
    PARSER = ArgumentParser()
    PARSER.add_argument("--node", "-n",
                        help="Node name",
                        default="camera_calibration",
                        type=str)
    PARSER.add_argument("--subscribe", "-s",
                        help="ROS topic to subcribe to",
                        default="/cameras/left_hand_camera/image",
                        type=str)
    ARGS = PARSER.parse_args()

    main(ARGS.node, ARGS.subscribe)
