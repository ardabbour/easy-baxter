#!/usr/bin/env python
"""
    EASY BAXTER
    https://github.com/ardabbour/easy-baxter/

    Abdul Rahman Dabbour
    Cognitive Robotics Laboratory
    Faculty of Engineering and Natural Sciences
    Sabanci University

    Performs simple pick and place.
"""

from argparse import ArgumentParser

import numpy as np

import roslib
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import easy_baxter as eb

PKG = 'easy_baxter'
roslib.load_manifest(PKG)
PKG_DIR = rospkg.RosPack().get_path(PKG)


def normalize(num, lower=0.0, upper=360.0, b=False):
    """Normalize number to range [lower, upper) or [lower, upper]."""

    # abs(num + upper) and abs(num - lower) are needed, instead of
    # abs(num), since the lower and upper limits need not be 0. We need
    # to add half size of the range, so that the final result is lower +
    # <value> or upper - <value>, respectively.
    res = num
    if not b:
        if lower >= upper:
            raise ValueError("Invalid lower and upper limits: (%s, %s)" %
                             (lower, upper))

        res = num
        if num > upper or num == lower:
            num = lower + abs(num + upper) % (abs(lower) + abs(upper))
        if num < lower or num == upper:
            num = upper - abs(num - lower) % (abs(lower) + abs(upper))

        res = lower if res == upper else num
    else:
        total_length = abs(lower) + abs(upper)
        if num < -total_length:
            num += np.ceil(num / (-2 * total_length)) * 2 * total_length
        if num > total_length:
            num -= np.floor(num / (2 * total_length)) * 2 * total_length
        if num > upper:
            num = total_length - num
        if num < lower:
            num = -total_length - num

        res = num * 1.0  # Make all numbers float, to be consistent

    return res


def obtain_arrangement(subscriber):
    """Obtains the arrangment matrix from the specified ROS topic."""

    cubes = rospy.wait_for_message(
        subscriber + "/cubes", numpy_msg(Floats)).data.tolist()
    cuboids = rospy.wait_for_message(
        subscriber + "/cylinders", numpy_msg(Floats)).data.tolist()
    long_cuboids = rospy.wait_for_message(
        subscriber + "/pluses", numpy_msg(Floats)).data.tolist()

    cubes = [round(elem, 2) for elem in cubes]
    cubes = [cubes[x:x+6] for x in range(0, len(cubes), 6)]

    cuboids = [round(elem, 2) for elem in cuboids]
    cuboids = [cuboids[x:x+6] for x in range(0, len(cuboids), 6)]

    long_cuboids = [round(elem, 2) for elem in long_cuboids]
    long_cuboids = [long_cuboids[x:x+6]
                    for x in range(0, len(long_cuboids), 6)]

    return [cubes, cuboids, long_cuboids]


def main(node, subscriber, arm, initial_pose, gripper_height, x_offset, y_offset, theta_offset):
    """Creates a simulation of the arrangement detected by the robot and uses it
    to find an optimal arrangement of more bodies."""

    # Create node
    rospy.init_node(node)

    # Initialize robot and move selected arm to correct pose.
    eb.enable_robot()
    limb = eb.Arm(arm)
    # initial_pose = [0.35, 0.85, 0.15, np.pi, 0.0, 0]
    limb.move_to_pose(initial_pose)

    # Smile!
    eb.display_image(PKG_DIR + '/smiley.jpg')

    arrangement = obtain_arrangement(subscriber)

    # Simply place all objects 0.25m behind where they are
    for group in arrangement:
        for body in group:
            pick_x = (body[1]/100.0) + initial_pose[0] + x_offset
            pick_y = (body[0]/100.0) + initial_pose[1] + y_offset
            pick_theta = normalize(
                body[-1] + theta_offset, -np.pi/2.0, np.pi/2.0)

            pick_pose = [pick_x, pick_y, initial_pose[2] -
                         gripper_height, np.pi, 0.0, pick_theta]
            place_pose = [pick_pose[0]-0, pick_pose[1],
                          initial_pose[2] - gripper_height, np.pi, 0.0, pick_theta]

            limb.pick_and_place(pick_pose, place_pose)
    limb.move_to_pose(initial_pose)

    rospy.signal_shutdown("Shutting down.")


if __name__ == "__main__":
    PARSER = ArgumentParser()
    PARSER.add_argument("--node", "-n",
                        help="Node name.",
                        default="pick_and_place",
                        type=str)
    PARSER.add_argument("--subscriber", "-s",
                        help="ROS Topic to subscribe to.",
                        default="/easy_baxter/arrangement",
                        type=str)
    PARSER.add_argument("--arm", "-a",
                        help="Arm used for sensing, picking, and placing.",
                        default="left",
                        type=str)
    PARSER.add_argument("--initial_pose", "-i",
                        help="Initial (sensing) pose of the arm.",
                        default=[0.25, 0.9, 0.15, np.pi, 0.0, 0.0],
                        type=list)
    PARSER.add_argument("--gripper_height", "-g",
                        help="Distance of end-effector from picking pose in m.",
                        default=0.172,
                        type=float)
    PARSER.add_argument("--x_offset", "-x",
                        help="X offset from camera-detection frame to end-effector frame in m.",
                        default=0.055,
                        type=float)
    PARSER.add_argument("--y_offset", "-y",
                        help="Y offset from camera-detection frame to end-effector frame in m.",
                        default=-0.005,
                        type=float)
    PARSER.add_argument("--theta_offset", "-t",
                        help="Theta offset from camera-detection frame to end-effector frame in m.",
                        default=np.pi/2,
                        type=float)

    ARGS = PARSER.parse_args()

    main(ARGS.node, ARGS.subscriber, ARGS.arm, ARGS.initial_pose,
         ARGS.gripper_height, ARGS.x_offset, ARGS.y_offset, ARGS.theta_offset)

# Main table pose: [0.65, 0.4, 0.15, np.pi, 0.0, 0.0]
# Second table pose: [0.25, 0.9, 0.15, np.pi, 0.0, 0.0]
