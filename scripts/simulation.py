#!/usr/bin/env python
"""
    EASY BAXTER
    https://github.com/ardabbour/easy-baxter/

    Abdul Rahman Dabbour
    Cognitive Robotics Laboratory
    Faculty of Engineering and Natural Sciences
    Sabanci University

    Simulates the environment as captured by the image processor.
"""

from argparse import ArgumentParser

import pybullet as p

import roslib
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import easy_baxter as eb

PKG = 'easy_baxter'
roslib.load_manifest(PKG)
PKG_DIR = rospkg.RosPack().get_path(PKG)
URDF_DIR = PKG_DIR + "/data/models/basic/"


def obtain_arrangement(subscriber):
    """Obtains the arrangment matrix from the specified ROS topic."""

    cubes = rospy.wait_for_message(
        subscriber + "/cubes", numpy_msg(Floats)).data.tolist()
    cylinders = rospy.wait_for_message(
        subscriber + "/cylinders", numpy_msg(Floats)).data.tolist()
    pluses = rospy.wait_for_message(
        subscriber + "/pluses", numpy_msg(Floats)).data.tolist()

    cubes = [round(elem, 2) for elem in cubes]
    cubes = [cubes[x:x+6] for x in range(0, len(cubes), 6)]

    cylinders = [round(elem, 2) for elem in cylinders]
    cylinders = [cylinders[x:x+6] for x in range(0, len(cylinders), 6)]

    pluses = [round(elem, 2) for elem in pluses]
    pluses = [pluses[x:x+6]
              for x in range(0, len(pluses), 6)]

    return [cubes, cylinders, pluses]


def create_sim(arrangement):
    """Creates a simulation given the arrangement of the objects."""

    loaded_body_ids = []
    for i in arrangement:
        for j in i:
            j[0] = round((j[0] / -10.0), 2)
            j[1] = round((j[1] / 10.0), 2)
            j[2] = round((j[2] / 10.0), 2)
            j[5] = round((j[5] + 1.57), 2)
    for i, val in enumerate(arrangement):
        if i == 0:
            file_name = URDF_DIR + "plus.urdf"
            color = [0, 1, 0, 1]
        elif i == 1:
            file_name = URDF_DIR + "cylinder.urdf"
            color = [1, 1, 0, 1]
        elif i == 2:
            file_name = URDF_DIR + "cube.urdf"
            color = [1, 0, 0, 1]
        for j in val:
            loaded = p.loadURDF(
                fileName=file_name,
                basePosition=j[0:3],
                baseOrientation=p.getQuaternionFromEuler(j[3:6]))
            p.changeVisualShape(loaded, -1, rgbaColor=color)
            loaded_body_ids.append(loaded)

    return loaded_body_ids


def main(node, subscriber):
    """Creates a simulation of the arrangement detected by the robot and uses it
    to find an optimal arrangement of more bodies."""

    # Create node
    rospy.init_node(node)

    # Smile!
    eb.display_image(PKG_DIR + '/smiley.jpg')

    # Initialize physics engine and environment.
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.resetDebugVisualizerCamera(3, 180, -89.99, [0, 0, 0])
    surface = p.loadURDF(URDF_DIR + "surface.urdf", useFixedBase=True)
    p.changeVisualShape(surface, -1, rgbaColor=[1, 1, 1, 1])

    arrangement = obtain_arrangement(subscriber)
    loaded_body_ids = create_sim(arrangement)

    rate = rospy.Rate(10)
    flag = False
    while not rospy.is_shutdown():
        char = eb.get_char(5)
        if char in ['\x1b', '\x03']:
            p.disconnect()
            rospy.signal_shutdown("Shutting down.")
        if char == 'f':  # Freeze refreshing on simulation
            flag = True
        if not flag:
            arrangement = obtain_arrangement(subscriber)
            for i in loaded_body_ids:
                p.removeBody(i)
            loaded_body_ids = create_sim(arrangement)
        rate.sleep()

if __name__ == "__main__":
    PARSER = ArgumentParser()
    PARSER.add_argument("--node", "-n",
                        help="Node name.",
                        default="simulation",
                        type=str)
    PARSER.add_argument("--subscriber", "-s",
                        help="ROS Topic to subscribe to.",
                        default="/easy_baxter/arrangement",
                        type=str)

    ARGS = PARSER.parse_args()

    main(ARGS.node, ARGS.subscriber)
