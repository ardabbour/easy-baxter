#!/usr/bin/env python
"""
    BAXTER IMAGINATION
    https://github.com/ardabbour/baxter-imagination/

    Abdul Rahman Dabbour
    Cognitive Robotics Laboratory
    Faculty of Engineering and Natural Sciences
    Sabanci University

    Simulates the surface as captured by the image processor.
"""

from argparse import ArgumentParser

import roslib
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy as np
import pybullet as p

import simple_baxter as sb

PKG = 'baxter_imagination'
roslib.load_manifest(PKG)
rospack = rospkg.RosPack()
PKG_DIR = rospack.get_path(PKG)
URDF_DIR = PKG_DIR + "/data/models/basic/"
ARM = 'left'
POSE = [0.65, 0.4, 0.15, np.pi, 0.0, 0.0]


def obtain_arrangement(subscriber):
    """Obtains the arrangment matrix from the specified ROS topic."""


    cubes = rospy.wait_for_message(
        subscriber + "/cubes", numpy_msg(Floats)).data.tolist()
    cuboids = rospy.wait_for_message(
        subscriber + "/cuboids", numpy_msg(Floats)).data.tolist()
    long_cuboids = rospy.wait_for_message(
        subscriber + "/long_cuboids", numpy_msg(Floats)).data.tolist()

    cubes = [round(elem, 2) for elem in cubes]
    cubes = [cubes[x:x+6] for x in range(0, len(cubes), 6)]

    cuboids = [round(elem, 2) for elem in cuboids]
    cuboids = [cuboids[x:x+6] for x in range(0, len(cuboids), 6)]

    long_cuboids = [round(elem, 2) for elem in long_cuboids]
    long_cuboids = [long_cuboids[x:x+6]
                    for x in range(0, len(long_cuboids), 6)]

    return [cubes, cuboids, long_cuboids]


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
            file_name = URDF_DIR + "cube_act.urdf"
            color = [0, 0, 1, 1]
        elif i == 1:
            file_name = URDF_DIR + "cuboid_act.urdf"
            color = [1, 1, 0, 1]
        elif i == 2:
            file_name = URDF_DIR + "cuboid_long_act.urdf"
            color = [1, 0, 0, 1]
        for j in val:
            loaded = p.loadURDF(
                fileName=file_name,
                basePosition=j[0:3],
                baseOrientation=p.getQuaternionFromEuler(j[3:6]))
            p.changeVisualShape(loaded, -1, rgbaColor=color)
            loaded_body_ids.append(loaded)

    return loaded_body_ids


def main(subscriber):
    """Creates a simulation of the arrangement detected by the robot and uses it
    to find an optimal arrangement of more bodies."""

    # Create node
    rospy.init_node('baxter_imagination')

    # Initialize robot and move selected arm to correct pose.
    sb.enable_robot()
    limb = sb.Arm(ARM)
    limb.move_to_pose(POSE)

    # Smile!
    print PKG_DIR + '/smiley.jpg'
    sb.display_image(PKG_DIR + '/smiley.jpg')

    # Initialize physics engine.
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.resetDebugVisualizerCamera(3, 180, -89.99, [0, 0, 0])
    surface = p.loadURDF(URDF_DIR + "surface_act.urdf", useFixedBase=True)
    p.changeVisualShape(surface, -1, rgbaColor=[1, 1, 1, 1])

    arrangement = obtain_arrangement(subscriber)
    loaded_body_ids = create_sim(arrangement)

    rate = rospy.Rate(10)
    flag = False
    while not rospy.is_shutdown():
        char = sb.get_char()
        if char in ['\x1b', '\x03']:
            rospy.signal_shutdown("Shutting down.")
        if (char == 'f' and flag == False): # Freeze refreshing on simulation
            flag = True
        if (char == 'u' and flag == True): # Unfreeze refreshing on simulation
            flag = False
        if not flag:            
            arrangement = obtain_arrangement(subscriber)
            for i in loaded_body_ids:
                p.removeBody(i)
            loaded_body_ids = create_sim(arrangement)
        rate.sleep()

    p.disconnect()


if __name__ == "__main__":
    PARSER = ArgumentParser()
    PARSER.add_argument("--subscriber", "-s",
                        help="ROS Topic to subscribe to.",
                        default="/baxter_imagination/arrangement",
                        type=str)

    ARGS = PARSER.parse_args()

    main(ARGS.subscriber)
