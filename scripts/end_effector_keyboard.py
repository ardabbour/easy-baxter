#!/usr/bin/env python
"""
    EASY BAXTER
    https://github.com/ardabbour/easy-baxter/

    Abdul Rahman Dabbour
    Cognitive Robotics Laboratory
    Faculty of Engineering and Natural Sciences
    Sabanci University

    Allows user to control Baxter's end effector with a keyboard.
"""

from argparse import ArgumentParser

import rospy

import easy_baxter as eb


def print_help():
    """Prints the key bindings to control the end effector."""

    print "w/W: +x"
    print "s/S: -x"
    print "a/A: +y"
    print "d/D: -y"
    print "z/Z: +z"
    print "x/X: -z\n"

    print "i/I: +roll"
    print "k/K: -roll"
    print "j/J: +pitch"
    print "l/L: -pitch"
    print "n/N: +yaw"
    print "m/M: -yaw\n"

    print "o/O: open gripper"
    print "c/C: close gripper"

    print "p/P: get current pose"


def main(node, arm, initial_pose, prismatic_increment, rotational_increment):
    """Executes the ROS node"""

    print 'Initializing; please wait.'
    rospy.init_node(node)
    eb.enable_robot()
    limb = eb.Arm(arm)

    if not initial_pose:
        pass
    else:
        limb.move_to_pose(initial_pose)

    print "You are now in control; press h for help and Esc to quit."
    while not rospy.is_shutdown():
        char = eb.get_char()
        if char:
            # Esc or ctrl-c to exit
            if char in ['\x1b', '\x03']:
                rospy.signal_shutdown("Example finished.")

            # Prismatic motion
            elif char in ['w', 'W']:
                limb.move_by_increment('x', prismatic_increment)
            elif char in ['s', 'S']:
                limb.move_by_increment('x', -prismatic_increment)
            elif char in ['a', 'A']:
                limb.move_by_increment('y', prismatic_increment)
            elif char in ['d', 'D']:
                limb.move_by_increment('y', -prismatic_increment)
            elif char in ['z', 'Z']:
                limb.move_by_increment('z', prismatic_increment)
            elif char in ['x', 'X']:
                limb.move_by_increment('z', -prismatic_increment)

            # Rotational motion
            elif char in ['i', 'I']:
                limb.move_by_increment('roll', rotational_increment)
            elif char in ['k', 'K']:
                limb.move_by_increment('roll', -rotational_increment)
            elif char in ['j', 'J']:
                limb.move_by_increment('pitch', rotational_increment)
            elif char in ['l', 'L']:
                limb.move_by_increment('pitch', -rotational_increment)
            elif char in ['n', 'N']:
                limb.move_by_increment('yaw', rotational_increment)
            elif char in ['m', 'M']:
                limb.move_by_increment('yaw', -rotational_increment)

            # Gripper actions
            elif char in ['o', 'O']:
                limb.grip.open()
            elif char in ['c', 'C']:
                limb.grip.close()

            # Info
            elif char in ['p', 'P']:
                print limb.get_pose()

            # Help
            elif char in ['h', 'H']:
                print_help()


if __name__ == "__main__":
    PARSER = ArgumentParser()
    PARSER.add_argument("--node", "-n",
                        help="Node name",
                        default="endeffector_keyboard",
                        type=str)
    PARSER.add_argument("--arm", "-a",
                        help="Arm [left/right]",
                        default="left",
                        type=str)
    PARSER.add_argument("--init_pose", "-i",
                        help="Initial pose (6 DOF Cartesian/Euler in list)",
                        default=[],
                        type=list)
    PARSER.add_argument("--pris_inc", "-p",
                        help="Increment in prismatic motion in m",
                        default=0.05,
                        type=int)
    PARSER.add_argument("--rot_inc", "-r",
                        help="Increment in rotational motion in rad",
                        default=0.2,
                        type=int)

    ARGS = PARSER.parse_args()

    main(ARGS.node, ARGS.arm, ARGS.init_pose, ARGS.pris_inc, ARGS.rot_inc)
