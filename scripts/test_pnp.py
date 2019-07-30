#!/usr/bin/env python
import ast
import numpy as np
import pandas as pd

import rospy
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import easy_baxter as eb
from geometry_msgs.msg import PoseStamped

def normalize(num, lower=-np.pi/2.0, upper=np.pi/2.0, b=False):
    """Normalize number to range [lower, upper) or [lower, upper]."""

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

# Create node
rospy.init_node('test_pnp')

# Define camera poses
org_surface_pose = [0.65, 0.4, 0.15, np.pi, 0.0, 0.0]
new_surface_pose = [0.25, 0.85, 0.15, np.pi, 0.0, 0.0]

# Initialize robot and move left arm to initial pose.
eb.enable_robot()
limb = eb.Arm("left")

plan = pd.read_csv("my_plan.csv")
plan.poseFrom = plan.poseFrom.apply(ast.literal_eval)
plan.poseTo = plan.poseTo.apply(ast.literal_eval)

scene = PlanningSceneInterface()
box_pose = PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.position.x = 0
box_pose.pose.position.y = 10
box_pose.pose.position.z = 0
box_pose.pose.orientation.x = 0
box_pose.pose.orientation.y = 0
box_pose.pose.orientation.z = 0
box_pose.pose.orientation.w = 1
scene.add_box('obstacle', box_pose, size=(1, 1, 0.75))
box_pose2 = box_pose
box_pose2.pose.position.x = 10
box_pose2.pose.position.y = 0
scene.add_box('obstacle2', box_pose2, size=(1, 1, 0.75))

# Execute the rearrangement process
gripper_height = 0.172
x_offset = 0.055
y_offset = -0.005
theta_offset = np.pi/2
for index, step in plan.iterrows():
    pick_x = (step["poseFrom"][1]/10.0) + new_surface_pose[0] + x_offset
    pick_y = (step["poseFrom"][0]/10.0) + new_surface_pose[1] + y_offset
    pick_theta = normalize(step["poseFrom"][2] + theta_offset)

    pick_pose = [pick_x, pick_y, new_surface_pose[2] -
                    gripper_height, np.pi, 0.0, pick_theta]

    place_x = (step["poseTo"][1]/10.0) + org_surface_pose[0] + x_offset
    place_y = (step["poseTo"][0]/10.0) + org_surface_pose[1] + y_offset
    place_theta = normalize(step["poseTo"][2] + theta_offset)

    place_pose = [place_x, place_y, org_surface_pose[2] -
                    gripper_height + 0.01, np.pi, 0.0, place_theta]

    limb.pick_and_place_plan(pick_pose, place_pose, scene)
