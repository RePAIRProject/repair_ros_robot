#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion
# from sensor_msgs.msg import JointState
import math
from enum import Enum

from repair_interface.srv import *

from typing import Union, List
import numpy as np

from tf.transformations import quaternion_from_euler, quaternion_multiply
from transform_utils import TransformUtils

from manipulation_utils import ManipulationUtils

class ARM_ENUM(Enum):
    ARM_1 = 0
    ARM_2 = 1

class HAND_ENUM(Enum):
    HAND_1 = 0
    HAND_2 = 1

class HAND_STATE_ENUM(Enum):
    OPEN = 0
    CLOSE = 1
    VALUE = 2

class MotionPlannerTest:
    def __init__(self):
        self.mu = ManipulationUtils()
        self.tf_utils = TransformUtils()


    def go_to_place_pose_right_arm(self):

        right_arm_test_pose = PoseStamped()
        right_arm_test_pose.header.frame_id = "world"
        right_arm_test_pose.pose.position.x = -0.13466
        right_arm_test_pose.pose.position.y = -0.59542
        right_arm_test_pose.pose.position.z = 1.0291
        right_arm_test_pose.pose.orientation.x = 0.18639
        right_arm_test_pose.pose.orientation.y = 0.76968
        right_arm_test_pose.pose.orientation.z = -0.083908
        right_arm_test_pose.pose.orientation.w = 0.60483

        self.mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, right_arm_test_pose)

    def go_to_pick_pose_right_arm(self):
        right_arm_test_pose = PoseStamped()
        right_arm_test_pose.header.frame_id = "world"
        right_arm_test_pose.pose.position.x = -0.013786
        right_arm_test_pose.pose.position.y = 0.1573
        right_arm_test_pose.pose.position.z = 1.0141
        right_arm_test_pose.pose.orientation.x = -0.51325
        right_arm_test_pose.pose.orientation.y = 0.55574
        right_arm_test_pose.pose.orientation.z = 0.59477
        right_arm_test_pose.pose.orientation.w = 0.27198

        self.mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2.value, right_arm_test_pose)

    def go_to_pos_left_arm(self):

        left_arm_test_pose = PoseStamped()
        left_arm_test_pose.header.frame_id = "world"
        left_arm_test_pose.pose.position.x = 0.28266
        left_arm_test_pose.pose.position.y = 0.35953
        left_arm_test_pose.pose.position.z = 1.5011
        left_arm_test_pose.pose.orientation.x = -0.05735
        left_arm_test_pose.pose.orientation.y = 0.91022
        left_arm_test_pose.pose.orientation.z = -0.023061
        left_arm_test_pose.pose.orientation.w = 0.40948

        self.mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_1.value, left_arm_test_pose)

    def go_to_pos2_left_arm(self):

        left_arm_test_pose = PoseStamped()
        left_arm_test_pose.header.frame_id = "world"
        left_arm_test_pose.pose.position.x = 0.15975
        left_arm_test_pose.pose.position.y = -0.54494
        left_arm_test_pose.pose.position.z = 1.1949
        left_arm_test_pose.pose.orientation.x = 0.36366
        left_arm_test_pose.pose.orientation.y = 0.66415
        left_arm_test_pose.pose.orientation.z = -0.5178
        left_arm_test_pose.pose.orientation.w = 0.39816

        self.mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_1.value, left_arm_test_pose)

    def test_sliding_guide(self):
        # get the current pose of arm_1_tcp
        current_pose = self.tf_utils.get_link_pose("right_hand_v1_2_research_grasp_link", "world")
        current_pose.pose.position.z += .15
        print(current_pose)
        self.mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2.value, current_pose)

if __name__ == '__main__':
    node_name = "motion_planner_dawnik_test"
    rospy.init_node(node_name)

    motion_planner_test = MotionPlannerTest()
    # rospy.loginfo("Going to pick pose")
    # motion_planner_test.go_to_pick_pose_right_arm()
    # rospy.sleep(3)
    # rospy.loginfo("Going to place pose")
    # motion_planner_test.go_to_place_pose_right_arm()
    # rospy.sleep(3)
    # motion_planner_test.go_to_pos2_right_arm()
    # motion_planner_test.go_to_pos2_left_arm()
    motion_planner_test.test_sliding_guide()
    rospy.loginfo("Exiting...")
    
