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

class ManipulationUtils:
    def __init__(self):
        self.mp_moveit_topic = "/motion_planner/moveit_py"
        self.mp_dawnik_topic = "/motion_planner/dawnik",

    def move_arm_to_pose_moveit(self, arm: ARM_ENUM, pose: PoseStamped):
        rospy.loginfo("[ManipulationUtils] Waiting for moveit motion planner service...")
        rospy.wait_for_service(self.mp_moveit_topic)
        rospy.loginfo("[ManipulationUtils] Service found!")
        try:
            move_arm_to_pose = rospy.ServiceProxy(self.mp_moveit_topic, MoveArmToPose)

            # create request
            req = MoveArmToPoseRequest()
            req.arm = arm.value
            req.target_pose = pose

            resp = move_arm_to_pose(req)
            # check response
            if resp.success:
                rospy.loginfo("[ManipulationUtils] Moveit motion planner service call successful!")
                return True
            else:
                rospy.logerr("[ManipulationUtils] Moveit motion planner service call failed!")
                return False
        except rospy.ServiceException as e:
            print("[ManipulationUtils] Service call for move_arm_to_pose_moveit failed: %s" % e)
            return False
        
    def move_arm_to_pose_dawnik(self, arm: ARM_ENUM, pose: PoseStamped):
        rospy.loginfo("[ManipulationUtils] Waiting for dawnik motion planner service...")
        rospy.wait_for_service(self.mp_dawnik_topic)
        rospy.loginfo("[ManipulationUtils] Service found!")
        try:
            move_arm_to_pose = rospy.ServiceProxy(self.mp_dawnik_topic, MoveArmToPose)

            # create request
            req = MoveArmToPoseRequest()
            req.arm = arm.value
            req.target_pose = pose

            resp = move_arm_to_pose(req)
            # check response
            if resp.success:
                rospy.loginfo("[ManipulationUtils] Dawnik motion planner service call successful!")
                return True
            else:
                rospy.logerr("[ManipulationUtils] Dawnik motion planner service call failed!")
                return False
        except rospy.ServiceException as e:
            print("[ManipulationUtils] Service call for move_arm_to_pose_dawnik failed: %s" % e)
            return False