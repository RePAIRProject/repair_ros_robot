#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from repair_motion_controller.msg import RepairMoveToAction, RepairMoveToFeedback, RepairMoveToResult, RepairMoveToGoal
import actionlib
# from sensor_msgs.msg import JointState
import math
from enum import Enum

from repair_interface.srv import *

from typing import Union, List

class ARM_ENUM(Enum):
    ARM_1 = 0
    ARM_2 = 1
    BOTH = 2

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
        self.mp_dawnik_topic = "/motion_planner/dawnik"
        self.mp_klampt_topic = "/repair_motion_controller"
        self.klampt_mp_client = actionlib.SimpleActionClient(self.mp_klampt_topic, RepairMoveToAction)

    def move_to_home(self, use_klampt=False):
        left_pose = PoseStamped()
        right_pose = PoseStamped()

        left_pose.pose.position.x = 0.2281434536725418
        left_pose.pose.position.y = 0.2229998203688685
        left_pose.pose.position.z = 1.5244485559609986
        left_pose.pose.orientation.x =  0.3745776186543538
        left_pose.pose.orientation.y =  -0.22078314586194223
        left_pose.pose.orientation.z =  -0.046757647640861
        left_pose.pose.orientation.w =  0.8993109209242547

        right_pose.pose.position.x = 0.21597898714022729
        right_pose.pose.position.y = -0.24652714722018665
        right_pose.pose.position.z = 1.5566318838742945
        right_pose.pose.orientation.x = -0.37453463410597837
        right_pose.pose.orientation.y = -0.22085599528517516
        right_pose.pose.orientation.z = 0.0462163918152999
        right_pose.pose.orientation.w = 0.899338914052578

        success_1 = self.move_arm_to_pose_moveit(ARM_ENUM.ARM_1, left_pose)
        success_2 = self.move_arm_to_pose_moveit(ARM_ENUM.ARM_2, right_pose)
        return success_1 and success_2

    def move_arm_to_pose_moveit(self, arm: ARM_ENUM, pose: PoseStamped):
        #print("planing for arm ", arm)
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
        
    def move_home_klampt(self, dummy_pose:PoseStamped):
        try:
            goal = RepairMoveToGoal()
            goal.arm = 99
            goal.target_pose_left = dummy_pose.pose
            goal.target_time = 5 # sec

            self.klampt_mp_client.send_goal(goal)
            #rospy.loginfo("[ManipulationUtils] Action  goal is sent.")
            rospy.loginfo("[ManipulationUtils] Waiting for action result...")
            self.klampt_mp_client.wait_for_result()

            result = self.klampt_mp_client.get_result()
            if result.success:
                rospy.loginfo("[ManipulationUtils] Klampt motion planner action call successful!")
                return True
            else:
                rospy.logerr("[ManipulationUtils] Klampt motion planner action call failed!")
                return False
        except Exception as e:
            print("[ManipulationUtils] Action call for move_arm_to_pose_klampt failed: %s" % e)
            return False


    def move_sand(self, dummy_pose:PoseStamped):
        try:
            goal = RepairMoveToGoal()
            goal.arm = 88
            goal.target_pose_left = dummy_pose.pose
            goal.target_time = 5 # sec

            self.klampt_mp_client.send_goal(goal)
            #rospy.loginfo("[ManipulationUtils] Action  goal is sent.")
            rospy.loginfo("[ManipulationUtils] Waiting for action result...")
            self.klampt_mp_client.wait_for_result()

            result = self.klampt_mp_client.get_result()
            if result.success:
                rospy.loginfo("[ManipulationUtils] Klampt motion planner action call successful!")
                return True
            else:
                rospy.logerr("[ManipulationUtils] Klampt motion planner action call failed!")
                return False
        except Exception as e:
            print("[ManipulationUtils] Action call for move_arm_to_pose_klampt failed: %s" % e)
            return False
        
    def move_arm_to_pose_klampt(self, arm:ARM_ENUM, pose:PoseStamped, pose_right:PoseStamped=None):
        try:
            #rospy.loginfo("[ManipulationUtils] Waiting for Klampt motion planner Action Service...")
            self.klampt_mp_client.wait_for_server()
            #rospy.loginfo("[ManipulationUtils] Klampt motion planner Action Service is found!")

            goal = RepairMoveToGoal()
            
            if arm.value == 2: #
                goal.arm = 2
                goal.target_pose_left = pose.pose
                goal.target_pose_right = pose_right.pose
                goal.target_time = 5

            if arm.value == 0: # Left Arm
                goal.arm = 0
                goal.target_pose_left = pose.pose
                #rospy.loginfo(f"[ManipulationUtils] left arm pose: {pose}")
                goal.target_time = 5 # sec

            if arm.value == 1: # Right Arm
                goal.arm = 1
                #rospy.loginfo(f"[ManipulationUtils] right arm pose: {pose}")
                goal.target_pose_right = pose.pose
                goal.target_time = 5 # sec
            
            
            print("Sending goal: ",goal)
            print("Self current state klampter: ",self.klampt_mp_client.get_state())
            self.klampt_mp_client.send_goal(goal)
            #rospy.loginfo("[ManipulationUtils] Action  goal is sent.")
            rospy.loginfo("[ManipulationUtils] Waiting for action result...")
            self.klampt_mp_client.wait_for_result()

            result = self.klampt_mp_client.get_result()
            if result.success:
                rospy.loginfo("[ManipulationUtils] Klampt motion planner action call successful!")
                return True, self.multiarray_to_array(result.path)
            else:
                rospy.logerr("[ManipulationUtils] Klampt motion planner action call failed!")
                return False, None

        except Exception as e:
            print("[ManipulationUtils] Action call for move_arm_to_pose_klampt failed: %s" % e)
            return False, None
        
    def move_on_path(self, path):
        goal = RepairMoveToGoal()
        goal.arm = 103
        goal.path = self.array_to_multiarray(path)
        self.klampt_mp_client.send_goal(goal)
        #rospy.loginfo("[ManipulationUtils] Action  goal is sent.")
        rospy.loginfo("[ManipulationUtils] Waiting for action result...")
        self.klampt_mp_client.wait_for_result()

        result = self.klampt_mp_client.get_result()
        if result.success:
            rospy.loginfo("[ManipulationUtils] Klampt motion planner action call successful!")
            return True, self.multiarray_to_array(result.path)
        else:
            rospy.logerr("[ManipulationUtils] Klampt motion planner action call failed!")
            return False, None
        
    def array_to_multiarray(self, arr: np.ndarray) -> Float32MultiArray:
        """
        Convert a 2D numpy array into a ROS Float32MultiArray message.

        Args:
            arr (np.ndarray): Input array of shape (N, M)

        Returns:
            Float32MultiArray: ROS message with flattened data and layout
        """
        if arr.ndim != 2:
            raise ValueError(f"Expected 2D array, got shape {arr.shape}")

        msg = Float32MultiArray()
        msg.data = arr.flatten().tolist()

        # Define layout
        msg.layout.dim.append(MultiArrayDimension(
            label="rows",
            size=arr.shape[0],
            stride=arr.shape[0] * arr.shape[1]
        ))
        msg.layout.dim.append(MultiArrayDimension(
            label="cols",
            size=arr.shape[1],
            stride=arr.shape[1]
        ))

        return msg
        
    def multiarray_to_array(self, msg: Float32MultiArray) -> np.ndarray:
        """
        Convert a ROS Float32MultiArray message back into a 2D numpy array.

        Args:
            msg (Float32MultiArray): ROS message containing flattened data and layout

        Returns:
            np.ndarray: Reconstructed 2D numpy array
        """
        if len(msg.layout.dim) != 2:
            raise ValueError("Expected 2 dimensions in layout, got {}".format(len(msg.layout.dim)))

        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size

        arr = np.array(msg.data, dtype=np.float32).reshape(rows, cols)
        return arr

    def move_to_joint_pose(self, joint_values):
        goal = RepairMoveToGoal()
        goal.arm = 100
        goal.joint_values = joint_values

        self.klampt_mp_client.send_goal(goal)
        #rospy.loginfo("[ManipulationUtils] Action  goal is sent.")
        rospy.loginfo("[ManipulationUtils] Waiting for action result...")
        self.klampt_mp_client.wait_for_result()

        result = self.klampt_mp_client.get_result()
        if result.success:
            rospy.loginfo("[ManipulationUtils] Klampt motion planner action call successful!")
            return True, self.multiarray_to_array(result.path)
        else:
            rospy.logerr("[ManipulationUtils] Klampt motion planner action call failed!")
            return False, None

    def move_out_of_path(self, arm, use_klampt=True):
        pose = PoseStamped()

        if arm == ARM_ENUM.ARM_2:
            #move left
            pose.pose.position.x = 0.2281434536725418
            pose.pose.position.y = 0.4229998203688685
            pose.pose.position.z = 1.5244485559609986
            pose.pose.orientation.x =  0.3745776186543538
            pose.pose.orientation.y =  -0.22078314586194223
            pose.pose.orientation.z =  -0.046757647640861
            pose.pose.orientation.w =  0.8993109209242547
            move_arm = ARM_ENUM.ARM_1

        else:
            pose.pose.position.x = 0.21597898714022729
            pose.pose.position.y = -0.24652714722018665
            pose.pose.position.z = 1.5566318838742945
            pose.pose.orientation.x = -0.37453463410597837
            pose.pose.orientation.y = -0.22085599528517516
            pose.pose.orientation.z = 0.0462163918152999
            pose.pose.orientation.w = 0.899338914052578

            pose.pose.position.x = 0.21597898714022729
            pose.pose.position.y = -0.44652714722018665
            pose.pose.position.z = 1.5566318838742945
            pose.pose.orientation.x = -0.37453463410597837
            pose.pose.orientation.y = -0.22085599528517516
            pose.pose.orientation.z = 0.0462163918152999
            pose.pose.orientation.w = 0.899338914052578
            move_arm = ARM_ENUM.ARM_2
        
        if use_klampt:
            self.move_arm_to_pose_klampt(move_arm, pose)
        else:
            self.move_arm_to_pose_moveit(move_arm, pose)




if __name__ == "__main__":
    rospy.init_node("manipulation_utils_node")
    mu = ManipulationUtils()
    try:
        move_arm_to_pose = rospy.ServiceProxy("/motion_planner/dawnik", MoveArmToPose)

        # create request
        req = MoveArmToPoseRequest()
        req.arm = 1
        req.target_pose = PoseStamped()

        resp = move_arm_to_pose(req)
        # check response
        if resp.success:
            rospy.loginfo("[ManipulationUtils] Dawnik motion planner service call successful!")
        else:
            rospy.logerr("[ManipulationUtils] Dawnik motion planner service call failed!")
    except rospy.ServiceException as e:
        print("[ManipulationUtils] Service call for move_arm_to_pose_dawnik failed: %s" % e)