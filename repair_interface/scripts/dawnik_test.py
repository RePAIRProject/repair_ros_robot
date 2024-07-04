#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
from tf.transformations import (
    quaternion_from_euler,
    quaternion_multiply,
    euler_from_quaternion,
)
from geometry_msgs.msg import PoseStamped, Quaternion, Pose

# from sensor_msgs.msg import JointState
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
import math
from enum import Enum

from repair_interface.srv import *

import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from typing import Union, List
from scipy.spatial.transform import Rotation as R
import numpy as np

from dawn_ik.msg import IKGoal

from qbhand_test import QbHand

from traj_utils import TrajectoryUtils

import time


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


class DawnIKTest:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.wait_for_transform = 5
        self.transform_tries = 5

        self.traj_utils = TrajectoryUtils()

        namespace = rospy.get_namespace()
        if namespace == "/":
            namespace = "/arm_2/"
        self.dawn_ik_goal_pub = rospy.Publisher(f"{namespace}dawn_ik_solver/ik_goal", IKGoal, queue_size=5)
  

    def test(self):
        # get arm pose
        arm_pose = self.get_arm_pose(ARM_ENUM.ARM_2)

        # gp to pos 1
        self.go_to_pos_1(arm_pose)

    def get_arm_pose(self, arm: ARM_ENUM) -> Union[PoseStamped, None]:
        arm_1_grasp_link = "left_hand_v1s_grasp_link"
        arm_2_grasp_link = "right_hand_v1s_grasp_link"

        pose = None
        if arm == ARM_ENUM.ARM_1:
            pose = self.get_world_pose_from_link(arm_1_grasp_link)
        elif arm == ARM_ENUM.ARM_2:
            pose = self.get_world_pose_from_link(arm_2_grasp_link)
        else:
            rospy.logwarn("Invalid arm")
            return None

        if pose is None:
            rospy.logwarn("Could not get arm pose")
            return None

        return pose

    def go_to_pos_1(self, cuurent_pose: PoseStamped):
        # get fragment pose
        fragment_pose = self.get_fragment_pose()

        # create target pose
        target_pose = PoseStamped()
        target_pose.pose.position.x = fragment_pose[0]
        target_pose.pose.position.y = fragment_pose[1]
        target_pose.pose.position.z = fragment_pose[2] + 0.25

        euler = euler_from_quaternion(
            [
                cuurent_pose.pose.orientation.x,
                cuurent_pose.pose.orientation.y,
                cuurent_pose.pose.orientation.z,
                cuurent_pose.pose.orientation.w,
            ]
        )

        # q = quaternion_from_euler(euler[0], math.pi / 2, euler[2])
        q = quaternion_from_euler(euler[0], math.pi / 2, euler[2])
        target_pose.pose.orientation = Quaternion(*q)

        # create trajectory
        lin_points, rot_quats = self.traj_utils.compute_trajectory(
            cuurent_pose, target_pose
        )

        # publish trajectory
        self.traj_utils.publish_pose_array(lin_points, rot_quats, "world")

        # publish to dawn ik
        self.publish_to_dawn_ik(lin_points, rot_quats)

    def publish_to_dawn_ik(self, lin_points, rot_quats):
        
        pub_rate = rospy.Rate(100)

        for position, quat in zip(lin_points, rot_quats):
            dawn_ik_goal = IKGoal()
            dawn_ik_goal.mode = IKGoal.MODE_1 + IKGoal.MODE_2
            dawn_ik_goal.m1_x = position[0]
            dawn_ik_goal.m1_y = position[1]
            dawn_ik_goal.m1_z = position[2]
            dawn_ik_goal.m1_limit_dist = 0.1
            dawn_ik_goal.m1_weight = 4
            dawn_ik_goal.m2_x = quat[0]
            dawn_ik_goal.m2_y = quat[1]
            dawn_ik_goal.m2_z = quat[2]
            dawn_ik_goal.m2_w = quat[3]
            dawn_ik_goal.m2_weight = 1
            self.dawn_ik_goal_pub.publish(dawn_ik_goal)

            pub_rate.sleep()
        

    def get_world_pose_from_link(
        self, link_name: str
    ) -> Union[PoseStamped, None]:
        pose = PoseStamped()
        pose.header.frame_id = link_name
        pose.header.stamp = rospy.Time(0)

        pose_in_world = self.transformed_pose_with_retries(pose, "world", 5)

        if pose_in_world is None:
            rospy.logwarn("Could not transform pose to world")
            return None

        return pose_in_world

    def get_fragment_pose(self):
        # transform fragment pose from fragment_base_link to world
        fragment_pose = PoseStamped()
        fragment_pose.header.frame_id = "frag3__fragment_base_link"
        fragment_pose.header.stamp = rospy.Time(0)

        fragment_pose_in_world = self.transformed_pose_with_retries(
            fragment_pose, "world", 5
        )

        if fragment_pose_in_world is None:
            rospy.logwarn("Could not transform fragment pose to world")
            return None

        print("Fragment pose in world: ", fragment_pose_in_world.pose.position)
        return [
            fragment_pose_in_world.pose.position.x,
            fragment_pose_in_world.pose.position.y,
            fragment_pose_in_world.pose.position.z,
        ]

    def transformed_pose_with_retries(
        self,
        reference_pose: PoseStamped,
        target_frame: str,
        retries: int = 5,
        execute_arm: bool = False,
        offset: List[float] = [0.0, 0.0, 0.0],
    ) -> Union[PoseStamped, None]:
        """Transform pose with multiple retries

        input reference_pose: The reference pose.
        input target_frame: The name of the taget frame.
        input retries: The number of retries.
        input execute_arm: If true, the pose will be rotated by 180 degrees around the x axis.
        input offset: [r, p, y] offset in rad to be added to the pose if execute_arm is true.

        :return: The updated state.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        transformed_pose = None
        for i in range(0, retries):
            transformed_pose = self.transform_pose(reference_pose, target_frame)
            if transformed_pose:
                break

        if execute_arm:
            # rotate around z axis by 90 degrees
            euler = tf.transformations.euler_from_quaternion(
                [
                    transformed_pose.pose.orientation.x,
                    transformed_pose.pose.orientation.y,
                    transformed_pose.pose.orientation.z,
                    transformed_pose.pose.orientation.w,
                ]
            )
            q = tf.transformations.quaternion_from_euler(
                math.pi + offset[0], offset[1] + euler[1], euler[2] + offset[2]
            )
            transformed_pose.pose.orientation = Quaternion(*q)

        return transformed_pose

    def transform_pose(
        self, reference_pose: PoseStamped, target_frame: str
    ) -> Union[PoseStamped, None]:
        """
        Transforms a given pose into the target frame.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The name of the taget frame.
        :type target_frame: String

        :return: The pose in the target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            # common_time = self.listener.getLatestCommonTime(
            #     target_frame, reference_pose.header.frame_id
            # )

            self.listener.waitForTransform(
                target_frame,
                reference_pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(self.wait_for_transform),
            )
            # reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(
                target_frame,
                reference_pose,
            )

            return transformed_pose

        except tf.Exception as error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None


if __name__ == "__main__":
    node_name = "dawnik_test"
    rospy.init_node(node_name)

    # Get and print parameters
    side = str(rospy.get_param("/" + node_name + "/side", "right"))
    gazebo = bool(rospy.get_param("/" + node_name + "/gazebo", True))

    # Create QbHand object for controlling the hand
    # print("Connecting to qb Soft Hand")

    # if side == "right":
    #     arm_no = 2
    # elif side == "left":
    #     arm_no = 1
    # else:
    #     print("Error:Side value has to be left or right")
    #     raise ValueError

    # hand_api = QbHand(side, gazebo)
    # print("Connected!")

    # # open hand
    # hand_api.open_hand()
    # print("Opened!")

    # Create DawnIKTest object for controlling the arm
    dawn_ik_test = DawnIKTest()

    # move arm to pos 1
    dawn_ik_test.test()
