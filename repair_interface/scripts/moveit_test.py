#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
# from sensor_msgs.msg import JointState
import math
from enum import Enum

from repair_interface.srv import *

from typing import Union, List
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr


from vision_utils import get_transform, get_hand_tf, publish_tf_np
from vision_utils import get_pose_from_arr, get_pose_stamped_from_arr
from vision_utils import get_arr_from_pose, get_point_cloud_from_ros
from vision_utils import segment_table, transform_pose_vislab, get_pose_from_transform


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

class MoveItTest:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.wait_for_transform = 5
        self.transform_tries = 5
        #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    def send_gripper_command(self, hand: HAND_ENUM, hand_state: HAND_STATE_ENUM, value: float = 0.0):
        rospy.loginfo("Waiting for gripper command service")
        rospy.wait_for_service('/gripper_command_srv')
        rospy.loginfo("Service found")

        # create service proxy
        gripper_command_srv = rospy.ServiceProxy('/gripper_command_srv', GripperCommand)

        # create request
        # gripper_command_req = GripperCommandRequest()
        # gripper_command_req.hand = hand.value
        # gripper_command_req.command = hand_state.value
        # gripper_command_req.value = value

        # call service
        gripper_command_resp = gripper_command_srv(gripper_command_req)

        # check response
        if gripper_command_resp.success:
            rospy.loginfo("Successfully sent gripper command")
        else:
            rospy.logwarn("Could not send gripper command")

    def go_to_pos_2(self, target_pose):
        # target_pose.pose.position.x -= 0.025
        target_pose.pose.position.y -= 0.035
        target_pose.pose.position.z -= 0.13
        # create request
        move_arm_to_pose_req = MoveArmToPoseRequest()
        move_arm_to_pose_req.arm = ARM_ENUM.ARM_1.value
        move_arm_to_pose_req.target_pose = target_pose
        self.move_to_pose(ARM_ENUM.ARM_1, target_pose)

    def go_to_pos(self, target_pose):
        move_arm_to_pose_req = MoveArmToPoseRequest()
        move_arm_to_pose_req.arm = ARM_ENUM.ARM_1.value
        move_arm_to_pose_req.target_pose = target_pose
        self.move_to_pose(ARM_ENUM.ARM_1, target_pose)

    def move_to_pose(self, arm: ARM_ENUM, 
                        pose: PoseStamped):
       
        # wait for service
        rospy.loginfo("Waiting for move arm to pose service")
        rospy.wait_for_service('/move_arm_to_pose_srv')
        rospy.loginfo("Service found")

        # create service proxy
        move_arm_to_pose_srv = rospy.ServiceProxy('/move_arm_to_pose_srv', MoveArmToPose)

        # create request
        move_arm_to_pose_req = MoveArmToPoseRequest()
        move_arm_to_pose_req.arm = arm.value
        move_arm_to_pose_req.target_pose = pose

        # call service
        move_arm_to_pose_resp = move_arm_to_pose_srv(move_arm_to_pose_req)

        # check response
        if move_arm_to_pose_resp.success:
            rospy.loginfo("Successfully moved arm to pose")
        else:
            rospy.logwarn("Could not move arm to pose")

    def get_fragment_pose(self):
        # transform fragment pose from fragment_base_link to world
        fragment_pose = PoseStamped()
        fragment_pose.header.frame_id = "frag3__fragment_base_link"
        fragment_pose.header.stamp = rospy.Time(0)

        fragment_pose_in_world = self.transformed_pose_with_retries(fragment_pose, "world", 5)

        if fragment_pose_in_world is None:
            rospy.logwarn("Could not transform fragment pose to world")
            return None
        
        print("Fragment pose in world: ", fragment_pose_in_world.pose.position)
        return [fragment_pose_in_world.pose.position.x,
                fragment_pose_in_world.pose.position.y,
                fragment_pose_in_world.pose.position.z]
    
    def transformed_pose_with_retries(self, reference_pose: PoseStamped, 
                                      target_frame: str,
                                      retries  : int = 5,
                                      execute_arm: bool = False,
                                      offset: List[float] = [0., 0., 0.]) -> Union[PoseStamped, None]:
        """ Transform pose with multiple retries

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
                [transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w]
            )
            q = tf.transformations.quaternion_from_euler(math.pi + offset[0], offset[1]+euler[1], euler[2] + offset[2])
            transformed_pose.pose.orientation = Quaternion(*q)

        return transformed_pose
        
    def transform_pose(self, reference_pose: PoseStamped, 
                       target_frame: str) -> Union[PoseStamped, None]:
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
                target_frame, reference_pose.header.frame_id,
                rospy.Time(0), rospy.Duration(self.wait_for_transform)
            )
            # reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(
                target_frame, reference_pose,
            )

            return transformed_pose

        except tf.Exception as error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None


if __name__ == '__main__':
    rospy.init_node('moveit_test')
    tf_left = get_transform(parent_frame='left_hand_v1s_grasp_link', child_frame='arm_1_tcp')
    # tf_right = get_transform(parent_frame='arm_2_tcp', child_frame='right_hand_v1s_grasp_link')
    print (tf)

    left_hand_arm_transform = pytr.transform_from_pq([tf_left.transform.translation.x,
                                                      tf_left.transform.translation.y,
                                                      tf_left.transform.translation.z,
                                                      tf_left.transform.rotation.w,
                                                      tf_left.transform.rotation.x,
                                                      tf_left.transform.rotation.y,
                                                      tf_left.transform.rotation.z
                                                     ])

    debug = False

    hand_tf = get_hand_tf()

    print('Starting Point Cloud Processing')
    pcd = get_point_cloud_from_ros(debug)

    print ('Table Segmentation')
    table_cloud, object_cloud = segment_table(pcd)

    voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

    object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)
    object_cloud.paint_uniform_color([0, 1, 0])
    table_cloud.paint_uniform_color([1, 0, 0])

    if debug:
        o3d.visualization.draw_geometries([table_cloud, object_cloud])

    initial_pose = np.concatenate((object_cloud.get_center(), hand_tf))
    initial_pose = get_pose_from_arr(initial_pose)

    # Transform the pose from the camera frame to the base frame (world)
    hand_pose_world = transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
    hand_pose_world_np = get_arr_from_pose(hand_pose_world)
    hand_pose_world_np[2] += 0.15
    hand_pose_world_np[3:] = hand_tf
    publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

    hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
    T0 = pytr.transform_from_pq(hand_pose_world_np)
    T1 = pytr.concat(left_hand_arm_transform, T0)

    arm_target_pose_np = get_pose_from_transform(T1)

    # arm_target_pose_np = get_arr_from_pose(arm_target_pose)
    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    moveit_test = MoveItTest()
    print ("Planning trajectory")
    moveit_test.go_to_pos(arm_target_pose)
