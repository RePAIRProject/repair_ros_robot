#!/usr/bin/env python3

from __future__ import print_function

import rospy
import tf
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PoseStamped, Quaternion
#from sensor_msgs.msg import JointState
import math

from repair_interface.srv import *

from typing import Union, List
import numpy as np
import pytransform3d.transformations as pytr

from qbhand_test import QbHand

import time
from moveit_test import ARM_ENUM, HAND_ENUM, HAND_STATE_ENUM
from vision_utils import get_transform, get_hand_tf, publish_tf_np 
from vision_utils import get_pose_stamped_from_arr, get_pose_from_transform


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

    def test_srv(self, target_pose):
        # get current pose
        # rospy.loginfo("Waiting for get current pose service")
        # # rospy.wait_for_service('/get_current_pose_srv')
        # rospy.loginfo("Service found")

        # create service proxy
        # get_current_pose_srv = rospy.ServiceProxy('/get_current_pose_srv', GetCurrentPose)

        # # create request
        # get_current_pose_req = GetCurrentPoseRequest()

        # # call service
        # get_current_pose_resp = get_current_pose_srv(get_current_pose_req)

        # arm1_pose = get_current_pose_resp.current_pose_1
        # arm2_pose = get_current_pose_resp.current_pose_2

        self.go_to_pos_1(target_pose)
        #arm1_pose = get_current_pose_resp.current_pose_1
        # self.go_to_pos_2(arm1_pose)
        # self.send_gripper_command(HAND_ENUM.HAND_1, HAND_STATE_ENUM.VALUE, 1.0)
        # self.go_to_pos_1(arm1_pose)
        #rospy.sleep(1)

    def go_to_pos_2(self, target_pose):
        # target_pose.pose.position.x -= 0.025
        target_pose.pose.position.y -= 0.035
        target_pose.pose.position.z -= 0.13
        # create request
        move_arm_to_pose_req = MoveArmToPoseRequest()
        move_arm_to_pose_req.arm = ARM_ENUM.ARM_1.value
        move_arm_to_pose_req.target_pose = target_pose
        self.move_to_pose(ARM_ENUM.ARM_1, target_pose)

    def go_to_pos_1(self, target_pose):
        # fragment_pose = self.get_fragment_pose()
        # target_pose.pose.position.x = fragment_pose[0]
        # target_pose.pose.position.y = fragment_pose[1]
        # target_pose.pose.position.z = fragment_pose[2] + 0.15

        # euler = tf.transformations.euler_from_quaternion(
        #     [target_pose.pose.orientation.x,
        #      target_pose.pose.orientation.y,
        #      target_pose.pose.orientation.z,
        #      target_pose.pose.orientation.w]
        # )
        # = tf.transformations.quaternion_from_euler(math.pi / 2, -math.pi / 2, euler[2])
        # q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        # target_pose.pose.orientation = Quaternion(*q)
        # create request
        move_arm_to_pose_req = MoveArmToPoseRequest()
        move_arm_to_pose_req.arm = ARM_ENUM.ARM_2.value
        move_arm_to_pose_req.target_pose = target_pose

        self.move_to_pose(ARM_ENUM.ARM_2, target_pose)

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
    node_name = "manual_test"
    rospy.init_node(node_name)

    # Get and print parameters
    side = str(rospy.get_param("/"+node_name+"/side"))
    gazebo = bool(rospy.get_param("/"+node_name+"/gazebo"))

    # print()
    # print("Parameters")
    # print("side =", side)
    # print("gazebo =", gazebo)
    # print()

    # Create QbHand object for controlling the hand
    print('Connecting to qb Soft Hand')

    if side == "right":
        arm_no = 2
    elif side == "left":
        arm_no = 1
    else:
        print("Error:Side value has to be left or right")
        raise ValueError
    
    hand_api = QbHand(side, gazebo)
    print('Connected!')

    # open hand
    hand_api.open_hand()
    print('Opened!')

    tf_hand = get_transform(parent_frame=side+"_hand_v1s_grasp_link", child_frame="arm_"+str(arm_no)+"_tcp")
    # print (tf)

    # get hand orientation
    hand_tf = get_hand_tf()

    #hand_pose_world_np = get_arr_from_pose(hand_pose_world)
    hand_pose_world_np = [0.0, 0.0, 1.15, 0, 0, 0]
    hand_pose_world_np[2] += 0.15
    hand_pose_world_np[3:] = hand_tf
    publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

    hand_arm_transform = pytr.transform_from_pq([tf_hand.transform.translation.x,
                                                      tf_hand.transform.translation.y,
                                                      tf_hand.transform.translation.z,
                                                      tf_hand.transform.rotation.w,
                                                      tf_hand.transform.rotation.x,
                                                      tf_hand.transform.rotation.y,
                                                      tf_hand.transform.rotation.z
                                                     ])

    hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
    T0 = pytr.transform_from_pq(hand_pose_world_np)
    T1 = pytr.concat(hand_arm_transform, T0)

    arm_target_pose_np = get_pose_from_transform(T1)

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    moveit_test = MoveItTest()
    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # tilt hand
    # RPY to convert: 90deg (1.57), Pi/12, -90 (-1.57)
    y_ang = 0.26
    q_rot = quaternion_from_euler(0, y_ang, 0)

    q_orig = arm_target_pose_np[3:]
    q_new = quaternion_multiply(q_rot, q_orig)

    arm_target_pose_np[3:] = q_new
    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # go down to grasp (return to parallel, go down, then rotate again)
    arm_target_pose_np = [-0.114, -0.083, 1.162, 0, 0, 0, 0]
    arm_target_pose_np[3:] = q_new

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # - Translation: [-0.114, -0.083, 1.162]
    # - Rotation: in Quaternion [0.794, 0.004, 0.607, 0.002]
    #             in RPY (radian) [3.112, -1.305, 0.032]
    #             in RPY (degree) [178.304, -74.793, 1.817]


    # close hand
    time.sleep(0.5)
    hand_api.close_hand()
    print('Closed!')

    # # lift hand
    # q_rot = quaternion_from_euler(0, -y_ang, 0)
    # q_orig = arm_target_pose_np[3:]
    # q_new = quaternion_multiply(q_rot, q_orig)
    # arm_target_pose_np[3:] = q_new
    # arm_target_pose_np[2] = arm_target_pose_np[2] + 0.07
    # q_rot = quaternion_from_euler(0, y_ang, 0)
    # q_orig = arm_target_pose_np[3:]
    # q_new = quaternion_multiply(q_rot, q_orig)
    # arm_target_pose_np[3:] = q_new

    # publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    # arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    # print ("Planning trajectory")
    # moveit_test.test_srv(arm_target_pose)

    # lift up
    arm_target_pose_np[:3] = [-0.087, -0.084, 1.335]

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # move side
    arm_target_pose_np[:3] = [-0.087, -0.610, 1.47]

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # go down
    arm_target_pose_np[:3] = [-0.110, -0.609, 1.257]

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # open hand
    hand_api.open_hand()
    print('Opened!')

    # go up
    arm_target_pose_np[:3] = [-0.110, -0.609, 1.345]

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

# lift up
# - Translation: [-0.087, -0.084, 1.335]
# - Rotation: in Quaternion [0.559, 0.564, 0.428, 0.432]
#             in RPY (radian) [1.835, 0.009, 1.573]
#             in RPY (degree) [105.111, 0.536, 90.099]

# # move side
# - Translation: [-0.087, -0.610, 1.337]
# - Rotation: in Quaternion [0.561, 0.562, 0.429, 0.431]
#             in RPY (radian) [1.835, 0.004, 1.570]
#             in RPY (degree) [105.134, 0.205, 89.966]

# # go down
# - Translation: [-0.110, -0.609, 1.257]
# - Rotation: in Quaternion [0.562, 0.562, 0.428, 0.429]
#             in RPY (radian) [1.839, 0.002, 1.570]
#             in RPY (degree) [105.370, 0.106, 89.933]

# open hand

# move up

# At time 1697730613.938
# - Translation: [-0.086, -0.609, 1.345]
# - Rotation: in Quaternion [0.562, 0.563, 0.428, 0.430]
#             in RPY (radian) [1.838, 0.003, 1.570]
#             in RPY (degree) [105.328, 0.174, 89.933]