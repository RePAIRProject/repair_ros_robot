#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray
from std_msgs.msg import Int32MultiArray, Float32MultiArray

# from sensor_msgs.msg import JointState
import math
from enum import Enum

from repair_interface.srv import *

from attach_objects import attach_links, detach_links

from typing import Union, List
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from vision_utils import get_transform, get_hand_tf, publish_tf_np
from vision_utils import get_pose_from_arr, get_pose_stamped_from_arr
from vision_utils import get_arr_from_pose
from vision_utils import transform_pose_vislab, get_pose_from_transform
from vision_utils import segment_table, get_number_of_frescos, get_max_cluster, check_frescos_left

from qbhand_test import QbHand
#from moveit_test import MoveItTest

from manipulation_utils import ManipulationUtils, ARM_ENUM

from scipy.spatial.transform import Rotation as R
import angle_utils
import copy

import message_filters

from repair_interface.msg import RecognitionData, PlacementData

initial_pose_left = pytr.transform_from_pq([0.18584, 0.47267, 1.345, -0.15708, 0.97996, 0.12039, 0.022494])
initial_pose_right = pytr.transform_from_pq([0.18584, -0.47267, 1.345, 0.158, 0.98476, -0.071265, 0.014615])


class PicpkNPlaceDemo:
    def __init__(self, debug=False):
        # right hand
        # - Translation: [0.232, -0.114, 1.076]
        #- Rotation: in Quaternion [0.130, 0.755, -0.226, 0.601]

        # left hand
        #- Translation: [0.083, 0.518, 1.112]
        #- Rotation: in Quaternion [-0.064, 0.740, 0.276, 0.609]

        self.mu = ManipulationUtils()
        self.debug = debug
        self.use_gazebo = True
        self.gazebo_attached = False
        self.use_pyrealsense = False
        self.use_hands = True
        self.use_fragment_alignment = False
        self.fragment_pose_list = []
        self.fragment_ids_list = []
        self.fragment_rotations_list = []
        self.grasp_without_rotation = True
        if self.grasp_without_rotation == True:
            print("#" * 50)
            print('WARNING:\nWe are ignoring the rotation, as we set `grasp_without_rotation` to True!')
            print("#" * 50)
        self.follow_the_hand = True # it will choose placement side based on the hand and not on the published data
        if self.follow_the_hand == True:
            print("#" * 50)
            print('WARNING:\nWe are choosing the placement side based on the hand and not on the published data!')
            print("Set `follow_the_hand` to False to remove this and follow group-wise placement.")
            print("#" * 50)
        self.hardcoded_placement = False
        self.placement_pose_array_list = []
        self.placement_rotation_list = []
        self.placement_side_list = []
        self.use_wide_hand_grasping_list = []
        self.min_z_value_arm_1 = 1.137
        self.min_z_value_arm_2 = 1.076
        self.use_klampt = True
        if self.use_hands:
            self.hand_api_right = QbHand('right', self.use_gazebo)
            self.hand_api_left = QbHand('left', self.use_gazebo)
            print("self.hand_api left: ",self.hand_api_left)
            #self.moveit = MoveItTest()
            self.setup_hands()

        self.recognition_data_sub = rospy.Subscriber('/recognition/recognition_data', RecognitionData, self.recognition_data_callback)
        self.placement_data_sub = rospy.Subscriber('/recognition/placement_data', PlacementData, self.placement_data_callback)

        #self.recognition_data_sub = rospy.Subscriber('/recognition/recognition_data', RecognitionData)
        #self.placement_data_sub = rospy.Subscriber('/recognition/placement_data', PlacementData)

        # Synchronize the two subscribers with an ApproximateTimeSynchronizer
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.recognition_data_sub, self.placement_data_sub], 10, 0.1)  # 0.1 is the tolerance (in seconds)
        #self.ts.registerCallback(self.joint_fresco_callback)

        #self.fragment_pose_sub = rospy.Subscriber('/recognition/points', PoseArray, self.fragment_pose_callback)
        #self.fragment_id_sub = rospy.Subscriber('/recognition/ids', Int32MultiArray, self.fragment_ids_callback)
        ##self.fragment_rotation_sub = rospy.Subscriber('/recognition/rotations', Float32MultiArray, self.fragment_rotations_callback)
        #self.placement_pose_array_sub = rospy.Subscriber('/placement/positions', PoseArray, self.placement_pose_array_callback)
        #self.placement_rotation_sub = rospy.Subscriber('/placement/rotations', Float32MultiArray, self.placement_rotation_list_callback)
        #self.placement_side_sub = rospy.Subscriber('/placement/side', Int32MultiArray, self.placement_side_list_callback)
        #self.use_wide_hand_grasping_sub = rospy.Subscriber('/grasping/use_wide_hand', Int32MultiArray, self.use_wide_hand_callback)

    def reset_manipulation_utils(self):
        del self.mu
        self.mu = ManipulationUtils()

    #### 
    # Joint Callback
    ####    

    def recognition_data_callback(self, recognition_data, placement_data):
        print("Omega cool")
        # stores the received 'ids' data into self.fragment_ids_list
        self.fragment_ids_list = []
        self.fragment_ids_list = list(recognition_data.id_array.data)
        
        # stores the received 'rotations' data into self.fragment_rotations_list
        self.fragment_rotations_list = []
        if self.grasp_without_rotation == True:
            self.fragment_rotations_list = [0] * len(recognition_data.rotation_array.data)
        else:
            self.fragment_rotations_list = list(recognition_data.rotation_array.data)

        # stores the received 'poses' data into self.fragment_pose_list
        self.fragment_pose_list = []
        for pose in recognition_data.pose_array.poses:
            position = pose.position
            orientation = pose.orientation
            numpy_pose = np.array([
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.fragment_pose_list.append(numpy_pose)

        # stores the received 'rotations' data into self.fragment_rotations_list
        self.use_wide_hand_grasping_list = []
        self.use_wide_hand_grasping_list = list(recognition_data.use_wide_hand.data)   

        # stores the received 'poses' data into self.placement_pose_array_list
        self.placement_pose_array_list = []
        for pose in placement_data.placement_pose_array.poses:
            position = pose.position
            orientation = pose.orientation
            numpy_pose = np.array([
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.placement_pose_array_list.append(numpy_pose)

        # This callback stores the received 'rotations' data into self.fragment_rotations_list
        self.placement_rotation_list = []
        self.placement_rotation_list = list(placement_data.placement_rotation.data)

        # This callback stores the received 'side' data into self.placement_side_list
        self.placement_side_list = []
        self.placement_side_list = list(placement_data.placement_side.data) 



    #### 
    # Recognition
    ####

    def recognition_data_callback(self, recognition_data):
        print("Super cool")
        # stores the received 'ids' data into self.fragment_ids_list
        self.fragment_ids_list = []
        self.fragment_ids_list = list(recognition_data.id_array.data)
        
        # stores the received 'rotations' data into self.fragment_rotations_list
        self.fragment_rotations_list = []
        if self.grasp_without_rotation == True:
            self.fragment_rotations_list = [0] * len(recognition_data.rotation_array.data)
        else:
            self.fragment_rotations_list = list(recognition_data.rotation_array.data)

        # stores the received 'poses' data into self.fragment_pose_list
        self.fragment_pose_list = []
        print("recognition_data.pose_array.poses",recognition_data.pose_array.poses)
        for pose in recognition_data.pose_array.poses:
            #print("pose.position recognation data: ", pose.position)
            position = pose.position
            orientation = pose.orientation
            numpy_pose = np.array([
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.fragment_pose_list.append(numpy_pose)

        # stores the received 'rotations' data into self.fragment_rotations_list
        self.use_wide_hand_grasping_list = []
        self.use_wide_hand_grasping_list = list(recognition_data.use_wide_hand.data)   


    #### 
    # PLACEMENT
    ####   

    def placement_data_callback(self, placement_data):
        # stores the received 'poses' data into self.placement_pose_array_list
        self.placement_pose_array_list = []
        for pose in placement_data.placement_pose_array.poses:
            position = pose.position
            orientation = pose.orientation
            numpy_pose = np.array([
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.placement_pose_array_list.append(numpy_pose)

        # This callback stores the received 'rotations' data into self.fragment_rotations_list
        self.placement_rotation_list = []
        self.placement_rotation_list = list(placement_data.placement_rotation.data)

        # This callback stores the received 'side' data into self.placement_side_list
        self.placement_side_list = []
        self.placement_side_list = list(placement_data.placement_side.data) 


    # def fragment_ids_callback(self, fragment_ids):
    #     self.fragment_ids_list = []
    #     # This callback stores the received 'ids' data into self.fragment_ids_list
    #     self.fragment_ids_list = list(fragment_ids.data)


    # def fragment_rotations_callback(self, fragment_rotations):
    #     self.fragment_rotations_list = []
    #     # This callback stores the received 'rotations' data into self.fragment_rotations_list
    #     if self.grasp_without_rotation == True:
    #         self.fragment_rotations_list = [0] * len(fragment_rotations.data)
    #     else:
    #         self.fragment_rotations_list = list(fragment_rotations.data)


    # def fragment_pose_callback(self, pose_array):
    #     self.fragment_pose_list = []
    #     for pose in pose_array.poses:
    #         position = pose.position
    #         orientation = pose.orientation
    #         numpy_pose = np.array([
    #             position.x, position.y, position.z,
    #             orientation.x, orientation.y, orientation.z, orientation.w
    #         ])
    #         self.fragment_pose_list.append(numpy_pose)

    #### 
    # PLACEMENT
    ####
    # def placement_pose_array_callback(self, pose_array):
    #     self.placement_pose_array_list = []
    #     for pose in pose_array.poses:
    #         position = pose.position
    #         orientation = pose.orientation
    #         numpy_pose = np.array([
    #             position.x, position.y, position.z,
    #             orientation.x, orientation.y, orientation.z, orientation.w
    #         ])
    #         self.placement_pose_array_list.append(numpy_pose)

    # def placement_rotation_list_callback(self, placement_rotation):
    #     self.placement_rotation_list = []
    #     # This callback stores the received 'rotations' data into self.fragment_rotations_list
    #     self.placement_rotation_list = list(placement_rotation.data)

    # def placement_side_list_callback(self, placement_side):
    #     self.placement_side_list = []
    #     # This callback stores the received 'rotations' data into self.fragment_rotations_list
    #     self.placement_side_list = list(placement_side.data)   

    #########
    # USE WIDE HAND
    #########
    # def use_wide_hand_callback(self, use_wide_hand):
    #     self.use_wide_hand_grasping_list = []
    #     # This callback stores the received 'rotations' data into self.fragment_rotations_list
    #     self.use_wide_hand_grasping_list = list(use_wide_hand.data)   


    def setup_hands(self, open_hands=True):
        if open_hands:
            self.hand_api_right.open_hand()
            self.hand_api_left.open_hand()
            print('Initialized Hands!')

        self.tf_hand_left = get_transform(parent_frame="left_hand_v1_wide_grasp_link", child_frame="arm_1_angle_flange")
        self.tf_hand_right = get_transform(parent_frame="right_hand_v1_2_research_grasp_link",
                                           child_frame="arm_2_angle_flange")

        self.left_hand_arm_transform = pytr.transform_from_pq([self.tf_hand_left.transform.translation.x,
                                                               self.tf_hand_left.transform.translation.y,
                                                               self.tf_hand_left.transform.translation.z,
                                                               self.tf_hand_left.transform.rotation.w,
                                                               self.tf_hand_left.transform.rotation.x,
                                                               self.tf_hand_left.transform.rotation.y,
                                                               self.tf_hand_left.transform.rotation.z
                                                               ])

        self.right_hand_arm_transform = pytr.transform_from_pq([self.tf_hand_right.transform.translation.x,
                                                                self.tf_hand_right.transform.translation.y,
                                                                self.tf_hand_right.transform.translation.z,
                                                                self.tf_hand_right.transform.rotation.w,
                                                                self.tf_hand_right.transform.rotation.x,
                                                                self.tf_hand_right.transform.rotation.y,
                                                                self.tf_hand_right.transform.rotation.z
                                                                ])

        self.left_hand_arm_transform_np = get_pose_from_transform(initial_pose_left)
        publish_tf_np(self.left_hand_arm_transform_np, child_frame='left_initial_pose')
        self.left_arm_initial_pose = get_pose_stamped_from_arr(self.left_hand_arm_transform_np)

        self.right_hand_arm_transform_np = get_pose_from_transform(initial_pose_right)
        publish_tf_np(self.right_hand_arm_transform_np, child_frame='right_initial_pose')
        self.right_arm_initial_pose = get_pose_stamped_from_arr(self.right_hand_arm_transform_np)

        # get hand tf
        self.hand_tf = get_hand_tf()

    def get_final_placement(self):
        """ Get the final position and orientation for placing from  the publisher """
        while len(self.placement_pose_array_list) < 1 or len(self.placement_rotation_list) < 1 or \
            len(self.placement_side_list) < 1 or len(self.use_wide_hand_grasping_list) < 1:
            # print('side')
            # print(self.placement_side_list)
            # print('use_wide')
            # print(self.use_wide_hand_grasping_list)
            pass
    
        print("found the following final placements: ", self.placement_pose_array_list)
        print("found the following final placements rotations: ", self.placement_rotation_list)
        print("found the following final placements sides: ", self.placement_side_list)
        print("found the following grasping use wide hands bools: ", self.use_wide_hand_grasping_list)
        return self.placement_pose_array_list[0], self.placement_rotation_list[0], self.placement_side_list[0], self.use_wide_hand_grasping_list[0]

    def get_fragment_position(self):
        while len(self.fragment_pose_list) < 1 or len(self.fragment_ids_list) < 1 or len(self.fragment_rotations_list) < 1:
            pass
        print("found the following ids: ", self.fragment_ids_list)
        print("found the following rotations: ", self.fragment_rotations_list)

        fresco_center = copy.deepcopy(self.fragment_pose_list[0][:3])
        num_frescos = len(self.fragment_pose_list)
        return fresco_center, self.fragment_rotations_list[0], num_frescos, o3d.geometry.PointCloud(), self.fragment_ids_list[0]

    def set_active_arm(self):
        print("YOYOYO")
        self.arm = ARM_ENUM.BOTH
        self.hand_api = self.hand_api_left
        self.used_hand = "left"
        
        arm_target_pose_np = np.zeros(7)
        arm_target_pose_np[:3] = [0.2, 0.2, 1.2]
        arm_target_pose_np[6] = 1
        arm_target_pose_np2 = np.zeros(7)
        arm_target_pose_np2[:3] = [0.2, -0.2, 1.2]
        arm_target_pose_np2[6] = 1
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')

        print("TARGET POSE", arm_target_pose_np)
        print("TARGET POSE 2", arm_target_pose_np2)
        #input('Go To first pose')

        self.move_arm(self.arm, arm_target_pose_np, None)
        
        exit()
        
        # if self.use_wide_hand:
        #     self.arm = ARM_ENUM.ARM_1
        #     self.hand_api = self.hand_api_left
        #     self.used_hand = "left"
        #     print("=== Using Wide Hand")
        # else:
        #     self.arm = ARM_ENUM.ARM_2
        #     self.hand_api = self.hand_api_right
        #     self.used_hand = "right"
        #     print("=== Using QB Hand")
            
        # print("self.hand_api left after: ",self.hand_api)
        #self.mu.move_out_of_path(self.arm)


    def get_fresco_world_pose(self, fresco_position, z_offset=None):
        if z_offset is not None: 
            fresco_position[2] = z_offset
        #print("fresco position: ", fresco_position)
        #print("hand tf: ", self.hand_tf)
        initial_fresco_pose = np.concatenate((fresco_position, self.hand_tf))

        initial_fresco_pose_ros = get_pose_from_arr(initial_fresco_pose)
        #print("fresco position: ", initial_fresco_pose_ros)

        ### Transform the pose of fragment from the camera frame to the base frame (world)
        fresco_pose_world = transform_pose_vislab(initial_fresco_pose_ros, "camera_depth_optical_frame", "world")
        fresco_pose_world_np = get_arr_from_pose(fresco_pose_world)
        return fresco_pose_world, fresco_pose_world_np


    def run_demo(self, fresco_center, fresco_rotation, final_placements_position, final_rotations, placement_side, use_wide):
        fresco_release = 0
        # Select which hand should be used
        #obj_size = self.get_object_size(object_cloud)
        USE_WIDE_HAND_THRESHOLD = 0.13
        self.use_wide_hand = use_wide # True if obj_size.extent[1] > USE_WIDE_HAND_THRESHOLD else False
        print("USE WIDE HAND: ",self.use_wide_hand)
        self.set_active_arm()


        # Get fragment allignment if needed
        # if self.use_fragment_alignment:
        #     hand_tf_rotated = self.get_fresco_allignment(obj_size)

        # Get initial pose of fragment
        #object_center = np.array([0, 0, 0.64495862])
        fresco_pose_world_orig, fresco_pose_world_np_orig = self.get_fresco_world_pose(fresco_center.copy())
        self.fresco_world_z = fresco_pose_world_np_orig[2]
        fresco_pose_world, fresco_pose_world_np = self.get_fresco_world_pose(fresco_center.copy(), z_offset=1.21)

        print("fresco center world: ",fresco_center.copy())
        print("fresco pose world np: ", fresco_pose_world_np)
        print('Orig: ', fresco_pose_world_np_orig)

        #print("fresco_pose_world: ",fresco_pose_world)
        # if self.use_wide_hand:
        #     fresco_pose_world_np_orig[1] -= 0.1348
        #     fresco_pose_world_np[1] -= 0.1348
        # else:
        #     fresco_pose_world_np_orig[1] += 0.1348
        #     fresco_pose_world_np[1] += 0.1348

        #input('Go To first pose')

       
        #ToDo change 
        hand_pose_world_np = self.add_move_position(self.arm, fresco_pose_world_np.copy(),
                                                    [-0.0, -0.0, 0],
                                                    [0.0, 0.0, 0])
        hand_pose_world_np[2] = 1.29

        if self.use_fragment_alignment:
            hand_pose_world_np[3:] = hand_tf_rotated
        else:
            hand_pose_world_np[3:] = self.hand_tf
        publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

        hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)

        T0 = pytr.transform_from_pq(hand_pose_world_np)
        T1_left = pytr.concat(self.left_hand_arm_transform, T0)
        T1_right = pytr.concat(self.right_hand_arm_transform, T0)

        if self.use_wide_hand:
            arm_target_pose_np = get_pose_from_transform(T1_left)
        else:
            arm_target_pose_np = get_pose_from_transform(T1_right)

        q_orig = arm_target_pose_np[3:].copy()

        
            
        # grasp_yaw = np.clip(grasp_yaw, np.deg2rad(-90), np.deg2rad(90))

        

        # if grasp_yaw < :
        #     print("correcting the angle so we use only positive")
        #     grasp_yaw += np.deg2rad(180)
        if self.grasp_without_rotation == True:
            grasp_yaw = 0
        else:
            #######################
            # BETTER SOLUTION WOULD BE:
            # use the position of the chest to limit the angle
            #
            # position of the chest we can find in
            # /xbotcore/joint_states
            # fancy is to limit the rotation based on the position of the chest
            # we should use 
            # - link_position (first value)
            # link position is between -0.8 and 0.8
            #######################
            grasp_yaw = angle_utils.normalize(fresco_rotation + np.deg2rad(180)+ np.deg2rad(45), -180, 180)
            print(f"Angle: {np.rad2deg(grasp_yaw)}")
            #######################
            # HOTFIX
            # # the robot seems to have trouble to reach the grasping position
            # if the angle is larger than 90 degrees (in any direction)
            #######################
            if grasp_yaw > np.deg2rad(90):
                print("\n" * 3)
                print(f"correcting the angle! it was {np.rad2deg(grasp_yaw)}")
                grasp_yaw = np.deg2rad(-180) + grasp_yaw
                print(f"now it is {np.rad2deg(grasp_yaw)}")
                print("\n" * 3)
                input('sure?')
            elif grasp_yaw < np.deg2rad(-90):
                print("\n" * 3)
                print(f"correcting the angle! it was {np.rad2deg(grasp_yaw)}")
                grasp_yaw = np.deg2rad(180) + grasp_yaw
                print(f"now it is {np.rad2deg(grasp_yaw)}")
                print("\n" * 3)
                input('sure?')


        q_rot = quaternion_from_euler(np.deg2rad(180), np.deg2rad(0), grasp_yaw)
        q_new = quaternion_multiply(q_rot, q_orig)
        arm_target_pose_np[3:] = q_new

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')

        print("TARGET POSE", arm_target_pose_np)
        #input('Go To first pose')

        self.move_arm(self.arm, arm_target_pose_np)

        ### 2. Tilt hand
        ### RPY to convert: 90deg (1.57), Pi/12, -90 (-1.57)
        arm_target_pose_np = self.change_hand_angle(arm_target_pose_np)

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        self.move_arm(self.arm, arm_target_pose_np)

        # wait for user input DEBUG
        #input("Press Enter to continue...")

        ### 3. Go down to grasp (return to parallel, go down, then rotate again)
        fresco_down_pose = arm_target_pose_np.copy()
        fresco_down_pose[1] = fresco_down_pose[1] - 0.04 
        fresco_down_pose[2] = self.fresco_world_z
        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    fresco_down_pose[:3],
                                                    fresco_down_pose[:3])
        #print("go down", arm_target_pose_np)
        #input('Go Down')
        self.move_arm(self.arm, arm_target_pose_np)
        
        print("Grasping loop now")


        # 4. Grasp Object
        arm_target_pose_np, grasp_success = self.grasping_loop(arm_target_pose_np)

        if not grasp_success:
            self.go_home_pose()
            return
        else:
            print("Successfully grasped!")
        # wait for user input DEBUG
        #input("Press Enter to continue...")


        ### 5. Go To Placing Area
        # final_placements_position
        # final_rotations
        print('-' * 50)
        print("FINAL PLACEMENT")
        print(final_placements_position)
        placement_center_table_x = -0.1
        placement_center_table_y = 0.5
        # table_reference = [1.3, 1.6]
        # x_placement = final_placements_position[0] - table_reference[0]
        # y_placement = final_placements_position[1] - table_reference[1]
        # breakpoint()
        
        if self.follow_the_hand == True:
            print('following the hand')
            if self.use_wide_hand == True:
                placement_side = 1
            else:
                placement_side = -1
        else:
            print("not following the hand, following the published data for placement side")

        if self.hardcoded_placement == True:
            x_placement = 0.20 + 0.10 * fresco_release
            y_placement = placement_side * 0.50
        else:
            x_placement = placement_center_table_x + final_placements_position[0]
            y_placement = placement_side * placement_center_table_y + final_placements_position[1]
            print(f"going above the fragment at {x_placement}, {y_placement}")

        z = arm_target_pose_np[2]
        # When we use the wide hand, we need to turn it 90 degrees
        if self.use_wide_hand == True and self.follow_the_hand == False:
            # rotate
            current_quat = arm_target_pose_np[3:]
            current_euler = euler_from_quaternion(current_quat)
            new_euler = current_euler + np.array([0, 0, np.deg2rad(-90)])
            new_quat = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2] ) 
            arm_target_pose_np[3:] = new_quat

        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    [x_placement, y_placement, z],
                                                    [x_placement, y_placement, z])
                                                    # [0.20 + x_placement, 0.50 + y_placement, z],
                                                    # [0.20 + x_placement, 0.50 + y_placement, z])
                                                    # OLD CODE HARD CODED
                                                    # [0.20 + 0.10 * fresco_release, placement_side * 0.50, z],
                                                    # [0.20 + 0.10 * fresco_release, placement_side * 0.50, z])
        print("last move?")
        self.move_arm(self.arm, arm_target_pose_np)

        # 6. Go down
        z_wide_hand = 1.15
        z_small_hand = 1.1
        if self.hardcoded_placement == True:
            down_x_placement_wide_hand = 0.20 # why not anymore the 0.1 * fresco_release?
            down_y_placement_wide_hand = placement_side * 0.50 - 0.10 * fresco_release
            down_x_placement_small_hand = 0.20
            down_y_placement_small_hand = placement_side * 0.50 + 0.10 * fresco_release
        else:
            # same as above, i changes only the Z value!
            down_x_placement_wide_hand = final_placements_position[0]
            down_y_placement_wide_hand = placement_side * placement_center_table_y + final_placements_position[1]
            down_x_placement_small_hand = final_placements_position[0]
            down_y_placement_small_hand = placement_side * placement_center_table_y + final_placements_position[1]
            if use_wide == True:
                print(f"going down (wide hand) at {down_x_placement_wide_hand}, {down_y_placement_wide_hand}")
            else:
                print(f"going down (small hand) at {down_x_placement_small_hand}, {down_y_placement_small_hand}")
        
        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    [down_x_placement_wide_hand, down_y_placement_wide_hand, z_wide_hand],
                                                    [down_x_placement_small_hand, down_y_placement_small_hand, z_small_hand])
                                                    # [0.20 + x_placement, -1 * (-0.50 + y_placement), 1.15],
                                                    # [0.20 + x_placement, -0.50 + y_placement, 1.1])
                                                    # OLD CODE HARD CODED
                                                    # [0.20, placement_side * 0.50 - 0.10 * fresco_release, 1.15],
                                                    # [0.20, placement_side * 0.50 + 0.10 * fresco_release, 1.1])
        self.move_arm(self.arm, arm_target_pose_np)
        print('-' * 50)


        ### 7. Open hand
        if self.use_hands:
            self.hand_api.open_hand()
            print('Opened!')
        
        if(self.use_gazebo):
            if(self.used_hand=="left"):
                detach_links(model_1="repair", link_1="left_hand_v1_wide_palm_central_little_link", model_2="RPf_00205", link_2="RPf_00204_link")
            elif(self.used_hand=="right"):
                detach_links(model_1="repair", link_1="right_hand_v1_2_research_palm_link", model_2="RPf_00205", link_2="RPf_00204_link")
            else:
                print("Validate names of used hands")

        # if self.use_wide_hand == True and self.follow_the_hand == False:
        #     # rotate
        #     current_quat = arm_target_pose_np[3:]
        #     current_euler = euler_from_quaternion(current_quat)
        #     new_euler = current_euler + [0, 0, -np.deg2rad(90)]
        #     new_quat = quaternion_from_euler(new_euler) 
        #     arm_target_pose_np[3:] = new_quat

        ### 8. Go up
        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    [0.20, 0.5, z],
                                                    [0.20, -0.5, z])
        ### Go Back To Home Position
        self.move_arm(self.arm, arm_target_pose_np)
        self.go_home_pose()


    def change_hand_angle(self, arm_target_pose, y_ang=0, r_ang=0, p_ang=0.26):
        q_rot = quaternion_from_euler(r_ang, p_ang, y_ang)
        q_orig = arm_target_pose[3:].copy()
        q_new = quaternion_multiply(q_rot, q_orig)
        arm_target_pose[3:] = q_new
        return arm_target_pose


    def set_move_position(self, arm, arm_target_pose_np, position_l, position_r):
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = position_l
            arm_target_pose_np[2] = np.clip(arm_target_pose_np[2], a_min=self.min_z_value_arm_1, a_max=2.0)
        else:
            arm_target_pose_np[:3] = position_r
            arm_target_pose_np[2] = np.clip(arm_target_pose_np[2], a_min=self.min_z_value_arm_2, a_max=2.0)

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        return arm_target_pose_np


    def add_move_position(self, arm, target_pose_np, position_l, position_r):
        if arm == ARM_ENUM.ARM_1:
            target_pose_np[:3] += position_l
            target_pose_np[2] = np.clip(target_pose_np[2], a_min=self.min_z_value_arm_1, a_max=2.0)
        else:
            target_pose_np[:3] += position_r
            target_pose_np[2] = np.clip(target_pose_np[2], a_min=self.min_z_value_arm_2, a_max=2.0)
        publish_tf_np(target_pose_np, child_frame='arm_grasp_pose')
        return target_pose_np

    def go_home_pose(self):
        dummy_pose = get_pose_stamped_from_arr(np.zeros(7))
        publish_tf_np(np.zeros(7), child_frame='arm_grasp_pose')
        if not self.mu.move_home_klampt(dummy_pose):
            print("test 1")
            if not self.mu.move_to_home():
                print("test dead")
                exit()

    def move_arm(self, arm, pose_np, pose_np2=None):
        print("Planning trajectory")
        arm_target_pose = get_pose_stamped_from_arr(pose_np)
        if(pose_np2 is not None): pose_np2 = get_pose_stamped_from_arr(pose_np2)
        if self.use_klampt:
            if not self.mu.move_arm_to_pose_klampt(arm, arm_target_pose, pose_np2):
                print("Klmapt failed, resetting manipulation utils")
                # Idk if this is nessesary
                # self.reset_manipulation_utils()
                # if not self.mu.move_arm_to_pose_klampt(arm, arm_target_pose):
                #     print("Klmapt failed, will try moveit")
                #     if not self.mu.move_arm_to_pose_moveit(arm, arm_target_pose):
                #         exit()
                exit()
        else:
            if not self.mu.move_arm_to_pose_moveit(arm, arm_target_pose):
                exit()


    def grasping_loop(self, arm_target_pose_np):
        if(self.use_gazebo):
            rospy.sleep(3.0)
            print(f"Attaching to {self.used_hand}")
            if(self.used_hand=="left"):
                result = attach_links(model_1="repair", link_1="left_hand_v1_wide_palm_central_little_link", model_2="RPf_00205", link_2="RPf_00204_link")
            elif(self.used_hand=="right"):
                result = attach_links(model_1="repair", link_1="right_hand_v1_2_research_palm_link", model_2="RPf_00205", link_2="RPf_00204_link")
            else:
                print("Validate names of used hands")

            if(result==True):self.gazebo_attached = True
            
        ### Attempt Grasping
        self.hand_api.close_hand_2(self.used_hand)
        print('Closing!')

        arm_target_pose_np[2] += 0.16

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')

        self.move_arm(self.arm, arm_target_pose_np)

        print("self.hand_api before error: ", self.hand_api)
        qbhand_curr = self.hand_api.get_current()
        print(qbhand_curr)
        print('curre', qbhand_curr.m1_curr)
        print('curre2', qbhand_curr.m2_curr)
        print('Is the fresco present?')
        grasp_count = 1
        orig_arm_target_pose_np = arm_target_pose_np.copy()
        if(self.use_gazebo == False):
            while (not (int(qbhand_curr.m1_curr) > 100 and int(qbhand_curr.m2_curr) > 100)) and grasp_count<400:

                self.hand_api.open_hand()
                rospy.sleep(1)

                if grasp_count > 1:
                    yaw_angle = np.random.uniform(-np.deg2rad(20), np.deg2rad(20))
                    arm_target_pose_np = self.change_hand_angle(orig_arm_target_pose_np,  y_ang=yaw_angle, p_ang=0)

                ### Go down
                arm_target_pose_np[2] -= 0.10 + 0.06
                publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
                self.move_arm(self.arm, arm_target_pose_np)

                
                self.hand_api.close_hand_2(self.used_hand)      
                    
                rospy.sleep(1)

                ### Lift up
                arm_target_pose_np[2] += 0.10 + 0.06
                publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
                self.move_arm(self.arm, arm_target_pose_np)

                qbhand_curr = self.hand_api.get_current()
                print("qbhand_curr: ",int(qbhand_curr.m1_curr))
                
                if(self.gazebo_attached == True): break
                grasp_count += 1
        
        success = False

        if (int(qbhand_curr.m1_curr) > 100 and int(qbhand_curr.m2_curr) > 100):
            success = True
        elif(self.gazebo_attached == True):
            success = True
        else: print("No success :(")

        print('Fresco is Grasped')
        self.hand_api.close_hand()
        print('Closing Hand Tight!')

        ### 5. Lift up
        #arm_target_pose_np[2] += 0.133
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        self.move_arm(self.arm, arm_target_pose_np)
        return arm_target_pose_np, success


    def get_fresco_allignment(self, obj_bbox):
        # Get fragment bounding box pose, transform to world frame & publish
        bbox_pose = get_pose_from_arr(
            np.concatenate((obj_bbox.get_center(), R.from_matrix(obj_bbox.R.tolist()).as_quat())))
        bbox_pose_world = transform_pose_vislab(bbox_pose, "camera_depth_optical_frame", "world")
        publish_tf_np(get_arr_from_pose(bbox_pose_world), child_frame='obj_box_rot')

        # Get fragment bounding box rotation
        bbox_rot = R.from_quat(get_arr_from_pose(bbox_pose_world)[3:]).as_euler('xyz')

        # ==> Code for hand re-orientation
        fragment_2nd_principal_axis_angle = bbox_rot[2]

        # === Manel: If 90deg hand mount is availble this block can PROBABLY be deleted
        # ===>
        # Convert angle to -90:90 range if necessary
        # if fragment_2nd_principal_axis_angle > np.pi / 2:
        #     fragment_2nd_principal_axis_angle -= np.pi
        # elif fragment_2nd_principal_axis_angle < -np.pi / 2:
        #     fragment_2nd_principal_axis_angle += np.pi

        # # Apply rules using feasibility limits tested in gazebo with moveit (and without torso yaw and linear guide)
        # principal_axis_delta = np.pi/6 # np.pi / 4
        # if fragment_2nd_principal_axis_angle - principal_axis_delta > np.deg2rad(-75):
        #     grasp_yaw = angle_utils.normalize(fragment_2nd_principal_axis_angle + principal_axis_delta + np.pi, -180, 180)
        # # 2nd preference: point fingers away from the torso. For that, the hand should rotate outward relative to principal ais
        # elif fragment_2nd_principal_axis_angle + principal_axis_delta < np.deg2rad(50):
        #     grasp_yaw = fragment_2nd_principal_axis_angle - principal_axis_delta
        # <===

        # If 90deg mount for the hand is available
        principal_axis_delta = np.pi / 4
        grasp_yaw = angle_utils.normalize(fragment_2nd_principal_axis_angle + principal_axis_delta, -180, 180)

        hand_tf_euler = R.from_quat(self.hand_tf).as_euler('xyz')
        hand_tf_euler[2] = grasp_yaw
        return R.from_euler('xyz', hand_tf_euler).as_quat()


    def get_object_size(self, object_cloud):
        print('Getting object with max number of points')
        object_cloud = get_max_cluster(object_cloud, True)
        return object_cloud.get_oriented_bounding_box()


if __name__ == '__main__':
    node_name = "moveit_test"
    rospy.init_node(node_name)
    demo = PicpkNPlaceDemo(True)

    #demo.hand_api_right.open_hand()
    #exit()
    # wait for user input
    input("Press enter to start The Experiment")
    while True:
        demo.reset_manipulation_utils()
        demo.setup_hands(open_hands=False)
        
        #demo = PicpkNPlaceDemo(True)
        demo.go_home_pose()
        
        #del demo
        #demo = PicpkNPlaceDemo(True)
        demo.reset_manipulation_utils()
        demo.setup_hands()

        # Get number of frescos from object data
        rerun_detection = True
        while rerun_detection:
            print("Try to detect Frescos")
            fresco_center, fresco_rotation, num_frescos, object_cloud, fragment_id = demo.get_fragment_position()
            print("FRESCO CENTERMAN: ",fresco_center)
            final_placements_position, final_rotations, placement_side, use_wide = demo.get_final_placement()
            print(f"placement: {final_placements_position}" )
            print(f'Number of frescos detected: {num_frescos}')
            #inp = input('detection good? Y/n:').strip() or "y"
            inp = "y"
            rerun_detection = False if inp=="y" else True

        if num_frescos > 0:
            #input("Press Enter to Start Next Grasp...")
            print("-" * 40)
            print(f"Yay! Let's go!\nWe now pick and place {fragment_id} (it is now in {fresco_center})")
            if use_wide == True:
                print(f"We take it with the wide hand")
            else:
                print(f"We take it with the small hand")
            if placement_side < 0:
                print("we place it to the right (based on the group)")
            else:
                print("we place it to the left (based on the group)")
            print("-" * 40)
            demo.run_demo(fresco_center, fresco_rotation, final_placements_position, final_rotations, placement_side, use_wide)
        print("Finished pick and place, will try next fresco")
        #del demo