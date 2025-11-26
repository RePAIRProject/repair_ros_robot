#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from collections import deque

# from sensor_msgs.msg import JointState
import math
from enum import Enum

from repair_interface.srv import *

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

from repair_interface.msg import RecognitionData, PlacementData, PlacedPieces
import argparse
import yaml
import os
from sensor_msgs.msg import JointState

initial_pose_left = pytr.transform_from_pq([0.18584, 0.47267, 1.345, -0.15708, 0.97996, 0.12039, 0.022494])
initial_pose_right = pytr.transform_from_pq([0.18584, -0.47267, 1.345, 0.158, 0.98476, -0.071265, 0.014615])

def load_config(path: str) -> dict:
    """Load a YAML config file and return it as a dict."""
    with open(path, 'r') as f:
        return yaml.safe_load(f)

class PicpkNPlaceDemo:
    def __init__(self, use_gazebo, debug=False):
        # right hand
        # - Translation: [0.232, -0.114, 1.076]
        #- Rotation: in Quaternion [0.130, 0.755, -0.226, 0.601]

        # left hand
        #- Translation: [0.083, 0.518, 1.112]
        #- Rotation: in Quaternion [-0.064, 0.740, 0.276, 0.609]
        self.mu = ManipulationUtils()

        self.debug = debug
        
        # define gazebo parameter
        self.use_gazebo = use_gazebo
        if self.use_gazebo:
            from attach_objects import attach_links, detach_links

        self.gazebo_attached = False

        
        # load config file
        this_dir = os.path.dirname(os.path.abspath(__file__))
        if self.use_gazebo:
            from attach_objects import attach_links, detach_links
            self.config = load_config(os.path.join(this_dir, "configs", "gazebo_pipeline_config.yaml"))
        else:
            self.config = load_config(os.path.join(this_dir, "configs", "real_pipeline_config.yaml"))

        # define general parameter
        self.use_both_hands = self.config["use_both_hands"]
        self.use_pyrealsense = self.config["use_pyrealsense"]
        self.use_hands = self.config["use_hands"]
        self.use_fragment_alignment = self.config["use_fragment_alignment"]
        self.hardcoded_placement = self.config["hardcoded_placement"]
        self.grasp_without_rotation = self.config["grasp_without_rotation"]
        self.place_without_rotation = self.config['place_without_rotation']
        self.follow_the_hand = self.config["follow_the_hand"] # it will choose placement side based on the hand and not on the published data
        self.use_klampt = self.config["use_klampt"]
        self.single_step_execution = self.config["single_step_execution"]

        
        # initialize lists
        self.fragment_pose_list = []
        self.fragment_ids_list = []
        self.fragment_rotations_list = []
        self.placement_pose_array_list = []
        self.placement_rotation_list = []
        self.placement_side_list = []
        self.use_wide_hand_grasping_list = []
        
        if self.grasp_without_rotation == True:
            print("#" * 50)
            print('WARNING:\nWe are ignoring the rotation, as we set `grasp_without_rotation` to True!')
            print("#" * 50)
        if self.follow_the_hand == True:
            print("#" * 50)
            print('WARNING:\nWe are choosing the placement side based on the hand and not on the published data!')
            print("Set `follow_the_hand` to False to remove this and follow group-wise placement.")
            print("#" * 50)
        
        self.min_z_value_arm_1 = self.config["min_z_value_arm_1"]
        self.min_z_value_arm_2 = self.config["min_z_value_arm_2"]
        
        if self.use_hands:
            self.hand_api_right = QbHand('right', self.use_gazebo)
            self.hand_api_left = QbHand('left', self.use_gazebo)
            #self.moveit = MoveItTest()
            # self.setup_hands()


        self.joint_history = deque(maxlen=10)
        self.current_joint_states = None
        self.is_robot_stable = True
        self.placed_pieces = []

        self.placed_pieces_pub = rospy.Publisher('/placed_pieces', PlacedPieces, queue_size=10)

        self.recognition_data_sub = rospy.Subscriber('/recognition/recognition_data', RecognitionData, self.recognition_data_callback)
        self.placement_data_sub = rospy.Subscriber('/recognition/placement_data', PlacementData, self.placement_data_callback)
        self.robot_states_sub = rospy.Subscriber('/joint_states', JointState, self.update_current_joint_config)
        # self.reset_sand()
        while self.current_joint_states is None:
            pass

        self.mu.set_single_step_execution(self.single_step_execution)
        # self.reset_sand(True)

        # test_joints = self.current_joint_states
        # test_joints[1] = -np.pi/8
        # success, path = self.mu.move_to_joint_pose(test_joints)
        # inv_path = self.inverse_path(path)
        # self.move_path(inv_path)
        # exit()

    def update_current_joint_config(self, joint_states: JointState, threshold: float = 1e-4):
        """
        Update robot joint configuration and check whether the joint values
        have changed beyond a given threshold over the last `window_size` steps.
        """
        # Convert to numpy array
        current = np.array(joint_states.position)

        # Append current joint values
        self.joint_history.append(current)

        # Store the newest state
        self.current_joint_states = current
        # Only check if we have at least two states
        if len(self.joint_history) > 1:
            oldest = self.joint_history[0]
            diff = np.linalg.norm(current - oldest)
            self.is_robot_stable = not (diff > threshold)
        else:
            self.is_robot_stable = True


    # def update_current_joint_config(self, joint_states:JointState):
    #     # get robot current joint configuration from robot_state publisher
    #     # set motion_planners real_robot configuration to received robot state
    #     self.current_joint_states = np.array(joint_states.position)
        
        
    def reset_manipulation_utils(self):
        del self.mu
        self.mu = ManipulationUtils()

    #### 
    # Joint Callback
    ####    

    # def recognition_data_callback(self, recognition_data, placement_data):
    #     # stores the received 'ids' data into self.fragment_ids_list
    #     self.fragment_ids_list = []
    #     self.fragment_ids_list = list(recognition_data.id_array.data)
        
    #     # stores the received 'rotations' data into self.fragment_rotations_list
    #     self.fragment_rotations_list = []
    #     if self.grasp_without_rotation == True:
    #         self.fragment_rotations_list = [0] * len(recognition_data.rotation_array.data)
    #     else:
    #         self.fragment_rotations_list = list(recognition_data.rotation_array.data)

    #     # stores the received 'poses' data into self.fragment_pose_list
    #     self.fragment_pose_list = []
    #     for pose in recognition_data.pose_array.poses:
    #         position = pose.position
    #         orientation = pose.orientation
    #         numpy_pose = np.array([
    #             position.x, position.y, position.z,
    #             orientation.x, orientation.y, orientation.z, orientation.w
    #         ])
    #         self.fragment_pose_list.append(numpy_pose)

    #     # stores the received 'rotations' data into self.fragment_rotations_list
    #     self.use_wide_hand_grasping_list = []
    #     self.use_wide_hand_grasping_list = list(recognition_data.use_wide_hand.data)   

    #     # stores the received 'poses' data into self.placement_pose_array_list
    #     self.placement_pose_array_list = []
    #     for pose in placement_data.placement_pose_array.poses:
    #         position = pose.position
    #         orientation = pose.orientation
    #         numpy_pose = np.array([
    #             position.x, position.y, position.z,
    #             orientation.x, orientation.y, orientation.z, orientation.w
    #         ])
    #         self.placement_pose_array_list.append(numpy_pose)

    #     # This callback stores the received 'rotations' data into self.fragment_rotations_list
    #     self.placement_rotation_list = []
    #     self.placement_rotation_list = list(placement_data.placement_rotation.data)

    #     # This callback stores the received 'side' data into self.placement_side_list
    #     self.placement_side_list = []
    #     self.placement_side_list = list(placement_data.placement_side.data) 



    #### 
    # Recognition
    ####

    def recognition_data_callback(self, recognition_data):
        # stores the received 'ids' data into self.fragment_ids_list
        self.fragment_ids_list = []
        self.fragment_ids_list = list(recognition_data.id_array.data)
        
        # stores the received 'rotations' data into self.fragment_rotations_list
        self.fragment_rotations_list = []
        if self.grasp_without_rotation == True:
            self.fragment_rotations_list = [0] * len(recognition_data.rotation_array.data)
        else:
            self.fragment_rotations_list = list(recognition_data.rotation_array.data)
        
        # stores the received 'area' data into self.fragment_areas_list
        self.fragment_areas_list = []
        self.fragment_areas_list = list(recognition_data.area_array.data)

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

    def transform_world_to_hand(self):
        self.world_tf_hand_left = get_transform(parent_frame="world", child_frame="left_hand_v1_wide_grasp_link")
        self.world_tf_hand_right = get_transform(parent_frame="world",
                                           child_frame="right_hand_v1_2_research_grasp_link")

        self.left_hand_world_transform = pytr.transform_from_pq([self.world_tf_hand_left.transform.translation.x,
                                                               self.world_tf_hand_left.transform.translation.y,
                                                               self.world_tf_hand_left.transform.translation.z,
                                                               self.world_tf_hand_left.transform.rotation.w,
                                                               self.world_tf_hand_left.transform.rotation.x,
                                                               self.world_tf_hand_left.transform.rotation.y,
                                                               self.world_tf_hand_left.transform.rotation.z
                                                               ])

        self.right_hand_world_transform = pytr.transform_from_pq([self.world_tf_hand_right.transform.translation.x,
                                                                self.world_tf_hand_right.transform.translation.y,
                                                                self.world_tf_hand_right.transform.translation.z,
                                                                self.world_tf_hand_right.transform.rotation.w,
                                                                self.world_tf_hand_right.transform.rotation.x,
                                                                self.world_tf_hand_right.transform.rotation.y,
                                                                self.world_tf_hand_right.transform.rotation.z
                                                                ])



    def setup_hands(self, open_hands=True):
        if open_hands:
            self.hand_api_right.open_hand()
            self.hand_api_left.open_hand()
            # print('Initialized Hands!')

        self.tf_hand_left = get_transform(parent_frame="left_hand_v1_wide_grasp_link", child_frame="arm_1_angle_flange")
        self.tf_hand_right = get_transform(parent_frame="right_hand_v1_2_research_grasp_link", child_frame="arm_2_angle_flange")
        

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
            pass
    
        # print("found the following final placements: ", self.placement_pose_array_list)
        # print("found the following final placements rotations: ", self.placement_rotation_list)
        # print("found the following final placements sides: ", self.placement_side_list)
        # print("found the following grasping use wide hands bools: ", self.use_wide_hand_grasping_list)
        return self.placement_pose_array_list[0], self.placement_rotation_list[0], self.placement_side_list[0], self.use_wide_hand_grasping_list[0]

    def get_fragment_position(self):
        while len(self.fragment_pose_list) < 1 or len(self.fragment_ids_list) < 1 or len(self.fragment_rotations_list) < 1:
            pass
        print("found the following ids: ", self.fragment_ids_list)
        # print("found the following rotations: ", self.fragment_rotations_list)
        # print("found the following areas: ", self.fragment_areas_list)


        gazebo_fragment_dict = {
            "104": ["RPf_00104", "RPf_00104_link"],
            "204": ["RPf_00204", "RPf_00204_link"],
            "205": ["RPf_00205", "RPf_00205_link"],
            }
        fresco_center = copy.deepcopy(self.fragment_pose_list[0][:3])
        num_frescos = len(self.fragment_pose_list)
        try:
            self.fragment_id_gazebo = gazebo_fragment_dict[str(self.fragment_ids_list[0])]
        except:
            self.fragment_id_gazebo = ["", ""]
        return fresco_center, self.fragment_rotations_list[0], self.fragment_areas_list[0], num_frescos, o3d.geometry.PointCloud(), self.fragment_ids_list[0]

    def set_active_arm(self):
        if self.use_both_hands == True:
            self.arm = ARM_ENUM.BOTH
            self.hand_api = self.hand_api_left
            self.used_hand = "both"
            print("=== Using Both Hands")
        else:
            if self.use_wide_hand:
                self.arm = ARM_ENUM.ARM_1
                self.hand_api = self.hand_api_left
                self.used_hand = "left"
                print("=== Using Wide Hand")
            else:
                self.arm = ARM_ENUM.ARM_2
                self.hand_api = self.hand_api_right
                self.used_hand = "right"
                print("=== Using QB Hand")
            
        #self.mu.move_out_of_path(self.arm)


    def get_fresco_world_pose(self, fresco_position, z_offset=None):
        # Benno: Why do we add the z_offset inside the camera_color_optical_frame and not after?
        # if z_offset is not None: 
        #     fresco_position[2] = z_offset
        initial_fresco_pose = np.concatenate((fresco_position, np.array([0,0,0,1])))

        initial_fresco_pose_ros = get_pose_from_arr(initial_fresco_pose)

        ### Transform the pose of fragment from the camera frame to the base frame (world)
        if(self.use_gazebo): # I assume we need to take the depth frame here aswell in gazebo, similar to recognition
            fresco_pose_world = transform_pose_vislab(initial_fresco_pose_ros, "camera_depth_optical_frame", "world")
        else:
            fresco_pose_world = transform_pose_vislab(initial_fresco_pose_ros, "camera_color_optical_frame", "world")
        fresco_pose_world_np = get_arr_from_pose(fresco_pose_world)
        return fresco_pose_world, fresco_pose_world_np

    def wait_for_robot(self):
     while not self.is_robot_stable:
            rospy.sleep(0.1)

    def run_demo(self, fresco_center, fresco_rotation, fresco_area, final_placements_position, fresco_placement_rotation, placement_side, use_wide, fragment_id):
        original_placement_side = placement_side
        fresco_release = 0

        # Select which hand should be used
        self.use_wide_hand = use_wide
        self.set_active_arm()

        # Get initial pose of fragment
        fresco_pose_world, fresco_pose_world_np = self.get_fresco_world_pose(fresco_center.copy())

        self.fresco_world_z = fresco_pose_world_np[2]
        # print("Fresco Pose First", fresco_pose_world_np)
        publish_tf_np(fresco_pose_world_np, child_frame='unchanged_fresco_world_pose')

        # Get initial hand pose 
        hand_pose_world_np = fresco_pose_world_np.copy()
        hand_pose_world_np[3:] = self.hand_tf
        publish_tf_np(hand_pose_world_np, child_frame='fresco_world_pose')
        hand_pose_world_np[2] = self.config["fresco_pose_world_z"]
        hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
        publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')
        # print("Hand Pose", hand_pose_world_np)
       
        ############################################################################################################
        ####################################### GRASPING LOOP BEGIN ################################################
        ############################################################################################################


        arm_target_pose_np = hand_pose_world_np.copy()
        q_orig = arm_target_pose_np[3:].copy()
        
        grasp_yaw = 0
        q_rot = quaternion_from_euler(np.deg2rad(180), np.deg2rad(0), grasp_yaw)
        q_new = quaternion_multiply(q_rot, q_orig)
        arm_target_pose_np[3:] = q_new
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        # print("TARGET POSE 1", arm_target_pose_np)

        ### MOVE TO FIRST POSE ABOVE OBJECT
        # center the hand with fresco position + offset
        # arm_target_pose_np[0] = fresco_pose_world_np[0]
        # arm_target_pose_np[1] = fresco_pose_world_np[1]
        # print("HAND SHOULD GO TO POSITION: ", fresco_pose_world_np[:2])
        self.move_arm(self.arm, arm_target_pose_np)
        self.wait_for_robot()

        rot_amount = 0
        activate_offset_flag = False
        if fresco_area > self.config["fresco_area_threshold_for_rotation"]:
            ### ROTATE HAND 
            ### Only apply rotation if small hand currently or if big hand with big fragment (205)
            # if self.grasp_without_rotation == False and (self.arm==ARM_ENUM.ARM_2 or (self.arm==ARM_ENUM.ARM_1 and fragment_id == 205)):
            if self.grasp_without_rotation == False and self.arm==ARM_ENUM.ARM_2:
                print("== Rotating hand to align with fresco")
                publish_tf_np(arm_target_pose_np, child_frame='BEFORE_arm_grasp_pose')
                arm_target_pose_np, rot_amount, activate_offset_flag, offset_direction = self.calculate_hand_rotation(self.arm, arm_target_pose_np.copy(), fresco_rotation)
                # arm_target_pose_np, rot_amount = self.calculate_hand_rotation_in_hand_frame(arm_target_pose_np.copy(), fresco_rotation)
                publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
                publish_tf_np(arm_target_pose_np, child_frame='AFTER_arm_grasp_pose')
                # print("TARGET POSE 2", arm_target_pose_np)

                ### ROTATE HAND TO ALIGN WITH FRESCO
                self.move_arm(self.arm, arm_target_pose_np)
                self.wait_for_robot()
        else:
            print("== Skipping hand rotation due to small area:", fresco_area)


        # Apply offset in x-y plane
        if self.arm==ARM_ENUM.ARM_2 and activate_offset_flag:
            arm_target_pose_np = self.apply_offset(arm_target_pose_np, offset_direction*self.config["fresco_pose_world_x_offset"], offset_direction*self.config["fresco_pose_world_y_offset"])
            # self.move_arm(self.arm, arm_target_pose_np)
            # self.wait_for_robot()

        ### TILT HAND
        arm_target_pose_np = self.change_hand_angle(arm_target_pose_np)
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        
        ### TILT HAND TO BETTER GRASP FRESCO 
        # self.move_arm(self.arm, arm_target_pose_np)
        q_orig = arm_target_pose_np[3:].copy()

        above_fresco_height = self.config["grasp_z_above_fresco_in_meters_wide"] if self.arm==ARM_ENUM.ARM_1 else self.config["grasp_z_above_fresco_in_meters_qb"]

        ### 3. GO DOWN TO GRASP
        arm_target_pose_np[2] = self.fresco_world_z + above_fresco_height
        self.move_arm(self.arm, arm_target_pose_np.copy())
        self.wait_for_robot()


        ### 4. GRASP OBJECT
        arm_target_pose_np, grasp_success = self.grasping_loop(arm_target_pose_np.copy())

        q_when_grasping = arm_target_pose_np[3:].copy()

        if not grasp_success:
            self.go_home_pose()
            return
        else:
            print("Successfully grasped!")
        # print('Closing Hand Tightly!')

        ############################################################################################################
        ####################################### GRASPING LOOP ENDED ################################################
        ############################################################################################################

        self.placed_pieces.append(fragment_id)
        place_id_msg = PlacedPieces()
        ids_array_msg = Int32MultiArray()
        ids_array_msg.data = self.placed_pieces

        place_id_msg.header.stamp = rospy.Time.now()
        place_id_msg.placed_pieces = ids_array_msg
        self.placed_pieces_pub.publish(place_id_msg)

        ### GO TO PLACING AREA
        print('-' * 50)
        # print("Saved Placement Position: ", final_placements_position)
        placement_center_table_x = self.config["placement_center_table_x"]
        placement_center_table_y = self.config["placement_center_table_y"]

        ### Decide Placement Side
        placement_side = self.choose_placement_side(placement_side)

        ### Calculate Placement Position
        before_placement_height = arm_target_pose_np[2]
        if self.hardcoded_placement == True:
            x_placement = 0.20 + (0.10 * fresco_release)
            y_placement = placement_side * 0.50
        else:
            x_placement = placement_center_table_x + final_placements_position[0]
            y_placement = placement_side * placement_center_table_y + final_placements_position[1]
            # print(f"Going Above the Fragment at {x_placement}, {y_placement}")

        ### Turn Robot Base according to hand and side
        needs_right_turn = (not self.use_wide_hand and original_placement_side == 1)
        needs_left_turn  = (self.use_wide_hand and original_placement_side == -1)

        if needs_right_turn or needs_left_turn:
            rotation_joints = np.copy(self.current_joint_states)
            rotation_joints[0] = 0
            self.mu.move_to_joint_pose(rotation_joints)
            self.wait_for_robot()

            rotation_joints = np.copy(self.current_joint_states)
            rotation_joints[1] = np.pi/2 if needs_right_turn else -np.pi/2
            print("TURN BASE RIGHT" if needs_right_turn else "TURN BASE LEFT")
            self.mu.move_to_joint_pose(rotation_joints)
            self.wait_for_robot()
        
        arm_target_pose_np = self.calculate_placement_rotation_new(arm_target_pose_np.copy(), fresco_placement_rotation, q_when_grasping)
        # arm_target_pose_np = self.calculate_placement_rotation(arm_target_pose_np.copy(), 0,  q_orig)

        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    [x_placement, y_placement, before_placement_height],
                                                    [x_placement, y_placement, before_placement_height])

        ### Move Robot To Place Position
        self.move_arm(self.arm, arm_target_pose_np)
        self.wait_for_robot()
       
        # 6. Go Down
        if self.hardcoded_placement == True:
            x_placement = 0.20
            y_placement = placement_side * 0.50 - 0.10 * fresco_release
            x_placement = 0.20
            y_placement = placement_side * 0.50 + 0.10 * fresco_release

        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    [x_placement, y_placement, self.config["dropping_position_z_arm_1"]],
                                                    [x_placement, y_placement, self.config["dropping_position_z_arm_2"]])

        place_down_path = self.move_arm(self.arm, arm_target_pose_np)
        self.wait_for_robot()

        print('-' * 50)


        ### 7. Open Hand
        if self.use_hands:
            self.hand_api.open_hand()
            # print('Opened!')
        
        if(self.use_gazebo):
            if(self.used_hand=="left"):
                detach_links(model_1="repair", link_1="left_hand_v1_wide_palm_central_little_link", model_2=self.fragment_id_gazebo[0], link_2=self.fragment_id_gazebo[1])
            elif(self.used_hand=="right"):
                detach_links(model_1="repair", link_1="right_hand_v1_2_research_palm_link", model_2=self.fragment_id_gazebo[0], link_2=self.fragment_id_gazebo[1])
            else:
                print("Validate names of used hands")

        ### 8. Go Up
        if place_down_path is not None:
            place_up_path = self.inverse_path(place_down_path)
            self.move_path(place_up_path)
        else:
            arm_target_pose_np[2] += 0.15
            place_up_path = self.move_arm(self.arm, arm_target_pose_np)

        arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                    [0.20, 0.5, before_placement_height],
                                                    [0.20, -0.5, before_placement_height])
        self.wait_for_robot()

        if self.use_wide_hand:
            rotation_joints = np.copy(self.current_joint_states)
            rotation_joints[0] = 0
            self.mu.move_to_joint_pose(rotation_joints)
            self.wait_for_robot()

        ### Go Back To Home Position
        self.go_home_pose()


    def choose_placement_side(self, placement_side):
        if self.follow_the_hand == True:
            # print('USE SIDE ACCORDING TO CHOSEN HAND')
            placement_side = 1 if self.use_wide_hand == True else -1 
        # else:
            # print("USE PLACEMENT SIDE OF PUBLISHED DATA")

        # print("Placement Side: ", placement_side)
        print("Placement on Robots ","left" if(placement_side == -1) else "right", " side")
        # print("Use Wide Hand: ", self.use_wide_hand)

        return placement_side


    def calculate_placement_rotation_new(self, arm_target_pose_np, fresco_placement_rotation, q_when_grasping):
        # TODO possibly need to consider here how the fresco lays in hand already from grasp
        if self.place_without_rotation == False:
            # this is the amount thefresco should rotate from its initial position to the placement position
            amount_to_rotate = np.deg2rad(-fresco_placement_rotation)
            amount_to_rotate_quat = quaternion_from_euler(0, 0, amount_to_rotate)
            print("Amount to Rotate (deg): ", np.rad2deg(amount_to_rotate))

            # 'q_when_grasping' is the snapshot of the pose of the hand in the moment it grasped the fresco
            # the final orientation is that snapshot rotated by the amount needed to place the fresco correctly
            rotated1_hand_tf = quaternion_multiply(amount_to_rotate_quat, q_when_grasping)
            arm_target_pose_np[3:] = rotated1_hand_tf
        elif self.use_wide_hand == True and self.follow_the_hand == False:
            # place without rotation, but for wide hand we rotate -90 deg to compensate for torso rotation
            current_quat = arm_target_pose_np[3:]
            current_euler = euler_from_quaternion(current_quat)
            new_euler = current_euler + np.array([0, 0, np.deg2rad(-90)])
            new_quat = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2] ) 
            arm_target_pose_np[3:] = new_quat

        return arm_target_pose_np

    def calculate_placement_rotation(self, arm_target_pose_np, grasp_rotation, initial_rotation):
        # TODO possibly need to consider here how the fresco lays in hand already from grasp

        if(self.place_without_rotation == True):
            grasp_yaw = 0
            best_rotated_hand_tf = None
        else:
            # --- Apply the rotation from the second and third axes to hand_tf ---
            # rotated_hand_tfs = []
            hand_rot = final_placements_position[2]
            if hand_rot is not None:
                q_rot1 = quaternion_from_euler(0, 0, hand_rot - grasp_rotation)
                q_curr = arm_target_pose_np[3:].copy()
                # rotated1_hand_tf = quaternion_multiply(q_rot1, self.hand_tf)
                rotated1_hand_tf = quaternion_multiply(q_rot1, q_curr)
                # Remove the 45deg angle offset
                

            else:
                raise ValueError("hand_rot not specified")

            if rotated1_hand_tf is not None:
                arm_target_pose_np[3:] = rotated1_hand_tf
            else:
                q_rot = quaternion_from_euler(np.deg2rad(180), np.deg2rad(0), grasp_yaw)
                q_new = quaternion_multiply(q_rot, initial_rotation)
                arm_target_pose_np[3:] = q_new


        # When we use the wide hand, we need to turn it 90 degrees
        if self.use_wide_hand == True and self.follow_the_hand == False:
            # rotate
            current_quat = arm_target_pose_np[3:]
            current_euler = euler_from_quaternion(current_quat)
            new_euler = current_euler + np.array([0, 0, np.deg2rad(-90)])
            new_quat = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2] ) 
            arm_target_pose_np[3:] = new_quat
        return arm_target_pose_np


    def calculate_hand_rotation(self, arm_enum, arm_target_pose_np, fresco_rotation):
        
        pos_orig = arm_target_pose_np[:3].copy()
        q_orig = arm_target_pose_np[3:].copy()
        target_hand_rot_1, target_hand_rot_2 = self.calc_hand_rotation(arm_enum, fresco_rotation)

        euler_before_rot= euler_from_quaternion(q_orig)
        yaw_before_rot = euler_before_rot[0]
        print("Original angle:", np.rad2deg(yaw_before_rot))

        rotated_hand_tfs = []
        if target_hand_rot_1 is not None:
            to_rotate_z1 = -1.57 - target_hand_rot_1
            # to_rotate_z1 = - target_hand_rot_1
            q_rot1 = quaternion_from_euler(0, 0, to_rotate_z1)
            rotated1_hand_tf = quaternion_multiply(q_rot1, self.hand_tf)
            rotated_hand_tfs.append(rotated1_hand_tf)
            rot_amount = to_rotate_z1
        if target_hand_rot_2 is not None:
            to_rotate_z2 = -1.57 - target_hand_rot_2
            # to_rotate_z2 = - target_hand_rot_2
            q_rot2 = quaternion_from_euler(0, 0, to_rotate_z2)
            rotated2_hand_tf = quaternion_multiply(q_rot2, self.hand_tf)
            rotated_hand_tfs.append(rotated2_hand_tf)
            rot_amount = to_rotate_z2
            
        # Compare z axes and select the best (against hand_tf)
        orig_rot = R.from_quat(self.hand_tf)
        orig_z = orig_rot.apply([0, 0, 1])
        best_idx = 0
        best_dot = -np.inf
        for idx, rotated_hand_tf in enumerate(rotated_hand_tfs):
            rot = R.from_quat(rotated_hand_tf)
            z_axis = rot.apply([0, 0, 1])
            dot = np.dot(z_axis, orig_z)
            if dot > best_dot:
                best_dot = dot
                best_idx = idx

        best_rotated_hand_tf = rotated_hand_tfs[best_idx]

        if best_rotated_hand_tf is not None:
            #q_new = quaternion_multiply(best_rotated_hand_tf, q_orig)
            arm_target_pose_np[3:] = best_rotated_hand_tf
        else:
            q_rot = quaternion_from_euler(np.deg2rad(180), np.deg2rad(0), 0)
            q_new = quaternion_multiply(q_rot, q_orig)
            arm_target_pose_np[3:] = q_new
        arm_target_pose_np[:3] = pos_orig

        # Check if we need to apply an offset to the position based on amount we rotate
        quat_after_rot = arm_target_pose_np[3:]
        euler_after_rot= euler_from_quaternion(quat_after_rot)
        yaw_after_rot = euler_after_rot[0]
        print("Rotated angle:", np.rad2deg(yaw_after_rot))
        
        # Rushed fix: we add an offset when rotating in negative direction, if rotating the other way we do not rotate.
        activate_offset_flag = True if np.abs(yaw_after_rot-yaw_before_rot) > np.deg2rad(35) else False
        offset_direction = np.sign(yaw_after_rot - yaw_before_rot)
        if offset_direction == 1:
            offset_direction = 0
        print("Activate Offset Flag:", activate_offset_flag)

        # return arm_target_pose_np.copy(), rot_amount
        return arm_target_pose_np.copy(), euler_from_quaternion(best_rotated_hand_tf)[2], activate_offset_flag, offset_direction


    def calc_hand_rotation(self, arm_enum, angle):
        # Second axis: 45 deg right
        phi = np.deg2rad(45)
        if arm_enum == ARM_ENUM.ARM_1:
            rotated_angle = angle - phi
        else:
            rotated_angle = angle + phi

        # Third axis: opposite to second axis
        phi2 = np.deg2rad(180)
        rotated_angle_2 = rotated_angle + phi2

        return rotated_angle, rotated_angle_2   

    def apply_offset(self, arm_target_pose, x_offset=0, y_offset=0):
            orientation = arm_target_pose[3:].copy()

            arm_pose = get_pose_from_arr(arm_target_pose)
            if self.arm == ARM_ENUM.ARM_1:
                hand_pose = transform_pose_vislab(arm_pose, "world", "left_hand_v1_wide_grasp_link")
            elif self.arm == ARM_ENUM.ARM_2:
                hand_pose = transform_pose_vislab(arm_pose, "world", "right_hand_v1_2_research_grasp_link")

            
            hand_pose_array = get_arr_from_pose(hand_pose)
            
            hand_pose_array[2] += x_offset  # for some reason axes are swaped
            hand_pose_array[1] += y_offset

            hand_pose = get_pose_from_arr(hand_pose_array)
            if self.arm == ARM_ENUM.ARM_1:
                arm_target_pose = transform_pose_vislab(hand_pose, "left_hand_v1_wide_grasp_link", "world")
            elif self.arm == ARM_ENUM.ARM_2:
                arm_target_pose = transform_pose_vislab(hand_pose, "right_hand_v1_2_research_grasp_link", "world")
            out = get_arr_from_pose(arm_target_pose)
            out[3:] = orientation

            # publish_tf_np(out, child_frame='hand_pose_array')

            return out

    def change_hand_angle(self, arm_target_pose, y_ang=0, r_ang=0, p_ang=0.26):
            positions = arm_target_pose[:3].copy()

            arm_pose = get_pose_from_arr(arm_target_pose)
            if self.arm == ARM_ENUM.ARM_1:
                hand_pose = transform_pose_vislab(arm_pose, "world", "left_hand_v1_wide_grasp_link")
            elif self.arm == ARM_ENUM.ARM_2:
                hand_pose = transform_pose_vislab(arm_pose, "world", "right_hand_v1_2_research_grasp_link")

            
            hand_pose_array = get_arr_from_pose(hand_pose)

            q_rot = quaternion_from_euler(r_ang, p_ang, y_ang)
            # q_orig = arm_target_pose[3:].copy()
            q_orig = hand_pose_array[3:].copy()
            q_new = quaternion_multiply(q_rot, q_orig)
            # arm_target_pose[3:] = q_new
            
            hand_pose_array[3:] = q_new
            hand_pose = get_pose_from_arr(hand_pose_array)
            if self.arm == ARM_ENUM.ARM_1:
                arm_target_pose = transform_pose_vislab(hand_pose, "left_hand_v1_wide_grasp_link", "world")
            elif self.arm == ARM_ENUM.ARM_2:
                arm_target_pose = transform_pose_vislab(hand_pose, "right_hand_v1_2_research_grasp_link", "world")
            out = get_arr_from_pose(arm_target_pose)
            out[:3] = positions

            publish_tf_np(out, child_frame='hand_pose_array')

            return out
    
    # def change_hand_angle(self, arm_target_pose, y_ang=0, r_ang=0, p_ang=0.26):
    #     q_rot = quaternion_from_euler(r_ang, p_ang, y_ang)
    #     q_orig = arm_target_pose[3:].copy()
    #     q_new = quaternion_multiply(q_rot, q_orig)
    #     arm_target_pose[3:] = q_new
    #     return arm_target_pose


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
    
    def reset_sand(self, reset=False):
        pose = np.zeros(7)
        if not reset:
            pose[0] = 10

        dummy_pose = get_pose_stamped_from_arr(pose)
        if not self.mu.move_sand(dummy_pose=dummy_pose):
            exit()

    def go_home_pose(self):
        dummy_pose = get_pose_stamped_from_arr(np.zeros(7))
        if not self.mu.move_home_klampt(dummy_pose):
            if not self.mu.move_to_home():
                exit()

    def move_arm(self, arm, pose_np):
        print("Planning trajectory")
        arm_target_pose = get_pose_stamped_from_arr(pose_np)
        if self.use_klampt:
            success = self.mu.reset_active_dof_klampt(arm=arm)
            success, path = self.mu.move_arm_to_pose_klampt(arm, arm_target_pose)
            if not success:
                print("Klampt failed, resetting manipulation utils and setting new active DOFs without translation")
                # Idk if this is nessesary
                self.reset_manipulation_utils()
                success = self.mu.remove_translation_active_dof_klampt()
                success, path = self.mu.move_arm_to_pose_klampt(arm, arm_target_pose)
                if not success:
                    print("Klampt failed, will try moveit")
                    input("Execute Moveit")
                    if not self.mu.move_arm_to_pose_moveit(arm, arm_target_pose):
                        exit()
        else:
            if not self.mu.move_arm_to_pose_moveit(arm, arm_target_pose):
                exit()
            path = None
        return path
    
    def move_path(self, path):
        success, path = self.mu.move_on_path(path)
        return path
        

    def inverse_path(self, path):
        return path[::-1]    


    def grasping_loop(self, hand_target_pose_np):
        ### Attempt Grasping
        if self.use_gazebo:
            # gazebo wide hand disintegrates in simulation here
            rospy.sleep(3.0)
            print(f"Attaching to {self.used_hand}")
            if(self.used_hand=="left"):
                result = attach_links(model_1="repair", link_1="left_hand_v1_wide_palm_central_little_link", model_2=self.fragment_id_gazebo[0], link_2=self.fragment_id_gazebo[1])
            elif(self.used_hand=="right"):
                result = attach_links(model_1="repair", link_1="right_hand_v1_2_research_palm_link", model_2=self.fragment_id_gazebo[0], link_2=self.fragment_id_gazebo[1])
            else:
                print("Validate names of used hands")

            if(result==True):self.gazebo_attached = True
        else: 
            ### close hand
            self.hand_api.close_hand(self.used_hand, gazebo_flag=self.use_gazebo)
            # print('Closing Hand!')

        ### Store upper and lower hand position before and after grasping for later use in loop
        upper_hand_target_pose_np, lower_hand_target_pose_np = hand_target_pose_np.copy(), hand_target_pose_np.copy()
        upper_hand_target_pose_np[2] += self.config["lift_position_z_offset_after_grasp"]
        ### Lift Hand Up
        publish_tf_np(upper_hand_target_pose_np, child_frame='arm_grasp_pose')
        self.move_arm(self.arm, upper_hand_target_pose_np)
        success = False

        if not self.use_gazebo:
            qbhand_curr = self.hand_api.get_current()
            print('Current m1 %f and current m2 %f' % (qbhand_curr.m1_curr, qbhand_curr.m2_curr))
            grasp_count = 1
            if self.arm == ARM_ENUM.ARM_1:
                threshold = self.config["wideHand_current_thresh"]
                threshold1 = self.config["wideHand_current_thresh"]
            elif self.arm == ARM_ENUM.ARM_2:
                threshold = self.config["qbHand_current_thresh"]
                threshold1 = self.config["qbHand_current_thresh2"]
            
            while (not (int(qbhand_curr.m1_curr) > threshold and int(qbhand_curr.m2_curr) > threshold1)) and grasp_count<=self.config["max_grasp_attempts"]:
                           
                self.hand_api.open_hand()
                rospy.sleep(1)
                
                ### change angle for slight adjustment
                # if grasp_count > 1:
                #     yaw_angle = np.random.uniform(-np.deg2rad(20), np.deg2rad(20))
                #     arm_target_pose_np = self.change_hand_angle(lower_hand_target_pose_np,  y_ang=yaw_angle, p_ang=0)

                ### Go down
                publish_tf_np(lower_hand_target_pose_np, child_frame='arm_grasp_pose')
                self.move_arm(self.arm, lower_hand_target_pose_np)

                ### close hand
                self.hand_api.close_hand(self.used_hand)      
                rospy.sleep(1)

                ### Lift up
                publish_tf_np(upper_hand_target_pose_np, child_frame='arm_grasp_pose')
                self.move_arm(self.arm, upper_hand_target_pose_np)

                ### check current
                qbhand_curr = self.hand_api.get_current()
                print('Current m1 %f and current m2 %f' % (qbhand_curr.m1_curr, qbhand_curr.m2_curr))
                grasp_count += 1
            
            if grasp_count > self.config["max_grasp_attempts"]:
                print("Failed to grasp fresco after 3 attempts")
                success = False
                return upper_hand_target_pose_np, success
            
            success = True
    
        elif self.gazebo_attached:
            success = True
        else: 
            print("No success :(")
            return upper_hand_target_pose_np, False

        # print('Fresco is Grasped')
        self.hand_api.close_hand(self.used_hand, is_tight=True)

        return upper_hand_target_pose_np, success


    def get_fresco_allignment(self, obj_bbox):
        # Get fragment bounding box pose, transform to world frame & publish
        bbox_pose = get_pose_from_arr(
            np.concatenate((obj_bbox.get_center(), R.from_matrix(obj_bbox.R.tolist()).as_quat())))
        bbox_pose_world = transform_pose_vislab(bbox_pose, "camera_color_optical_frame", "world")
        publish_tf_np(get_arr_from_pose(bbox_pose_world), child_frame='obj_box_rot')

        # Get fragment bounding box rotation
        bbox_rot = R.from_quat(get_arr_from_pose(bbox_pose_world)[3:]).as_euler('xyz')

        # ==> Code for hand re-orientation
        fragment_2nd_principal_axis_angle = bbox_rot[2]

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
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_gazebo', action='store_true')
    args = parser.parse_args()
    
    node_name = "moveit_test"
    rospy.init_node(node_name)
    demo = PicpkNPlaceDemo(use_gazebo=args.use_gazebo, debug=True)

    # wait for user input
    input("Press enter to start The Experiment")
    while True:
        demo.reset_manipulation_utils()
        demo.setup_hands()

        # Get number of frescos from object data
        rerun_detection = True
        while rerun_detection:
            print("Try to detect Frescos")
            fresco_center, fresco_rotation, fresco_area, num_frescos, object_cloud, fragment_id = demo.get_fragment_position()
            final_placements_position, final_rotations, placement_side, use_wide = demo.get_final_placement()
            rerun_detection = False

        if num_frescos > 0:
            #input("Press Enter to Start Next Grasp...")
            print("-" * 40)
            print(f"Yay! Let's go!\nWe now pick and place {fragment_id} (it is now in {fresco_center})")
            
            # if use_wide == True:
            #     print(f"We take it with the wide hand")
            # else:
            #     print(f"We take it with the small hand")
            # if placement_side < 0:
            #     print("we place it to the right (based on the group)")
            # else:
            #     print("we place it to the left (based on the group)")
            print("-" * 40)
            demo.run_demo(fresco_center, fresco_rotation, fresco_area, final_placements_position, final_rotations, placement_side, use_wide, fragment_id)
        print("Finished pick and place, will try next fresco")
