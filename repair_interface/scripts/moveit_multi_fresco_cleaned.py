#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray

# from sensor_msgs.msg import JointState
import math
from enum import Enum

from repair_interface.srv import *

from typing import Union, List
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr

from tf.transformations import quaternion_from_euler, quaternion_multiply

from vision_utils import get_transform, get_hand_tf, publish_tf_np
from vision_utils import get_pose_from_arr, get_pose_stamped_from_arr
from vision_utils import get_arr_from_pose
from vision_utils import transform_pose_vislab, get_pose_from_transform
from vision_utils import segment_table, get_number_of_frescos, get_max_cluster, check_frescos_left

from qbhand_test import QbHand
from moveit_test import MoveItTest

from manipulation_utils import ManipulationUtils, ARM_ENUM

from scipy.spatial.transform import Rotation as R
import angle_utils
import copy

initial_pose_left = pytr.transform_from_pq([0.18584, 0.47267, 1.345, -0.15708, 0.97996, 0.12039, 0.022494])
initial_pose_right = pytr.transform_from_pq([0.18584, -0.47267, 1.345, 0.158, 0.98476, -0.071265, 0.014615])


class PicpkNPlaceDemo:
    def __init__(self, debug=False):
        self.mu = ManipulationUtils()
        self.debug = debug
        self.use_gazebo = False
        self.use_pyrealsense = False
        self.use_hands = True
        self.use_fragment_alignment = False
        self.fragment_pose_list = []

        if self.use_hands:
            self.hand_api_right = QbHand('right', False)
            self.hand_api_left = QbHand('left', False)
            self.moveit = MoveItTest()
            self.setup_hands()

        self.fragment_pose_sub = rospy.Subscriber('/detection/points', PoseArray, self.fragemnt_pose_callback)


    def fragemnt_pose_callback(self, pose_array):
        self.fragment_pose_list = []
        for pose in pose_array.poses:
            position = pose.position
            orientation = pose.orientation
            numpy_pose = np.array([
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.fragment_pose_list.append(numpy_pose)


    def setup_hands(self):
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

    def get_fragment_position(self):
        if len(self.fragment_pose_list) < 1:
            while len(self.fragment_pose_list) < 1:
                pass
            object_center = copy.deepcopy(self.fragment_pose_list[0][:3])
            num_frescos = len(self.fragment_pose_list)
        else:
            print(len(self.fragment_pose_list))
            num_frescos = len(self.fragment_pose_list)
            object_center = copy.deepcopy(self.fragment_pose_list[0][:3])
        object_center[2] = 1.21
        return  object_center, num_frescos, o3d.geometry.PointCloud()

    def set_active_arm(self):
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
        self.mu.move_out_of_path(self.arm)


    def run_demo(self):
        # Get number of frescos from object data
        object_center, num_frescos, object_cloud = self.get_fragment_position()
        #num_frescos, _, _, object_cloud = get_number_of_frescos(self.debug, self.use_pyrealsense)
        print(f'Number of frescos detected: {num_frescos}')

        fresco_release = 0
        while num_frescos > 0:
            # Select which hand should be used
            #obj_size = self.get_object_size(object_cloud)
            USE_WIDE_HAND_THRESHOLD = 0.13
            self.use_wide_hand = False # True if obj_size.extent[1] > USE_WIDE_HAND_THRESHOLD else False
            self.set_active_arm()

            # Get fragment allignment if needed
            if self.use_fragment_alignment:
                hand_tf_rotated = self.get_fresco_allignment(obj_size)

            # Get initial pose of fragment
            #object_center = object_cloud.get_center()
            #object_center = [0, 0 , 1.21]
            initial_pose = np.concatenate((object_center, self.hand_tf))
            initial_pose = get_pose_from_arr(initial_pose)

            ### Transform the pose from the camera frame to the base frame (world)
            hand_pose_world = transform_pose_vislab(initial_pose, "camera_color_optical_frame", "world")
            hand_pose_world_np = get_arr_from_pose(hand_pose_world)
            print(hand_pose_world_np)
            hand_pose_world_np = self.add_move_position(self.arm, hand_pose_world_np.copy(),
                                                       [0.05, -0.15, 0],
                                                       [0.07, 0.13, 0])
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
            q_rot = quaternion_from_euler(np.deg2rad(180), np.deg2rad(0), np.deg2rad(0))
            q_new = quaternion_multiply(q_rot, q_orig)
            arm_target_pose_np[3:] = q_new

            publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
            self.move_arm_moveit(self.arm, arm_target_pose_np)

            ### 2. Tilt hand
            ### RPY to convert: 90deg (1.57), Pi/12, -90 (-1.57)
            y_ang = 0.26
            q_rot = quaternion_from_euler(0, y_ang, 0)
            q_orig = arm_target_pose_np[3:].copy()
            q_new = quaternion_multiply(q_rot, q_orig)
            arm_target_pose_np[3:] = q_new


            publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
            self.move_arm_moveit(self.arm, arm_target_pose_np)


            # wait for user input DEBUG
            #input("Press Enter to continue...")


            ### 3. Go down to grasp (return to parallel, go down, then rotate again)
            arm_target_pose_np = self.add_move_position(self.arm, arm_target_pose_np.copy(),
                                                        [0, 0.0, -0.250 + 0.06],
                                                        [0, 0.0, -0.260 + 0.06])
            self.move_arm_moveit(self.arm, arm_target_pose_np)


            # 4. Grasp Object
            self.grasping_loop(arm_target_pose_np)


            # wait for user input DEBUG
            #input("Press Enter to continue...")


            ### 5. Go To Placing Area
            arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                        [0.20 + 0.10 * fresco_release, 0.50, 1.5+ 0.06],
                                                        [0.20 + 0.10 * fresco_release, -0.50, 1.5+ 0.06])
            self.move_arm_moveit(self.arm, arm_target_pose_np)


            # 6. Go down
            arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                        [0.20, -1 * (-0.50 + 0.10 * fresco_release), 1.15],
                                                        [0.20, -0.50 + 0.10 * fresco_release, 1.08])
            self.move_arm_moveit(self.arm, arm_target_pose_np)


            ### 7. Open hand
            if self.use_hands:
                self.hand_api.open_hand()
                print('Opened!')


            ### 8. Go up
            arm_target_pose_np = self.set_move_position(self.arm, arm_target_pose_np.copy(),
                                                        [0.20, 0.5, 1.5+ 0.06],
                                                        [0.20, -0.5, 1.5+ 0.06])
            ### Go Back To Home Position
            self.move_arm_moveit(self.arm, arm_target_pose_np)
            self.mu.move_to_home()


    def set_move_position(self, arm, arm_target_pose_np, position_l, position_r):
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = position_l
        else:
            arm_target_pose_np[:3] = position_r
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        return arm_target_pose_np

    def add_move_position(self, arm, target_pose_np, position_l, position_r):
        if arm == ARM_ENUM.ARM_1:
            target_pose_np[:3] += position_l
        else:
            target_pose_np[:3] += position_r
        publish_tf_np(target_pose_np, child_frame='arm_grasp_pose')
        return target_pose_np

    def move_arm_moveit(self, arm, pose_np):
        print("Planning trajectory")
        arm_target_pose = get_pose_stamped_from_arr(pose_np)
        if not self.mu.move_arm_to_pose_moveit(arm, arm_target_pose):
            exit()

    def grasping_loop(self, arm_target_pose_np):
        ### Attempt Grasping
        self.hand_api.close_hand_2(self.used_hand)
        print('Closing!')

        arm_target_pose_np[2] += 0.10 + 0.06

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')

        self.move_arm_moveit(self.arm, arm_target_pose_np)


        qbhand_curr = self.hand_api.get_current()
        print('curre', qbhand_curr.m1_curr)
        print('curre2', qbhand_curr.m2_curr)
        print('Is the fresco present?')
        while (not (int(qbhand_curr.m1_curr) > 100 and int(qbhand_curr.m2_curr) > 100)):

            self.hand_api.open_hand()
            rospy.sleep(1)

            ### Go down
            arm_target_pose_np[2] -= 0.10 + 0.06
            publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
            self.move_arm_moveit(self.arm, arm_target_pose_np)

            self.hand_api.close_hand_2(self.used_hand)
            rospy.sleep(1)

            ### Lift up
            arm_target_pose_np[2] += 0.10 + 0.06
            publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
            self.move_arm_moveit(self.arm, arm_target_pose_np)

            qbhand_curr = self.hand_api.get_current()

        print('Fresco is Grasped')
        self.hand_api.close_hand()
        print('Closing Hand Tight!')

        ### 5. Lift up
        arm_target_pose_np[2] += 0.073 + 0.06
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        self.move_arm_moveit(self.arm, arm_target_pose_np)

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

    # wait for user input
    while True:
        input("Press Enter to Start Next Grasp...")
        demo.run_demo()
