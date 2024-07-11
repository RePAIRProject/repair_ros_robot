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

initial_pose_left = pytr.transform_from_pq([0.247, 0.410, 1.315, 0.056, 0.819, 0.338, 0.459])
initial_pose_right = pytr.transform_from_pq([0.245, -0.396, 1.309, -0.216, 0.771, -0.486, 0.348 ])

        

if __name__ == '__main__':
    node_name = "moveit_test"
    rospy.init_node(node_name)

    mu = ManipulationUtils()

    debug = False
    use_pyrealsense = False

    # Get and print parameters
    gazebo = bool(rospy.get_param("/"+node_name+"/gazebo"))

    hand = True
    if hand:
      # Create QbHand object for controlling the hand
      print('Connecting to qb Soft Hand')
      hand_api_right = QbHand('right', gazebo)
      hand_api_left = QbHand('left', gazebo)
      print('Connected!')

      # open hand
      hand_api_right.open_hand()
      hand_api_left.open_hand()
      print('Opened!')

    tf_hand_left = get_transform(parent_frame="left_hand_v1_wide_grasp_link", child_frame="arm_1_tcp")
    tf_hand_right = get_transform(parent_frame="right_hand_v1_2_research_grasp_link", child_frame="arm_2_tcp")
   
    left_hand_arm_transform = pytr.transform_from_pq([tf_hand_left.transform.translation.x,
                                                      tf_hand_left.transform.translation.y,
                                                      tf_hand_left.transform.translation.z,
                                                      tf_hand_left.transform.rotation.w,
                                                      tf_hand_left.transform.rotation.x,
                                                      tf_hand_left.transform.rotation.y,
                                                      tf_hand_left.transform.rotation.z
                                                      ])

    right_hand_arm_transform = pytr.transform_from_pq([tf_hand_right.transform.translation.x,
                                                       tf_hand_right.transform.translation.y,
                                                       tf_hand_right.transform.translation.z,
                                                       tf_hand_right.transform.rotation.w,
                                                       tf_hand_right.transform.rotation.x,
                                                       tf_hand_right.transform.rotation.y,
                                                       tf_hand_right.transform.rotation.z
                                                       ])

    # get hand orientation
    hand_tf = get_hand_tf()

    num_frescos, pcd, table_cloud, object_cloud = get_number_of_frescos(debug, use_pyrealsense)
    print (f'Number of frescos detected: {num_frescos}')
    
    # print ('Table Segmentation')
    # table_cloud, object_cloud = segment_table(pcd)

    # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

    # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)

    # if debug:
    #     object_cloud.paint_uniform_color([0, 1, 0])
    #     table_cloud.paint_uniform_color([1, 0, 0])
    #     o3d.visualization.draw_geometries([table_cloud, object_cloud])

    left_hand_arm_transform_np = get_pose_from_transform(initial_pose_left)
    publish_tf_np(left_hand_arm_transform_np, child_frame='left_initial_pose')
    left_arm_initial_pose = get_pose_stamped_from_arr(left_hand_arm_transform_np)
    
    right_hand_arm_transform_np = get_pose_from_transform(initial_pose_right)
    publish_tf_np(right_hand_arm_transform_np, child_frame='right_initial_pose')
    right_arm_initial_pose = get_pose_stamped_from_arr(right_hand_arm_transform_np)
    exit()

    fresco_release = 0
    moveit_test = MoveItTest()
    while num_frescos > 0:

        # Move arms to initial pose
        moveit_test.move_to_pose(ARM_ENUM.ARM_1, left_arm_initial_pose)
        moveit_test.move_to_pose(ARM_ENUM.ARM_2, right_arm_initial_pose)

        # table_cloud, object_cloud = segment_table(pcd)
        # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)
        # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)
        print ('Getting object with max number of points')
        object_cloud = get_max_cluster(object_cloud, True)
        obj_bbox = object_cloud.get_oriented_bounding_box()

        use_fragment_alignment = False
        if use_fragment_alignment:
            # Get fragment bounding box pose, transform to world frame & publish
            bbox_pose = get_pose_from_arr(np.concatenate((obj_bbox.get_center(), R.from_matrix(obj_bbox.R.tolist()).as_quat())))
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
            principal_axis_delta = np.pi/4
            grasp_yaw = angle_utils.normalize(fragment_2nd_principal_axis_angle + principal_axis_delta, -180, 180)
        
            hand_tf_euler = R.from_quat(hand_tf).as_euler('xyz')
            hand_tf_euler[2] = grasp_yaw
            hand_tf_rotated = R.from_euler('xyz', hand_tf_euler).as_quat()
 
        initial_pose = np.concatenate((object_cloud.get_center(), hand_tf))
        initial_pose = get_pose_from_arr(initial_pose)

        ### Transform the pose from the camera frame to the base frame (world)
        hand_pose_world = transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
        hand_pose_world_np = get_arr_from_pose(hand_pose_world)
        hand_pose_world_np[0] += 0.04
        hand_pose_world_np[1] += 0.03
        hand_pose_world_np[2] = 1.15 + 0.15
        if use_fragment_alignment:
            hand_pose_world_np[3:] = hand_tf_rotated
            # hand_pose_world_np[3:] = R.from_euler('xyz', np.array([0.0, 1.57, np.deg2rad(50)])).as_quat()
        else:
            hand_pose_world_np[3:] = hand_tf
        publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

        hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
        
        T0 = pytr.transform_from_pq(hand_pose_world_np)
        T1_left = pytr.concat(left_hand_arm_transform, T0)
        T1_right = pytr.concat(right_hand_arm_transform, T0)

        USE_WIDE_HAND_THRESHOLD = 0.035
        use_wide_hand = True if obj_bbox.extent[1] > USE_WIDE_HAND_THRESHOLD else False
        print("=== Extent:", obj_bbox.extent[1])
        if use_wide_hand:
            arm_target_pose_np = get_pose_from_transform(T1_left)
            arm = ARM_ENUM.ARM_1
            hand_api = hand_api_left
            print("=== Using Wide Hand")
        else:
            arm_target_pose_np = get_pose_from_transform(T1_right)
            arm = ARM_ENUM.ARM_2
            hand_api = hand_api_right
            print("=== Using QB Hand")

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        ### 1. Go to position over the object
        moveit_test = MoveItTest()
        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        ### 2. Tilt hand
        ### RPY to convert: 90deg (1.57), Pi/12, -90 (-1.57)
        y_ang = -0.26
        q_rot = quaternion_from_euler(0, y_ang, 0)

        q_orig = arm_target_pose_np[3:].copy()
        q_new = quaternion_multiply(q_orig, q_rot)

        arm_target_pose_np[3:] = q_new
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        ### 3. Go down to grasp (return to parallel, go down, then rotate again)
        arm_target_pose_np[2] -= 0.170
        arm_target_pose_np[0] += 0.02
        #arm_target_pose_np[1] += 0.02
        arm_target_pose_np[3:] = q_new

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        if hand:
            ### 4. close hand
            hand_api.close_hand()
            print('Closed!')

        ### 5. Lift up
        arm_target_pose_np[2] += 0.173

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        ### 5. Move side
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = [-0.1 + 0.15* fresco_release, 0.610, 1.5]
        else:
            arm_target_pose_np[:3] = [-0.1 + 0.15* fresco_release, -0.610, 1.5]
        
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        # 6. Go down
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = [-0.130, -1*(-0.70 + 0.13* fresco_release), 1.17]
        else:
            arm_target_pose_np[:3] = [-0.130, -0.70 + 0.13* fresco_release, 1.17]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        if hand:
            ### 7. Open hand
            hand_api.open_hand()
            print('Opened!')

        ### 8. Go up
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = [-0.100, 0.609, 1.5]
        else:
            arm_target_pose_np[:3] = [-0.100, -0.609, 1.5]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        fresco_release += 1.
        num_frescos, object_cloud, table_cloud = check_frescos_left(debug, False)
        print (f'Objects left: {num_frescos}')

