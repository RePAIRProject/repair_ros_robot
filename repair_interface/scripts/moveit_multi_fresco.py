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
        

if __name__ == '__main__':
    node_name = "moveit_test"
    rospy.init_node(node_name)

    mu = ManipulationUtils()

    debug = False
    use_pyrealsense = False

    # Get and print parameters
    side = "right" #str(rospy.get_param("/"+node_name+"/side"))
    gazebo = False #bool(rospy.get_param("/"+node_name+"/gazebo"))

    if side == "right":
        arm_no = 2
    elif side == "left":
        arm_no = 1
    else:
        print("Error:Side value has to be left or right")
        raise ValueError
    
    hand = True
    if hand:
      # Create QbHand object for controlling the hand
      print('Connecting to qb Soft Hand')
      hand_api = QbHand(side, gazebo)
      print('Connected!')

      # open hand
      hand_api.open_hand()
      print('Opened!')

    tf_hand = get_transform(parent_frame=side+"_hand_v1_2_research_grasp_link", child_frame="arm_"+str(arm_no)+"_tcp")
    # print (tf)

    hand_arm_transform = pytr.transform_from_pq([tf_hand.transform.translation.x,
                                                 tf_hand.transform.translation.y,
                                                 tf_hand.transform.translation.z,
                                                 tf_hand.transform.rotation.w,
                                                 tf_hand.transform.rotation.x,
                                                 tf_hand.transform.rotation.y,
                                                 tf_hand.transform.rotation.z
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

    fresco_release = 0
    while num_frescos > 0:

        # table_cloud, object_cloud = segment_table(pcd)
        # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)
        # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)
        print ('Getting object with max number of points')
        object_cloud = get_max_cluster(object_cloud, True)

        initial_pose = np.concatenate((object_cloud.get_center(), hand_tf))
        initial_pose = get_pose_from_arr(initial_pose)

        ### Transform the pose from the camera frame to the base frame (world)
        hand_pose_world = transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
        hand_pose_world_np = get_arr_from_pose(hand_pose_world)
        hand_pose_world_np[0] += 0.04
        hand_pose_world_np[1] += 0.03
        hand_pose_world_np[2] = 1.15 + 0.15
        hand_pose_world_np[3:] = hand_tf
        publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

        hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
        T0 = pytr.transform_from_pq(hand_pose_world_np)
        T1 = pytr.concat(hand_arm_transform, T0)

        arm_target_pose_np = get_pose_from_transform(T1)

        # arm_target_pose_np = get_arr_from_pose(arm_target_pose)
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        ### 1. Go to position over the object
        moveit_test = MoveItTest()
        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        ### 2. Tilt hand
        ### RPY to convert: 90deg (1.57), Pi/12, -90 (-1.57)
        y_ang = 0.20
        q_rot = quaternion_from_euler(0, y_ang, 0)

        q_orig = arm_target_pose_np[3:]
        q_new = quaternion_multiply(q_rot, q_orig)

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
        arm_target_pose_np[:3] = [-0.1 + 0.15* fresco_release, -0.60, 1.5]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        # 6. Go down
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
        arm_target_pose_np[:3] = [-0.100, -0.609, 1.5]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        # moveit_test.go_to_pos(arm_target_pose)
        mu.move_arm_to_pose_dawnik(ARM_ENUM.ARM_2, arm_target_pose)

        fresco_release += 1.
        num_frescos, object_cloud, table_cloud = check_frescos_left(debug, False)
        print (f'Objects left: {num_frescos}')

