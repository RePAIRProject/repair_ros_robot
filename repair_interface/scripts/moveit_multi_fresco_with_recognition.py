#!/usr/bin/env python3

from __future__ import print_function
import pdb
import rospy
# from sensor_msgs.msg import JointState

from repair_interface.srv import *

import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr

from tf.transformations import quaternion_from_euler, quaternion_multiply

from vision_utils import get_transform, get_hand_tf, publish_tf_np
from vision_utils import get_pose_from_arr, get_pose_stamped_from_arr
from vision_utils import get_arr_from_pose, get_point_cloud_from_ros, get_point_cloud_from_real_rs
from vision_utils import transform_pose_vislab, get_pose_from_transform
from vision_utils import get_number_of_frescos
from align_utils import recognize_objects
from qbhand_test import QbHand
from moveit_test import MoveItTest, ARM_ENUM
from scipy.spatial.transform import Rotation as R
import angle_utils

initial_pose_left = pytr.transform_from_pq([0.247, 0.410, 1.315, 0.056, 0.819, 0.338, 0.459])
initial_pose_right = pytr.transform_from_pq([0.245, -0.396, 1.309, -0.216, 0.771, -0.486, 0.348 ])


if __name__ == '__main__':

    # node_name = "recognize_it_test"
    node_name = "moveit_multi"
    rospy.init_node(node_name)

    debug = False
    use_pyrealsense = False

    # Get and print parameters
    gazebo = True #  bool(rospy.get_param("/"+node_name+"/gazebo"))
    
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

    print('Starting Point Cloud Processing')
    _, pcd, table, object_cloud = get_number_of_frescos(debug, use_pyrealsense)
    
    #if use_pyrealsense:
    #    pcd = get_point_cloud_from_real_rs(debug)
    #else:
    #    pcd = get_point_cloud_from_ros(debug)

    #print('prepare scene')
    #object_cloud, table = prepare_scene(pcd, debug=debug)

    print('recognition')
    recognized_objects = recognize_objects(object_cloud)

    print("moving fragments")
    fresco_release = 0

    moveit_test = MoveItTest()
    
    left_hand_arm_transform_np = get_pose_from_transform(initial_pose_left)
    publish_tf_np(left_hand_arm_transform_np, child_frame='left_initial_pose')
    left_arm_initial_pose = get_pose_stamped_from_arr(left_hand_arm_transform_np)
        
    right_hand_arm_transform_np = get_pose_from_transform(initial_pose_right)
    publish_tf_np(right_hand_arm_transform_np, child_frame='right_initial_pose')
    right_arm_initial_pose = get_pose_stamped_from_arr(right_hand_arm_transform_np)

    for obj_key in recognized_objects.keys():
        
        # Move arms to initial pose
        moveit_test.move_to_pose(ARM_ENUM.ARM_1, left_arm_initial_pose)
        moveit_test.move_to_pose(ARM_ENUM.ARM_2, right_arm_initial_pose)

        obj_bbox = recognized_objects[obj_key]['bbox']
        obj_pcd = recognized_objects[obj_key]['pcd']
        
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
 
        initial_pose = np.concatenate((obj_bbox.get_center(), hand_tf))
        initial_pose = get_pose_from_arr(initial_pose)
        
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
        
        USE_WIDE_HAND_THRESHOLD = 0.04
        use_wide_hand = True if obj_bbox.extent[1] > USE_WIDE_HAND_THRESHOLD else False
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
        
        ### 1. Go to position over the objec
        print ("Going above the detected fragment! Planning trajectory..")
        moveit_test.move_to_pose(arm, arm_target_pose)
        
        ### 2. Tilt hand
        y_ang = -0.26
        q_rot = quaternion_from_euler(0, y_ang, 0)

        q_orig = arm_target_pose_np[3:].copy()
        q_new = quaternion_multiply(q_orig, q_rot)

        arm_target_pose_np[3:] = q_new
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Tilting hand! Planning trajectory..")
        moveit_test.move_to_pose(arm, arm_target_pose)
        
        ### 3. Go down to grasp (return to parallel, go down, then rotate again)
        arm_target_pose_np[2] -= 0.17
        arm_target_pose_np[3:] = q_new

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Go down to grasp! Planning trajectory..")
        moveit_test.move_to_pose(arm, arm_target_pose)

        if hand:
            print("Closing hand..")
            ### 4. close hand
            hand_api.close_hand()
            print('Closed!')

        ### 5. Lift up
        arm_target_pose_np[2] += 0.173

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Lift it up! Planning trajectory..")
        moveit_test.move_to_pose(arm, arm_target_pose)

        ### 5. Move side
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = [-0.087, 0.610, 1.47]
        else:
            arm_target_pose_np[:3] = [-0.087, -0.610, 1.47]
        #arm_target_pose_np[:3] = arm_target_pose_np[:3] + 0.12*recognized_objects[obj_key]['sol_offset']

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Move it to the side! Planning trajectory..")
        # moveit_test.go_to_pos(arm_target_pose)
        moveit_test.move_to_pose(arm, arm_target_pose)

        # 6. Go down
        if arm == ARM_ENUM.ARM_1:
            target_down_position = np.asarray([-0.110, 0.61, 1.18])
        else:
            target_down_position = np.asarray([-0.110, -0.61, 1.18])
        # we have an offset based on the grasped fragment
        target_position_current_fragment = target_down_position + 0.12*recognized_objects[obj_key]['sol_offset']
        print(f"Fragment {obj_key} target position: {target_position_current_fragment}")
        arm_target_pose_np[:3] = target_position_current_fragment

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Placing it down! Planning trajectory..")
        # moveit_test.go_to_pos(arm_target_pose)
        moveit_test.move_to_pose(arm, arm_target_pose)

        if hand:
            print("Opening..")
            ### 7. Open hand
            hand_api.open_hand()
            print('Opened!')

        ### 8. Go up
        if arm == ARM_ENUM.ARM_1:
            arm_target_pose_np[:3] = [-0.110, 0.609, 1.345]
        else:
            arm_target_pose_np[:3] = [-0.110, -0.609, 1.345]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Go up again to start! Planning trajectory.. ")
        # moveit_test.go_to_pos(arm_target_pose)
        moveit_test.move_to_pose(arm, arm_target_pose)

        print(f'Placed the fragments')
            

    print("Finished")