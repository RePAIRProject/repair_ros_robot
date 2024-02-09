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
from vision_utils import prepare_scene
from align_utils import recognize_objects
from qbhand_test import QbHand
from moveit_test import MoveItTest


if __name__ == '__main__':

    node_name = "recognize_it_test"
    rospy.init_node(node_name)

    debug = True
    use_pyrealsense = False

    # Get and print parameters
    side = str(rospy.get_param("/"+node_name+"/side"))
    gazebo = bool(rospy.get_param("/"+node_name+"/gazebo"))

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

    tf_hand = get_transform(parent_frame=side+"_hand_v1s_grasp_link", child_frame="arm_"+str(arm_no)+"_tcp")
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

    print('Starting Point Cloud Processing')
    if use_pyrealsense:
        pcd = get_point_cloud_from_real_rs(debug)
    else:
        pcd = get_point_cloud_from_ros(debug)

    print('prepare scene')    
    object_cloud, table = prepare_scene(pcd, debug=debug)

    print('recognition')
    recognized_objects = recognize_objects(object_cloud)

    print("moving fragments")
    fresco_release = 0
    for obj_key in recognized_objects.keys():
        obj_bbox = recognized_objects[obj_key]['bbox']
        obj_pcd = recognized_objects[obj_key]['pcd']

        initial_pose = np.concatenate((obj_pcd.get_center(), hand_tf))
        initial_pose = get_pose_from_arr(initial_pose)
        
        ### Transform the pose from the camera frame to the base frame (world)
        hand_pose_world = transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
        
        hand_pose_world_np = get_arr_from_pose(hand_pose_world)
        hand_pose_world_np[0] += 0.04
        hand_pose_world_np[1] += 0.03
        hand_pose_world_np[2] = 1.15 + 0.15
        hand_pose_world_np[3:] = hand_tf
        publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')
        o3d.visualization.draw_geometries([obj_pcd])

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
        moveit_test.go_to_pos(arm_target_pose)
        
        ### 2. Tilt hand
        ### RPY to convert: 90deg (1.57), Pi/12, -90 (-1.57)
        y_ang = 0.26
        q_rot = quaternion_from_euler(0, y_ang, 0)

        q_orig = arm_target_pose_np[3:]
        q_new = quaternion_multiply(q_rot, q_orig)

        arm_target_pose_np[3:] = q_new
        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        moveit_test.go_to_pos(arm_target_pose)

        ### 3. Go down to grasp (return to parallel, go down, then rotate again)
        arm_target_pose_np[2] -= 0.168 
        arm_target_pose_np[3:] = q_new

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        moveit_test.go_to_pos(arm_target_pose)

        if hand:
            ### 4. close hand
            hand_api.close_hand()
            print('Closed!')

        ### 5. Lift up
        arm_target_pose_np[2] += 0.173

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        moveit_test.go_to_pos(arm_target_pose)

        ### 5. Move side
        arm_target_pose_np[:3] = [-0.087, -0.610, 1.47]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        moveit_test.go_to_pos(arm_target_pose)

        # 6. Go down
        arm_target_pose_np[:3] = [-0.110 + 0.1* fresco_release, -0.609, 1.257]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        moveit_test.go_to_pos(arm_target_pose)

        if hand:
            ### 7. Open hand
            hand_api.open_hand()
            print('Opened!')

        ### 8. Go up
        arm_target_pose_np[:3] = [-0.110, -0.609, 1.345]

        publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
        arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

        print ("Planning trajectory")
        moveit_test.go_to_pos(arm_target_pose)

        fresco_release += 1.
        print(f'Moved {fresco_release} fragments! Up with the next')
        # num_frescos, object_cloud, table_cloud = check_frescos_left(True, False)
        # print (f'Objects left: {num_frescos}')

    print("Finished")