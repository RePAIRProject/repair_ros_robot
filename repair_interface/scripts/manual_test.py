#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
#from sensor_msgs.msg import JointState
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
import math
from enum import Enum

from repair_interface.srv import *

import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from typing import Union, List
from scipy.spatial.transform import Rotation as R
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot

from qbhand_test import QbHand

from sensor_msgs.msg import PointCloud2, PointField
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

def get_transform(parent_frame='base_link', child_frame='camera_depth_frame'):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        return transform

def get_hand_tf():
    quatR = R.from_quat([0, 0, 0, 1])
    quat_to_mat = quatR.as_matrix()

    transf1 = R.from_euler('y', -90, degrees=True)
    # transf2 = R.from_euler('z', -90, degrees=True)
    matF = transf1.apply(quat_to_mat)
    # matF = transf2.apply(matF)
    matF_to_quat = R.from_matrix(matF).as_quat()

    return matF_to_quat

def publish_tf_np(pose, par_frame="world", child_frame="goal_frame"):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = par_frame
    static_transformStamped.child_frame_id = child_frame

    static_transformStamped.transform.translation.x = float(pose[0])
    static_transformStamped.transform.translation.y = float(pose[1])
    static_transformStamped.transform.translation.z = float(pose[2])

    static_transformStamped.transform.rotation.x = float(pose[3])
    static_transformStamped.transform.rotation.y = float(pose[4])
    static_transformStamped.transform.rotation.z = float(pose[5])
    static_transformStamped.transform.rotation.w = float(pose[6])

    broadcaster.sendTransform(static_transformStamped)
    rospy.sleep(1)

    return True

def get_pose_from_arr(arr):
    pose = Pose()
    pose.position.x = arr[0]
    pose.position.y = arr[1]
    pose.position.z = arr[2]
    pose.orientation.x = arr[3]
    pose.orientation.y = arr[4]
    pose.orientation.z = arr[5]
    pose.orientation.w = arr[6]
    
    return pose

def get_pose_stamped_from_arr(arr):
    p = PoseStamped()
    p.header.frame_id = "world"
    p.header.stamp = rospy.Time(0)
    p.pose.position.x = arr[0]
    p.pose.position.y = arr[1]
    p.pose.position.z = arr[2]
    p.pose.orientation.x = arr[3]
    p.pose.orientation.y = arr[4]
    p.pose.orientation.z = arr[5]
    p.pose.orientation.w = arr[6]
    return p

def get_arr_from_pose(pose):
    arr = np.array([pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w])

    return arr

def get_point_cloud_from_ros(debug=False):
    point_cloud = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
    pc = []
    for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
        # print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
        pc.append([p[0], p[1], p[2]])

    # Segmentation of Point Cloud
    xyz = np.asarray(pc)
    #idx = np.where(xyz[:, 2] < 0.8)     # Prune point cloud to 0.8 meters from camera in z direction
    #xyz = xyz[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    print ('Received point cloud')

    if debug:
        o3d.visualization.draw_geometries([pcd])

    return pcd

def segment_table(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.005,
                                             ransac_n=5,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model

    # Partial Point Cloud
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    return inlier_cloud, outlier_cloud

def transform_pose_vislab(input_pose, from_frame, to_frame):
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    # pose_stamped.header.stamp = rospy.Time.now()
    rospy.sleep(1)
    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def get_pose_from_transform(T):
    quat = pyrot.quaternion_xyzw_from_wxyz(pyrot.quaternion_from_matrix(T[:3, :3]))
    pos = T[:3, 3]
    return np.concatenate((pos, quat))

if __name__ == '__main__':
    rospy.init_node('manual_test')

    # Create QbHand object for controlling the hand
    print('Connecting to qb Soft Hand')
    hand_api = QbHand()
    print('Connected!')

    tf_left = get_transform(parent_frame='left_hand_v1s_grasp_link', child_frame='arm_1_tcp')
    # tf_right = get_transform(parent_frame='arm_2_tcp', child_frame='right_hand_v1s_grasp_link')
    # print (tf)

    # get hand orientation
    hand_tf = get_hand_tf()

    #hand_pose_world_np = get_arr_from_pose(hand_pose_world)
    hand_pose_world_np = [0.0, 0.0, 1.0, 0, 0, 0]
    hand_pose_world_np[2] += 0.15
    hand_pose_world_np[3:] = hand_tf
    publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

    hand_arm_transform = pytr.transform_from_pq([tf_left.transform.translation.x,
                                                      tf_left.transform.translation.y,
                                                      tf_left.transform.translation.z,
                                                      tf_left.transform.rotation.w,
                                                      tf_left.transform.rotation.x,
                                                      tf_left.transform.rotation.y,
                                                      tf_left.transform.rotation.z
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

    moveit_test = MoveItTest()
    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # go down to grasp (return to parallel, go down, then rotate again)
    q_rot = quaternion_from_euler(0, -y_ang, 0)
    q_orig = arm_target_pose_np[3:]
    q_new = quaternion_multiply(q_rot, q_orig)
    arm_target_pose_np[3:] = q_new
    arm_target_pose_np[2] = arm_target_pose_np[2] - 0.055
    q_rot = quaternion_from_euler(0, y_ang, 0)
    q_orig = arm_target_pose_np[3:]
    q_new = quaternion_multiply(q_rot, q_orig)
    arm_target_pose_np[3:] = q_new

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    moveit_test = MoveItTest()
    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # close hand
    hand_api.close_hand(0.95)
    print('Closed!')

    # lift hand
    q_rot = quaternion_from_euler(0, -y_ang, 0)
    q_orig = arm_target_pose_np[3:]
    q_new = quaternion_multiply(q_rot, q_orig)
    arm_target_pose_np[3:] = q_new
    arm_target_pose_np[2] = arm_target_pose_np[2] + 0.07
    q_rot = quaternion_from_euler(0, y_ang, 0)
    q_orig = arm_target_pose_np[3:]
    q_new = quaternion_multiply(q_rot, q_orig)
    arm_target_pose_np[3:] = q_new

    publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = get_pose_stamped_from_arr(arm_target_pose_np)

    moveit_test = MoveItTest()
    print ("Planning trajectory")
    moveit_test.test_srv(arm_target_pose)

    # close more
    hand_api.close_hand(0.9)
    print('Closed!')
