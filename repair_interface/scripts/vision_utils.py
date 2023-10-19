"""
Helper functions for detecting the fragments
"""

import numpy as np
import rospy
import tf2_ros
import open3d as o3d
import geometry_msgs
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2
import pytransform3d.rotations as pyrot
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose


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
        if np.linalg.norm(p) > 0.65:
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