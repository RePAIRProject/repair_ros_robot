#!/usr/bin/env python
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import subprocess
import rospy
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import tf2_ros
import geometry_msgs.msg

from vision_utils import get_transform


def callback(point_cloud):
    pc = []
    for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
        pc.append([p[0], p[1], p[2]])

    xyz = np.asarray(pc)
   
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    pcd.points = o3d.utility.Vector3dVector(xyz)
    
    tf_camera_to_world = get_transform(parent_frame="working_surface_link", child_frame="camera_depth_optical_frame")
    tran = np.array([tf_camera_to_world.transform.translation.x, tf_camera_to_world.transform.translation.y, tf_camera_to_world.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_camera_to_world.transform.rotation.w,
                                                                    tf_camera_to_world.transform.rotation.x,
                                                                    tf_camera_to_world.transform.rotation.y,
                                                                    tf_camera_to_world.transform.rotation.z]))
    pcd.rotate(rot, center=(0, 0, 0)).translate(tran, relative=True)
    xyz = np.asarray(pcd.points)
    
    idx = np.where(xyz[:, 2] < 0.08)[0]
    xyz = xyz[idx]
    
    points = []
    for i in range(xyz.shape[0]):
        points.append([xyz[i, 0], xyz[i, 1], xyz[i, 2]])

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    
    header = Header()
    header.frame_id = "working_surface_link"
    filtered_pc = point_cloud2.create_cloud(header, fields, points)
    filtered_pc.header.stamp = rospy.Time.now()
    pub.publish(filtered_pc)

if __name__ == "__main__":
    rospy.init_node('cloud_filter', anonymous=True)

    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
    pub = rospy.Publisher('/my_gen3/filtered_pc', PointCloud2, queue_size=10)

    rospy.spin()
