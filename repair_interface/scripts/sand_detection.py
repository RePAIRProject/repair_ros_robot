#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped, Point

from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import namedtuple
import pyrealsense2 as rs
from ultralytics import YOLO
import vedo 
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import PointCloud2
import copy
import open3d as o3d    
from align_utils import get_points_from_ros, align_with_icp

class SandDetection():
    def __init__(self):
        self.rgb_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback_rgb)
        self.depth_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback_depth)

        # Subscribe to the depth and RGB image topics
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        
        # self.pointcloud = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)

        

        # To hold the camera intrinsics and alignment
        self.CameraIntrinsics = namedtuple("CameraIntrinsics", ["fx", "fy", "cx", "cy", "distortion_coeffs"])

        # Store intrinsics for RGB and Depth cameras
        self.rgb_intrinsics = None
        self.depth_intrinsics = None
        self.rgb_image = None
        self.depth_image = None
        self.detection_model_g29 = YOLO('/home/repair/dev/repair_vision/checkpoints/yolo_2D_recognition_group29.pt')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    # Camera info callback
    def camera_info_callback_rgb(self, msg):
        global rgb_intrinsics
        # Extract intrinsics from the camera_info message
        rgb_intrinsics = self.CameraIntrinsics(
            fx=msg.K[0], fy=msg.K[4], cx=msg.K[2], cy=msg.K[5], distortion_coeffs=msg.D
        )
        #rospy.loginfo("Received RGB Camera Info: Intrinsics - fx: {}, fy: {}, cx: {}, cy: {}".format(
        #    rgb_intrinsics.fx, rgb_intrinsics.fy, rgb_intrinsics.cx, rgb_intrinsics.cy))

    def camera_info_callback_depth(self, msg):
        # Create an rs.intrinsics object manually
        intrinsics = rs.intrinsics()
        intrinsics.width = msg.width
        intrinsics.height = msg.height
        intrinsics.ppx = msg.K[2]
        intrinsics.ppy = msg.K[5]
        intrinsics.fx = msg.K[0]
        intrinsics.fy = msg.K[4]
        # intrinsics.model = rs.distortion.brown_conrady  # Brown-Conrady model for distortion
        # intrinsics.coeffs = msg.D
        
        self.depth_intrinsics = intrinsics
        #rospy.loginfo("Received Depth Camera Info: Intrinsics - fx: {}, fy: {}, cx: {}, cy: {}".format(
        #    self.depth_intrinsics.fx, self.depth_intrinsics.fy, self.depth_intrinsics.ppx, self.depth_intrinsics.ppy))
        #     self.depth_intrinsics.fx, self.depth_intrinsics.fy, self.depth_intrinsics.cx, self.depth_intrinsics.cy))

    # def pointcloud_callback(self, msg):
    #     pc = []

    #     for p in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
    #         if np.linalg.norm(p) > 0.65:
    #             pc.append([p[0], p[1], p[2]])
    
    # Segmentation of Point Cloud
    

    def rgb_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            #rospy.loginfo("Received RGB image")
            # Optionally, show the image using OpenCV
            # cv2.imshow("RGB Image", self.rgb_image)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logerr("Error converting RGB image: %s", str(e))

    def depth_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
            #rospy.loginfo("Received Depth image")
            # Optionally, show the image using OpenCV
            # cv2.imshow("Depth Image", self.depth_image)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logerr("Error converting Depth image: %s", str(e))


    def transform_pose_array_to_world(self, input_pose, from_frame, to_frame):
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        # pose_stamped.header.stamp = rospy.Time.now()
        # rospy.sleep(1)
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.5))
            print(output_pose_stamped.header)
            return output_pose_stamped.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('hello')
            raise

    # def transform_pose_array_to_world(self, pose_array_msg):

    #     # Prepare the transformed PoseArray message
    #     transformed_pose_array = PoseArray()
    #     transformed_pose_array.header = pose_array_msg.header
    #     transformed_pose_array.header.frame_id = "world"

    #     try:
    #         # Wait for the transform to be available
    #         rospy.sleep(1)  # Allow some time for the transform to be received

    #         # Get the transform from camera_color_optical_frame to world frame
    #         transform = self.tf_buffer.lookup_transform('world', 'camera_color_optical_frame', rospy.Time(0))
    #         print(transform)
    #         # Loop through each pose in the PoseArray and transform it
    #         for pose in pose_array_msg.poses:
    #             transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
    #             # Append the transformed pose to the new PoseArray
    #             transformed_pose_array.poses.append(transformed_pose.pose)

    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         rospy.logerr("Transform not available yet")

    #     return transformed_pose_array
    
    def mesh2pcl(self, mesh):
        pcl = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(np.asarray(mesh.vertices)))
        return pcl
    
    def lucas_code(self, pose_array_pub):
        # while True:
        T_opencv2rviz = np.eye(4)
        while not rospy.is_shutdown():
            points3d_rs = get_points_from_ros()
            
            img2draw = self.rgb_image.copy()
            det_res = self.detection_model_g29(self.rgb_image)
            points_in_3d_space = []
            vedo_spheres = []
            for bbox in det_res[0].boxes:
                xywh = bbox.xywh[0].cpu().numpy()
                xyxy = bbox.xyxy[0].cpu().numpy()
                centerx = np.floor(xyxy[0] + (xyxy[2]-xyxy[0]) / 2).astype(int)
                centery = np.floor(xyxy[1] + (xyxy[3]-xyxy[1]) / 2).astype(int)
                depth = self.depth_image[centery, centerx] #depth_raw_frame.get_distance(coordinates[0], coordinates[1])
                point_in_3d_space = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [centery, centerx], depth)
                fragment_id = int(bbox.cls.item())
                # print("point 3d", point_in_3d_space, 'frag', fragment_id)
                cv2.rectangle(img2draw, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 3)  # Green rectangle with thickness 3
                cv2.circle(img2draw, (centerx, centery), 2, (0, 0, 255), 3)
                test2 = np.asarray([int(xywh[0]), int(xywh[1])]).astype(int)
                cv2.circle(img2draw, (test2[0], test2[1]), 2, (255, 0, 0), 1)
                
                rospy.loginfo(f"Point in 3D: {point_in_3d_space}, Fragment ID: {fragment_id}, x {centerx}, y {centery}")

                # Create Pose from the 3D point (position only)
                pose = Pose()
                pose.position.x = point_in_3d_space[0]/1000 
                pose.position.y = point_in_3d_space[1]/1000 
                pose.position.z = point_in_3d_space[2]/1000 #* 1.5
                print('before', pose.position)
                T_opencv2rviz = np.asarray([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                pose_rviz = np.dot(T_opencv2rviz, np.asarray([pose.position.x, pose.position.y, pose.position.z, 1]))

                pose.position.x = pose_rviz[0]      
                pose.position.y = pose_rviz[1]      
                pose.position.z = pose_rviz[2]      
                print('after', pose.position)

                # Optionally, you can set orientation (we'll keep it as the identity here)
                #0.4999998, -0.4996018, 0.4999998, 0.5003982
                quat = quaternion_from_euler(0, 0, 0) #0.52, 0, 1.5707963267948966)
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]  # Identity quaternion (no rotation)

                # world_pose = pose #self.transform_pose_array_to_world(pose, "camera_color_optical_frame", "world")

                points_in_3d_space.append(vedo.Point(point_in_3d_space))
                vedo_sphere = vedo.Sphere(point_in_3d_space, c="red", r=50)#.apply_transform(T_opencv2rviz)
                vedo_spheres.append(vedo_sphere)

            # test 1
            pts3d = []
            colors3d = []
            for _x in range(self.rgb_image.shape[1]):
                for _y in range(self.rgb_image.shape[0]):
                    pts3d.append(rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [_y, _x], self.depth_image[_y, _x]))
                    colors3d.append(self.rgb_image[_y, _x])

            realsense_pcl = vedo.Points(points3d_rs * 1000)
            print('mean point realsense', np.mean(points3d_rs))
            # pts3d = np.dot(T_opencv2rviz, np.asarray([pose.position.x, pose.position.y, pose.position.z, 1]))
            vedo_pcl = vedo.Points(pts3d)
            vedo_pcl.apply_transform(T_opencv2rviz)
            # vedo_spheres[0].apply_transform(T_opencv2rviz)
            vedo_pcl.pointcolors = np.asarray(colors3d)
            print('mean point vedo_pcl', np.mean(pts3d))
            
            # OPEN3d
            voxel_size = 5
            realsense_pcl_o3d = self.mesh2pcl(vedo.utils.vedo2open3d(realsense_pcl))
            vedo_pcl_o3d = self.mesh2pcl(vedo.utils.vedo2open3d(vedo_pcl))
            # print(realsense_pcl_o3d)
            
            rs_d, rs_f = preprocess_point_cloud(realsense_pcl_o3d, voxel_size)
            rp_d, rp_f = preprocess_point_cloud(vedo_pcl_o3d, voxel_size)
            # print(rs_d)
            # print(rp_d)
            result_T = align_with_icp(rp_d, rs_d, voxel_size=voxel_size, fast=False)

            poses = []
            for vd_pt3d in points_in_3d_space:
                vd_pt3d.apply_transform(T_opencv2rviz).apply_transform(result_T.transformation)
                poses.append(pt3d_to_pose(vd_pt3d.vertices[0]))

            # result_ransac = execute_global_registration(rp_d, rs_d,
            #                                 rp_f, rs_f,
            #                                 voxel_size)

            # # reg_p2p = o3d.pipelines.registration.registration_icp(
            # #     vedo_pcl_o3d, realsense_pcl_o3d, 15, np.eye(4),
            # #     o3d.pipelines.registration.TransformationEstimationPointToPoint())
            #     # realsense_pcl_o3d = vedo.utils.vedo2open3d(realsense_pcl)

            # refined_result = refine_registration(rp_d, rs_d, result_ransac, voxel_size)
            

            # vedo.show(vedo_pcl, realsense_pcl, axes=1, interactive=True).close()
            # aligned_reprojected_pcl = vedo_pcl.clone().align_to(realsense_pcl, invert=False, iters=1000, rigid=True)
            # txt = aligned_reprojected_pcl.transform.__str__()
            print(result_T)
            print(result_T.transformation)
            # print(refined_result)
            # print(refined_result.transformation)
            # vedo.show(realsense_pcl, vedo_pcl.apply_transform(result_T.transformation), vedo_spheres[0].apply_transform(result_T.transformation), axes=1, interactive=True).close()
            # vedo.show(realsense_pcl, vedo_pcl, vedo_spheres[0], axes=1, interactive=True).close()
            # cv2.imshow('detection', img2draw)
            # cv2.imshow('rgb', self.rgb_image)
            # cv2.waitKey(1)
            # Create the PoseArray message and populate it with the poses
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = rospy.Time.now()
            pose_array_msg.header.frame_id = "camera_color_optical_frame"  # Use the appropriate frame_id

            # Add poses to PoseArray
            pose_array_msg.poses = poses


            # Publish the PoseArray
            pose_array_pub.publish(pose_array_msg)
            
            # Sleep to maintain the loop rate
            #rospy.sleep(1)  # Adjust sleep time as needed

            # pipeline = rs.pipeline()
            # config = rs.config()

            # # Get device product line for setting a supporting resolution
            # pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            # pipeline_profile = config.resolve(pipeline_wrapper)
            # device = pipeline_profile.get_device()
            # depth_sensor = device.first_depth_sensor()
            # # Get depth scale of the device
            # self.depth_scale = depth_sensor.get_depth_scale()
            # # Create an align object
            # align_to = rs.stream.color

            # self.align = rs.align(align_to)
            # device_product_line = str(device.get_info(rs.camera_info.product_line))
            # print("device product line:", device_product_line)
            # config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, 6)
            # config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, 6)
            
            
            
            # pass

def pt3d_to_pose(pt3d):
    pose = Pose()
    pose.position.x = pt3d[0]/1000 
    pose.position.y = pt3d[1]/1000 
    pose.position.z = pt3d[2]/1000 #* 1.5
    quat = quaternion_from_euler(0, 0, 0) #0.52, 0, 1.5707963267948966)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]  # Identity quaternion (no rotation)
    return pose 

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def refine_registration(source, target, result_ransac, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    # pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    #     pcd_down,
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, None

if __name__ == '__main__':
    node_name = "Sand_detection"
    rospy.init_node(node_name)
    
        # Initialize the publisher for PoseArray
    pose_array_pub = rospy.Publisher('/detection/points', PoseArray, queue_size=10)

    detection = SandDetection()
    detection.lucas_code(pose_array_pub)






    # # Start the main processing loop
    # node_name.lucas_code(pose_array_pub)

    rospy.spin()
