#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped, Point
import os 
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import namedtuple
import pyrealsense2 as rs
from ultralytics import YOLO
import vedo 
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import PointCloud2
import copy
import open3d as o3d    
from align_utils import get_points_from_ros, align_with_icp
import json 

"""
This script is used to detect fragments in the sand in the color image!
Once we have them we use the depth to reproject the detected objects in 3D. 
We realized that the pointcloud from the realsense is NOT aligned with the reprojection from RGBD.
Therefore (since we know the realsense to world tranformation) we need to align the reprojected RGBD point cloud 
with the realsense point cloud. This is done with ICP, it worked, but the robustness could be improved. 
We know that their position is similar and there is only some offset, so ICP should be good enough for this alignment.
"""
class SandRecognition():
    def __init__(self, data_folder: str, model_name: str, placement_file: str):
    # can we use parameters? otherwise just
    # def __init__(self):
        self.rgb_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback_rgb)
        self.depth_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback_depth)

        # Subscribe to the depth and RGB image topics
        # they are aligned to the RGB (and not to the pointcloud)
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)       

        # To hold the camera intrinsics and alignment
        self.CameraIntrinsics = namedtuple("CameraIntrinsics", ["fx", "fy", "cx", "cy", "distortion_coeffs"])

        # Store intrinsics for RGB and Depth cameras
        self.rgb_intrinsics = None
        self.depth_intrinsics = None
        self.rgb_image = None
        self.depth_image = None
        self.T_opencv2rviz = np.asarray([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # self.T_opencv2rviz = np.asarray([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        self.voxel_size = 5

        # is this needed?
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        ###############################################
        # PARAMETERS FOR RECOGNITION
        self.data_folder = data_folder #"/home/repair/dev/repair_vision/checkpoints"
        if not os.path.exists(self.data_folder):
            raise Exception(f"Missing root folder for models.\n{self.data_folder} not existing")
        self.model_full_path = os.path.join(self.data_folder, model_name)
        if not os.path.exists(self.model_full_path):
            raise Exception(f"No trained model found at {self.model_full_path}, please check the path")
        # in-house trained YOLO models for recognition
        self.recognition_model = YOLO(self.model_full_path)

        self.fresco_placement_file_path = os.path.join(self.data_folder, placement_file)
        if not os.path.exists(self.model_full_path):
            raise Exception(f"No placement json file found at {self.fresco_placement_file_path}, please check the path")
        with open(self.fresco_placement_file_path, 'r') as fpf:
            self.placements_dict = json.load(fpf)
        ###############################################

        # i am not proud, but i need it to work

        self.hardcoded_alignment_matrix = np.asarray([[0.99969224,0.02345452,-0.00808201, -93.17143693],
                                                    [-0.02330734,0.99956903,0.01784734, 100.71646801],
                                                    [0.00849713,-0.01765348,0.99980806,-6.69656267],
                                                    [0,0,0,1]])

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
    

    def rgb_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting RGB image: %s", str(e))

    def depth_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            rospy.logerr("Error converting Depth image: %s", str(e))


    def transform_pose_array_to_world(self, poses):
        transformed_poses = []
        for pose in poses:
            # Transform the poses into the world frame
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = "camera_color_optical_frame"
            # pose_stamped.header.stamp = rospy.Time.now()
            # rospy.sleep(1)
            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, "world", rospy.Duration(1.0))
                transformed_poses.append(output_pose_stamped.pose)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('ERROR')
                raise

        transformed_pose_array_msg = PoseArray()
        transformed_pose_array_msg.header.stamp = rospy.Time.now()
        transformed_pose_array_msg.header.frame_id = "world"  # Now in the world frame
        transformed_pose_array_msg.poses = transformed_poses

        return transformed_pose_array_msg

        # pose_stamped = tf2_geometry_msgs.PoseStamped()
        # pose_stamped.pose = input_pose
        # pose_stamped.header.frame_id = from_frame
        # try:
        #     # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        #     output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(0.5))
        #     print(output_pose_stamped.header)
        #     return output_pose_stamped.pose
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     print('hello')
        #     raise
    
    def mesh2pcl(self, mesh):
        pcl = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(np.asarray(mesh.vertices)))
        return pcl
    
    # def recognize_and_publish_DEBUG(self, pose_array_pub, verbosity=1):
        # """
        # We use for debugging (3D interactive visualization to make sure everything is correct)
        # """
        # # main loop
        # T_opencv2rviz = np.eye(4)
        # while not rospy.is_shutdown():
        #     points3d_rs = get_points_from_ros()
        #     img2draw = self.rgb_image.copy()
        #     # recognition
        #     det_res = self.recognition_model(self.rgb_image)
        #     points_in_3d_space = []
        #     vedo_spheres = []

        #     # each box has a faragment
        #     for bbox in det_res[0].boxes:
        #         xywh = bbox.xywh[0].cpu().numpy()
        #         xyxy = bbox.xyxy[0].cpu().numpy()
        #         centerx = np.floor(xyxy[0] + (xyxy[2]-xyxy[0]) / 2).astype(int)
        #         centery = np.floor(xyxy[1] + (xyxy[3]-xyxy[1]) / 2).astype(int)
        #         depth = self.depth_image[centery, centerx] #depth_raw_frame.get_distance(coordinates[0], coordinates[1])
        #         point_in_3d_space = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [centery, centerx], depth)
        #         fragment_id = int(bbox.cls.item())
        #         # print("point 3d", point_in_3d_space, 'frag', fragment_id)
        #         cv2.rectangle(img2draw, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 3)  # Green rectangle with thickness 3
        #         cv2.circle(img2draw, (centerx, centery), 2, (0, 0, 255), 3)
        #         test2 = np.asarray([int(xywh[0]), int(xywh[1])]).astype(int)
        #         cv2.circle(img2draw, (test2[0], test2[1]), 2, (255, 0, 0), 1)
                
        #         rospy.loginfo(f"Point in 3D: {point_in_3d_space}, Fragment ID: {fragment_id}, x {centerx}, y {centery}")

        #         # Create Pose from the 3D point (position only)
        #         pose = Pose()
        #         pose.position.x = point_in_3d_space[0]/1000 
        #         pose.position.y = point_in_3d_space[1]/1000 
        #         pose.position.z = point_in_3d_space[2]/1000 #* 1.5
        #         if verbosity > 1:
        #             print('before', pose.position)
        #         T_opencv2rviz = np.asarray([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        #         pose_rviz = np.dot(T_opencv2rviz, np.asarray([pose.position.x, pose.position.y, pose.position.z, 1]))

        #         pose.position.x = pose_rviz[0]      
        #         pose.position.y = pose_rviz[1]      
        #         pose.position.z = pose_rviz[2]      
        #         if verbosity > 1:
        #             print('after', pose.position)

        #         # Optionally, you can set orientation (we'll keep it as the identity here)
        #         #0.4999998, -0.4996018, 0.4999998, 0.5003982
        #         quat = quaternion_from_euler(0, 0, 0) #0.52, 0, 1.5707963267948966)
        #         pose.orientation.x = quat[0]
        #         pose.orientation.y = quat[1]
        #         pose.orientation.z = quat[2]
        #         pose.orientation.w = quat[3]  # Identity quaternion (no rotation)

        #         # world_pose = pose #self.transform_pose_array_to_world(pose, "camera_color_optical_frame", "world")

        #         points_in_3d_space.append(vedo.Point(point_in_3d_space))
        #         vedo_sphere = vedo.Sphere(point_in_3d_space, c="red", r=50)#.apply_transform(T_opencv2rviz)
        #         vedo_spheres.append(vedo_sphere)

        #     # MISALIGNMENT CORRECTION
        #     # align reprojected pointcloud with realsense pointcloud
        #     pts3d = []
        #     colors3d = []
        #     for _x in range(self.rgb_image.shape[1]):
        #         for _y in range(self.rgb_image.shape[0]):
        #             pts3d.append(rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [_y, _x], self.depth_image[_y, _x]))
        #             colors3d.append(self.rgb_image[_y, _x])

        #     realsense_pcl = vedo.Points(points3d_rs * 1000)
        #     vedo_pcl = vedo.Points(pts3d)
        #     vedo_pcl.apply_transform(T_opencv2rviz)
        #     vedo_pcl.pointcolors = np.asarray(colors3d)
        #     if verbosity > 1:
        #         print('mean point realsense', np.mean(points3d_rs))
        #         print('mean point vedo_pcl', np.mean(pts3d))
            
        #     # OPEN3D
        #     voxel_size = 5
        #     realsense_pcl_o3d = self.mesh2pcl(vedo.utils.vedo2open3d(realsense_pcl))
        #     vedo_pcl_o3d = self.mesh2pcl(vedo.utils.vedo2open3d(vedo_pcl))
        #     rs_d, rs_f = preprocess_point_cloud(realsense_pcl_o3d, voxel_size)
        #     rp_d, rp_f = preprocess_point_cloud(vedo_pcl_o3d, voxel_size)
        #     align_to_realsenseT = align_with_icp(rp_d, rs_d, voxel_size=voxel_size, fast=False)

        #     poses = []
        #     for vd_pt3d in points_in_3d_space:
        #         vd_pt3d.apply_transform(T_opencv2rviz).apply_transform(align_to_realsenseT.transformation)
        #         poses.append(pt3d_to_pose(vd_pt3d.vertices[0]))

        #     if verbosity > 1:
        #         print("# ALIGNMENT TO REALSENSE")
        #         print(align_to_realsenseT)
        #         print(align_to_realsenseT.transformation)

        #     pose_array_msg = PoseArray()
        #     pose_array_msg.header.stamp = rospy.Time.now()
        #     pose_array_msg.header.frame_id = "camera_color_optical_frame"  # Use the appropriate frame_id

        #     # Add poses to PoseArray
        #     pose_array_msg.poses = poses

        #     # Publish the PoseArray
        #     pose_array_pub.publish(pose_array_msg)

        #     ### OTHER ATTEMPTS FOR REGISTRATION
        #     # result_ransac = execute_global_registration(rp_d, rs_d,
        #     #                                 rp_f, rs_f,
        #     #                                 voxel_size)

        #     # # reg_p2p = o3d.pipelines.registration.registration_icp(
        #     # #     vedo_pcl_o3d, realsense_pcl_o3d, 15, np.eye(4),
        #     # #     o3d.pipelines.registration.TransformationEstimationPointToPoint())
        #     #     # realsense_pcl_o3d = vedo.utils.vedo2open3d(realsense_pcl)

        #     # refined_result = refine_registration(rp_d, rs_d, result_ransac, voxel_size)

    def recognize_and_publish(self, pose_array_pub, id_array_pub, rotation_array_pub, verbosity=1, debug=False, show_image_feed=False, use_hardcore=True):
        """
        The main loop with the 2D color recognition, reprojection and registration 
        """
        # main loop
        counter = 0
        while not rospy.is_shutdown():
            points3d_rs = get_points_from_ros()
            
            det_res = self.recognition_model(self.rgb_image, conf=0.5, iou=0.2)
            print(det_res)

            # We collect information which will be published
            points_in_3d_space = []
            fragments_ids = []
            fragments_rotation = []

            #cv2.imwrite(f'rgb_{counter}.png', self.rgb_image)
            counter += 1
            
            if debug ==True:
                vedo_spheres = []
                img2draw = self.rgb_image.copy()
            for obb in det_res[0].obb:
                xywhr = obb.xywhr[0]
                centerx = np.round(obb.xywhr[0][0].item()).astype(int)
                centery = np.round(obb.xywhr[0][1].item()).astype(int)
                fragment_id = int(obb.cls.item())
                rotation = xywhr[4].item()
                print("rotation", rotation)
                

                # name and group to fetch the assembly position
                name = det_res[0].names[fragment_id]
                group = name[name.index('G')+1:]
                fragment_name = name[:name.index('G')-1]
                fragment_id = int(fragment_name[-5:])
                fragment_name = fragment_name.split('_')[0] + '_' + fragment_name.split('_')[1]
                
                # rotations and ids for publishing
                fragments_rotation.append(rotation)
                fragments_ids.append(fragment_id)

                # we fetch here the final position where it should be placed
                assembly_position = self.placements_dict[f'group_{group}'][f'{fragment_name}_intact_mesh']
                print(f"found {fragment_name} (group {group}) with center in {centerx:.2f}, {centery:.2f} (pixel coordinates). \nIt should be placed (in real world coordaintes) in: {assembly_position['trans_x']}, {assembly_position['trans_y']}\n rotated by {assembly_position['ori_yaw']}")       
                # breakpoint()
                if debug == True:
                    cv2.rectangle(img2draw, (int(obb.xyxy[0][0].item()), int(obb.xyxy[0][1].item())), (int(obb.xyxy[0][2].item()), int(obb.xyxy[0][3].item())), (0, 255, 0), 3)  # Green rectangle with thickness 3
                    cv2.circle(img2draw, (centerx, centery), 2, (0, 0, 255), 3)
                    

                # 3D 
                depth = self.depth_image[centery, centerx]
                point_in_3d_space = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [centery, centerx], depth)
                points_in_3d_space.append((vedo.Point(point_in_3d_space), rotation))
                if debug == True:
                    vedo_sphere = vedo.Sphere(point_in_3d_space, c="red", r=50).apply_transform(self.T_opencv2rviz)
                    vedo_spheres.append(vedo_sphere)

            if debug == True:
                cv2.imshow("image", img2draw)
                cv2.waitKey(1)
            if show_image_feed == True:
                img2draw = self.rgb_image.copy()
                if len(det_res[0].obb) > 0:
                    for obb in det_res[0].obb:
                        xywhr = obb.xywhr[0]
                        centerx = np.round(obb.xywhr[0][0].item()).astype(int)
                        centery = np.round(obb.xywhr[0][1].item()).astype(int)
                        cv2.circle(img2draw, (centerx, centery), 2, (0, 0, 255), 3)
                        rectpts = obb.xyxyxyxy.cpu().numpy().reshape(4,2).astype(np.int32)
                        img2draw = cv2.polylines(img2draw, [rectpts], isClosed=True, color=(0, 0, 255))

                cv2.imshow(f'recognition', img2draw)
                print(f'detected {len(det_res[0].obb)} objects')
                cv2.waitKey(1)
            # MISALIGNMENT CORRECTION
            # align reprojected pointcloud with realsense pointcloud

            # STEP 1:
            # Creating the vedo pointcloud from the image (will be used later for alignment)
            pts3d = []
            colors3d = []
            for _x in range(self.rgb_image.shape[1]):
                for _y in range(self.rgb_image.shape[0]):
                    pts3d.append(rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [_y, _x], self.depth_image[_y, _x]))
                    colors3d.append(self.rgb_image[_y, _x])
            realsense_pcl = vedo.Points(points3d_rs * 1000) # meters to millimeters
            vedo_pcl = vedo.Points(pts3d)
            vedo_pcl.apply_transform(self.T_opencv2rviz)
            vedo_pcl.pointcolors = np.asarray(colors3d)
            if verbosity > 1:
                print('mean point realsense', np.mean(points3d_rs))
                print('mean point vedo_pcl', np.mean(pts3d))
            
            # STEP 2:
            if use_hardcore == False:
                # ICP Alignment (to the RealSense point cloud)
                realsense_pcl_o3d = self.mesh2pcl(vedo.utils.vedo2open3d(realsense_pcl))
                vedo_pcl_o3d = self.mesh2pcl(vedo.utils.vedo2open3d(vedo_pcl))
                rs_d, rs_f = preprocess_point_cloud(realsense_pcl_o3d, self.voxel_size)
                rp_d, rp_f = preprocess_point_cloud(vedo_pcl_o3d, self.voxel_size)
                align_to_realsenseT = align_with_icp(rp_d, rs_d, voxel_size=self.voxel_size, fast=False)

            # STEP 3:
            # for each detected point in the scene, apply the transformation to get it to the correct location
            poses = []
            transformed_points_in_3d_space = []
            for vd_pt3d, rotation in points_in_3d_space:
                # print(vd_pt3d)
                vd_pt3d.apply_transform(self.T_opencv2rviz)
                if use_hardcore == False:
                    vd_pt3d.apply_transform(align_to_realsenseT.transformation)
                else:
                    vd_pt3d.apply_transform(self.hardcoded_alignment_matrix)
                # print("transform")
                # print(vd_pt3d)
                transformed_points_in_3d_space.append(vd_pt3d)
                poses.append(pt3d_to_pose(vd_pt3d.vertices[0], rotation=rotation))

            if verbosity > 0 and use_hardcore == False:
                print("\n# ALIGNMENT TO REALSENSE")
                print(align_to_realsenseT)
                print(align_to_realsenseT.transformation)
                print("\n# ALIGNMENT TO REALSENSE")


            if debug == True:
                # vedo.show(vedo_pcl, realsense_pcl, vedo_spheres, transformed_points_in_3d_space, axes=1, interactive=True).close()
                if use_hardcore == False:
                    vedo.show(vedo_pcl.apply_transform(align_to_realsenseT.transformation), realsense_pcl, vedo_spheres, points_in_3d_space, axes=1, interactive=True).close()
                else:
                    vedo.show(vedo_pcl.apply_transform(self.hardcoded_alignment_matrix), realsense_pcl, vedo_spheres, points_in_3d_space, axes=1, interactive=True).close()
            
            # STEP 4:
            # Create the PoseArray and publish
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = rospy.Time.now()
            pose_array_msg.header.frame_id = "camera_color_optical_frame"  # Use the appropriate frame_id
            # # Add poses to PoseArray
            pose_array_msg.poses = poses
            #pose_array_msg = self.transform_pose_array_to_world(poses)
            # Publish the PoseArray
            pose_array_pub.publish(pose_array_msg)

            # we publish now also the ids
            id_array_msg = Int32MultiArray()
            id_array_msg.data = fragments_ids
            id_array_pub.publish(id_array_msg)

            # and the rotation alone for now
            rotation_array_msg = Float32MultiArray()
            rotation_array_msg.data = fragments_rotation
            rotation_array_pub.publish(rotation_array_msg)

def pt3d_to_pose(pt3d, rotation=0):
    """
    Creates a Pose object and fills it (checking sizes, conversion m to mm, and rotation if there is)
    """
    pose = Pose()
    pose.position.x = pt3d[0]/1000 
    pose.position.y = pt3d[1]/1000 
    pose.position.z = pt3d[2]/1000 #* 1.5
    quat = quaternion_from_euler(1.1836725, 0, 0) #0.52, 0, 1.5707963267948966)
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

    node_name = "Sand_Recognition"
    rospy.init_node(node_name)
    verbosity_level = 1 # increase value to print debug information in the recognition code

    # Initialize the publisher for PoseArray
    pose_array_pub = rospy.Publisher('/recognition/points', PoseArray, queue_size=10)
    id_array_pub = rospy.Publisher('/recognition/ids', Int32MultiArray, queue_size=10)
    rotation_array_pub = rospy.Publisher('/recognition/rotations', Float32MultiArray, queue_size=10)

    recognition = SandRecognition(data_folder="/home/repair/repair_ws/src/repair_ros_robot/repair_interface/config/weights_mix", 
                                  model_name="best.pt",
                                  placement_file='int_week_placements.json')
    recognition.recognize_and_publish(pose_array_pub, id_array_pub, rotation_array_pub, \
                                    verbosity=verbosity_level, debug=False, show_image_feed=True, \
                                    use_hardcore=True)

    rospy.spin()
