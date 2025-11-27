#!/usr/bin/env python3

from matplotlib.pyplot import box
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
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import PointCloud2
import copy
import open3d as o3d    
from align_utils import get_points_from_ros, align_with_icp
import json 
import argparse
from std_msgs.msg import Header


from repair_interface.msg import RecognitionData, PlacementData, PlacedPieces

"""
This script is used to detect fragments in the sand in the color image!
Once we have them we use the depth to reproject the detected objects in 3D. 
We realized that the pointcloud from the realsense is NOT aligned with the reprojection from RGBD.
Therefore (since we know the realsense to world tranformation) we need to align the reprojected RGBD point cloud 
with the realsense point cloud. This is done with ICP, it worked, but the robustness could be improved. 
We know that their position is similar and there is only some offset, so ICP should be good enough for this alignment.
"""
class SandRecognition():
    def __init__(self, data_folder: str, model_name: str, placement_file: str, use_gazebo: bool):
        self.use_gazebo = use_gazebo
        print("USE_GAZEBO: ", self.use_gazebo)
        
        # one-time setup (e.g., in __init__)
        if(use_gazebo):
            self.pose_array_pub = rospy.Publisher("debug/poses", PoseArray, queue_size=1, latch=True)
            self.optical_frame = "camera_depth_optical_frame"
        else:
            self.pose_array_pub = rospy.Publisher("debug/poses", PoseArray, queue_size=1, latch=True)
            self.optical_frame = "camera_color_optical_frame"
            

        # To hold the camera intrinsics and alignment
        self.CameraIntrinsics = namedtuple("CameraIntrinsics", ["fx", "fy", "cx", "cy", "distortion_coeffs"])
    
        self.rgb_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback_rgb)
        #self.depth_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback_depth)
        if self.use_gazebo:
            self.depth_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback_depth)
        else:
            self.depth_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback_depth)

        # Subscribe to the depth and RGB image topics
        # they are aligned to the RGB (and not to the pointcloud)
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        if self.use_gazebo:
            self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)       
        else:
            self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)  

        self.placed_pieces = []
        self.placed_pieces_sub = rospy.Subscriber('/placed_pieces', PlacedPieces, self.placed_pieces_callback)
        # Store intrinsics for RGB and Depth cameras
        self.rgb_intrinsics = None
        self.depth_intrinsics = None
        self.rgb_image = None
        self.depth_image = None
        self.T_opencv2rviz = np.asarray([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # self.T_opencv2rviz = np.asarray([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        self.voxel_size = 0.005 if self.use_gazebo else 5


        # is this needed?
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        ###############################################
        # PARAMETERS FOR RECOGNITION
        self.data_folder = data_folder #"/home/repair/dev/repair_vision/checkpoints"
        if not os.path.exists(self.data_folder):
            raise Exception(f"Missing root folder for models.\n{self.data_folder} not existing")
        self.model_full_path = model_name
        if not os.path.exists(self.model_full_path):
            raise Exception(f"No trained model found at {self.model_full_path}, please check the path")
        # in-house trained YOLO models for recognition
        self.recognition_model = YOLO(self.model_full_path)
        self.fresco_placement_file_path = placement_file
        if not os.path.exists(self.fresco_placement_file_path):
            raise Exception(f"No placement json file found at {self.fresco_placement_file_path}, please check the path")
        with open(self.fresco_placement_file_path, 'r') as fpf:
            self.placements_dict = json.load(fpf)
        ###############################################

        # i am not proud, but i need it to work

        self.hardcoded_alignment_matrix = np.asarray([[0.99969224,0.02345452,-0.00808201, -93.17143693],
                                                    [-0.02330734,0.99956903,0.01784734, 100.71646801],
                                                    [0.00849713,-0.01765348,0.99980806,-6.69656267],
                                                    [0,0,0,1]])
        
        self.principal_axis_history = None
        self.num_samples = 10
        self.prev_axis = None        # np.array([ux, uy])
        self.prev_angle = None

    def placed_pieces_callback(self, msg):
        self.placed_pieces = msg.placed_pieces.data

    def publish_pose_array(self, poses, frame_id="world"):
        pa = PoseArray()
        pa.header = Header()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = frame_id  # <- set this to your RViz fixed frame
        pa.poses = poses               # your list of geometry_msgs/Pose
        self.pose_array_pub.publish(pa)

    # Camera info callback
    def camera_info_callback_rgb(self, msg):
        global rgb_intrinsics
        # Extract intrinsics from the camera_info message
        rgb_intrinsics = self.CameraIntrinsics(
            fx=msg.K[0], fy=msg.K[4], cx=msg.K[2], cy=msg.K[5], distortion_coeffs=msg.D
        )
        # rospy.loginfo("Received RGB Camera Info: Intrinsics - fx: {}, fy: {}, cx: {}, cy: {}".format(
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
            if self.use_gazebo:
                depth_mm = bridge.imgmsg_to_cv2(msg, "16UC1")
                self.depth_image = depth_mm.astype(np.float32) / 4000.0
            else:
                self.depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")            
        except Exception as e:
            rospy.logerr("Error converting Depth image: %s", str(e))


    def transform_pose_array_to_world(self, poses):
        transformed_poses = []
        for pose in poses:
            # Transform the poses into the world frame
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = self.optical_frame
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
    

    def recognize_with_contraints(self, rgb_image, group_number, iou=1, placed_pieces=[], verbose=False):
        """
        Adds on top of the recognition a loop to limit the recognition to one object per class,
        to avoid duplicates.
        -------------
        Params:
        - rgb_image: the image from the realsense
        - group_number: the group we are working for the demo (15, 29 or 89)
        - iou: value for the "raw" recognition, use high values close to 1 (0.90 to 1) or just leave it to 1
        -------------
        Returns:
        - dicts with the pieces ids as key and as item another dictionary with 'box', 'conf' and 'mask' keys.
            Use the mask! 
                `output[piece_id]['mask']`
            The polygon can be accessed like this
                `polygon = detection['mask'].xy[0]`
        """
        conf_debug = 0.3 if verbose else 0.3
        detections = self.recognition_model(self.rgb_image, conf=conf_debug, iou=0.1, verbose=False)[0]
        final_detections = {}

        dict_mapping  = {"RPf_00096": 4,
                         "RPf_00097": 5,
                         "RPf_00103": 11,
                         "RPf_00104": 12,
                         "RPf_00106": 14,
                         "RPf_00107": 15,
                         "RPf_00109": 17, 
                         "RPf_00204": 1,
                         "RPf_00205": 2,
                         "RPf_00206": 3,
                         "RPf_00207": 4,
                         "RPf_00208": 5}
        

        if group_number == "29" or group_number == 29:
            group_number = 29
            classes = [1, 2, 3, 4, 5]
            if len(placed_pieces) > 0:
                print("received as placed pieces:", placed_pieces)
                print("received as placed pieces classes:", [dict_mapping[f'RPf_{str(pid).zfill(5)}'] for pid in placed_pieces])
                print("before:", classes)
                # remove already placed pieces from the classes to be detected
                classes = [cls for cls in classes if cls not in [dict_mapping[f'RPf_{str(pid).zfill(5)}'] for pid in placed_pieces]]
                print("after:", classes)
        elif group_number == "15" or group_number == 15:
            group_number = 15
            classes = [4, 5, 11, 12, 14, 17]
            if len(placed_pieces) > 0:
                print("received as placed pieces:", placed_pieces)
                print("received as placed pieces classes:", [dict_mapping[f'RPf_{str(pid).zfill(5)}'] for pid in placed_pieces])
                print("before:", classes)
                # remove already placed pieces from the classes to be detected
                classes = [cls for cls in classes if cls not in [dict_mapping[f'RPf_{str(pid).zfill(5)}'] for pid in placed_pieces]]
                print("after:", classes)
        elif group_number == "89" or group_number == 89:
            group_number = 89
            classes = [1, 2, 4, 5, 6, 7, 11]

        # loop for filtering
        for class_id in classes: 

            # Filter by class
            class_mask = detections.boxes.cls == class_id
            
            if class_mask.any():
                # Get confidences for this class
                class_confs = detections.boxes.conf[class_mask]
                    
                # Find index of max confidence
                best_idx = class_confs.argmax()

                # Get the best detection
                all_class_indices = class_mask.nonzero(as_tuple=True)[0]
                global_idx = all_class_indices[best_idx]
                # breakpoint()
                final_detections[class_id] = {
                    'name': detections.names[class_id][:-4],
                    'box': detections.boxes.xyxy[global_idx],
                    'conf': detections.boxes.conf[global_idx],
                    'mask': detections.masks[global_idx] if detections.masks else None
                }

        return final_detections
    
    
    def calculate_area(self, polygon, box):
        """Calculate area of the polygon"""
        # ==
        # area = cv2.contourArea(polygon)
        # ==
        x = polygon[:, 0]
        y = polygon[:, 1]
        area_polygon = 0.5 * np.abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
        box_area = (box[2]-box[0]) * (box[3]-box[1])
        # print("== Contour area:", area_polygon)
        # print("== Box Area:", box_area)
        return area_polygon

    def calculate_center_of_mass(self, polygon):
        """Calculate center of mass of the polygon"""
        M = cv2.moments(polygon)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = int(center_x), int(center_y)
        return cx, cy

    def calculate_rotation(self, polygon, method='PCA', imgdraw=None, fragment_name=""):
        """
        Calculates rotation from teh detected polygon. 
        Method can be:
        - 'PCA': estimates angle from polygon
        - 'OBB': compute OBB via cv2.minAreaRect and get angle from OBB
        ---------
        Returns:
        - angle in degrees
        """
        if method == 'PCA':
            from sklearn.decomposition import PCA 
            pca = PCA(n_components=2)
            pca.fit(np.squeeze(polygon))

            # First principal component
            axis = pca.components_[0]  # (ux, uy)

            # Normalize
            axis = axis / np.linalg.norm(axis)

            # Fix 180° ambiguity vs previous frame
            axis = self._fix_axis_flip(self.prev_axis, axis)

            axis_smoothed = self._update_axis_history(axis, fragment_name)
            
            # Get the angle from first principal component
            # angle_pca_rad = np.arctan2(pca.components_[0, 1], pca.components_[0, 0])
            angle_pca_rad = np.arctan2(axis_smoothed[1], axis_smoothed[0])
            angle = np.rad2deg(angle_pca_rad)
            
            self.prev_axis = axis_smoothed
            self.prev_angle = angle_pca_rad

            if imgdraw is not None:
                centerx, centery = self.calculate_center_of_mass(polygon)
                length = 70
                x2 = int(centerx + length * axis_smoothed[0])
                y2 = int(centery + length * axis_smoothed[1])
                x1 = int(centerx - length * axis_smoothed[0])
                y1 = int(centery - length * axis_smoothed[1])
                imgdraw = cv2.line(imgdraw, (x1, y1), (x2, y2), (255, 0, 0), 2)

        elif method == 'OBB':
            rect = cv2.minAreaRect(polygon)
            (centerx, centery), (width, height), angle = rect
            # angle is in deg, if you need to change, add
            # (and please comment or add a flag)
            # angle_rad = np.deg2rad(angle)
            if imgdraw is not None:
                length = 70
                angle_rad = np.deg2rad(angle)
                # Compute end point
                x1 = int(centerx + length * np.cos(angle_rad))
                y1 = int(centery + length * np.sin(angle_rad))
                x2 = int(centerx - length * np.cos(angle_rad))
                y2 = int(centery - length * np.sin(angle_rad))
                imgdraw = cv2.line(imgdraw, (x1, y1), (x2, y2), (255, 255, 0), 2)

        else:
            print("\n\nNO METHOD KNOWN FOR COMPUTING THE ANGLE!\n")
            print("\nPlease use `OBB` or `PCA`\nreturning 0..\n\n")
            return 0
        
        return angle_pca_rad
    

    def _fix_axis_flip(self, prev_axis, new_axis):
        """
        Fix 180° ambiguity in PCA eigenvectors.
        Ensures that the new axis points roughly in the same direction as previous one.
        """
        if prev_axis is None:
            return new_axis

        if np.dot(prev_axis, new_axis) < 0:
            return -new_axis   # flip by 180°
        return new_axis

    def _update_axis_history(self, new_axis, fragment_name):
        if new_axis is None:
            return None

        # Append to history
        self.principal_axis_history[fragment_name].append(new_axis)

        # Keep only the last max_history entries
        if len(self.principal_axis_history[fragment_name]) > self.num_samples:
            self.principal_axis_history[fragment_name].pop(0)

        # Compute average or median
        avg = np.median(self.principal_axis_history[fragment_name], axis=0)

        # Normalize
        avg = avg / np.linalg.norm(avg)

        return avg

    def smooth_polygon(self, polygon):
        pts = np.array(polygon, dtype=np.float32)

        # epsilon controls smoothness: larger → more smoothing
        eps = 0.01
        epsilon = eps * cv2.arcLength(pts, True)
        smooth = cv2.approxPolyDP(pts, epsilon, True)
        smooth = smooth.reshape(-1, 2)

        return smooth

    def recognize_and_publish(self, recognition_pub, placement_pub, group_num, iou=0.99,
                               verbosity=1, placed_pieces=[], debug=False, show_image_feed=False, 
                               use_hardcoded_alignment=True, only_g15=False):
        """
        The main loop with the 2D color recognition, reprojection and registration 
        """
        # main loop
        counter = 0
        while not rospy.is_shutdown():
            points3d_rs = get_points_from_ros()
            
            # print('-' * 40)
            detections = self.recognize_with_contraints(self.rgb_image, 
                                                        group_number=group_num,
                                                        placed_pieces=self.placed_pieces,
                                                        iou=iou)
            

            # get masks for all detections in one line of code
            masks = [detections[d]['mask'].xy[0].astype(np.int32) for d in detections]
            boxes = [detections[d]['box'].cpu().numpy().astype(int) for d in detections]
            areas = [self.calculate_area(masks[i], boxes[i]) for i in range(len(masks))]
            print("Areas of detected IDs:", areas)
            # reorder by area (largest first)
            sorted_indices = np.argsort(areas)[::-1]
            detections = {list(detections.keys())[i]: detections[list(detections.keys())[i]] for i in sorted_indices}
            print("Detections reordered by area:", [detections[d]['name'] for d in detections])

            names_list = [detections[d]['name'] for d in detections]
            if self.principal_axis_history is None: # initialize the history of principal axes
                self.principal_axis_history = {detection['name']: [] for detection in detections.values()}
            elif set(self.principal_axis_history.keys()) != set(names_list): # new object detected, start a history for it
                for name in names_list:
                    if name not in self.principal_axis_history.keys():
                        self.principal_axis_history[name] = []

            # We collect information which will be published
            points_in_3d_space = []
            fragments_ids = []
            fragments_rotation = []
            fragments_area = []
            fragments_placement_positions = []
            fragments_placement_rotations = []
            fragments_placement_side = []
            grasping_use_wide_hand = []
            detected = np.zeros((3,1)) #= 0

            counter += 1
            if debug ==True:
                vedo_spheres = []
                img2draw = self.rgb_image.copy()
           
            # stuff for drawing
            if show_image_feed == True:
                from ultralytics.utils.plotting import Colors
                yolo_colors = Colors()
                image_draw = self.rgb_image.copy()

            # here we print out what we detected 
            # (and draw if needed)
            detected_string = f"Group {group_num}:\n"
            # for class_id, detection in final_detections.items():
            for class_id, detection in detections.items():
                if detection['conf'].cpu().numpy() > 0.6:
                    # breakpoint()
                    # fragment_name = detections.names[class_id]
                    fragment_name = detection['name']
                    fragment_id = int(fragment_name[4:9])
                    conf = detection['conf'].cpu().numpy()
                    detected_string += f"  {fragment_name}: {conf:.2f}"
                    polygon = detection['mask'].xy[0]
                    polygon = self.smooth_polygon(polygon)
                    box = detection['box'].cpu().numpy().astype(int)
                    centerx, centery = self.calculate_center_of_mass(polygon)
                    rotation_angle_deg = self.calculate_rotation(polygon, method='PCA', imgdraw=image_draw, fragment_name=fragment_name) 
                    print(f"Rotation angle for {fragment_name}: {rotation_angle_deg:.2f} radians")
                    area = self.calculate_area(polygon, box)

                    ###############################
                    # prepare for publishing
                    # the stuff
                    ###############################
                    placement_side_string = self.placements_dict[f'group_{group_num}']['placement']['side']
                    if placement_side_string == 'left':
                        placement_side = -1
                    else:
                        placement_side = 1

                    # rotations and ids for publishing
                    fragments_rotation.append(rotation_angle_deg)
                    fragments_area.append(area)
                    fragments_ids.append(fragment_id)
                    fragments_placement_side.append(placement_side)

                    # we fetch here the final position where it should be placed
                    assembly_position = self.placements_dict[f'group_{group_num}'][f'{fragment_name}_intact_mesh']
                    print(f"found {fragment_name} (group {group_num}) with center in {centerx:.2f}, {centery:.2f} (pixel coordinates).")
                    if verbosity > 1:
                        print(f"\nIt should be placed (in real world coordaintes) in: {assembly_position['trans_x']}, {assembly_position['trans_y']}\n rotated by {assembly_position['ori_yaw']}")       
                    if self.use_gazebo:
                        fragment_pose = pt3d_to_pose([assembly_position['trans_x']/4000, assembly_position['trans_y']/4000, 0], use_gazebo=self.use_gazebo)
                    else:
                        fragment_pose = pt3d_to_pose([assembly_position['trans_x'], assembly_position['trans_y'], 0], use_gazebo=self.use_gazebo)

                    fragments_placement_positions.append(fragment_pose)
                    fragments_placement_rotations.append(assembly_position['ori_yaw'])

                    # grasping
                    try:
                        use_wide_hand = int(assembly_position['use_wide'])
                    except:
                        use_wide_hand = False
                    grasping_use_wide_hand.append(use_wide_hand)

                    # 3D 
                    depth = self.depth_image[centery, centerx]
                    point_in_3d_space = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [centery, centerx], depth)
                    points_in_3d_space.append((vedo.Point(point_in_3d_space), rotation_angle_deg))
                    if debug == True:
                        vedo_sphere = vedo.Sphere(point_in_3d_space, c="red", r=50).apply_transform(self.T_opencv2rviz)
                        vedo_spheres.append(vedo_sphere)

                    ################################
                    # image visualization
                    # show the recognized objects
                    ################################
                    if show_image_feed == True:
                        text = f"{fragment_name}: {conf:.2f}"
                        color = yolo_colors(class_id, bgr=True)
                        # 1. Draw bounding box
                        box = detection['box'].cpu().numpy().astype(int)
                        x1, y1, x2, y2 = box
                        cv2.rectangle(image_draw, (x1, y1), (x2, y2), color, 2)
                        # 2. Draw confidence text
                        conf = detection['conf'].cpu().numpy()
                        text = f"{fragment_name}: {conf:.2f}"
                        # Add text background for better visibility
                        (text_width, text_height), baseline = cv2.getTextSize(
                            text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(image_draw, 
                            (x1, y1 - text_height - baseline - 5),
                            (x1 + text_width, y1),
                            color, -1) # -1 = Filled rectangle
                        cv2.putText(image_draw, text, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 25), 2 )# White text
                        # Convert to integer coordinates and reshape for cv2.polylines
                        polygon = polygon.astype(np.int32).reshape((-1, 1, 2))
                        # Draw polygon
                        image_draw = cv2.polylines(image_draw, [polygon], isClosed=True, color=color, thickness=2)
                        # draw circle
                        image_draw = cv2.circle(image_draw, (centerx, centery), 2, color, 5)
                        
                    
            if show_image_feed == True:
                cv2.imshow(f'recognition', image_draw)
                print(detected_string)
                # f'detected {len(det_res[0].obb)} objects:\n\t- {detected[0]} of group 15\n\t- {detected[1]} of group 29\n\t- {detected[2]} of group 89')
                cv2.waitKey(1)


            # MISALIGNMENT CORRECTION
            # align reprojected pointcloud with realsense pointcloud
            # which is then used in the system for the successive transformations

            # STEP 1:
            # Creating the vedo pointcloud from the image (will be used later for alignment)
            pts3d = []
            colors3d = []
            for _x in range(self.rgb_image.shape[1]):
                for _y in range(self.rgb_image.shape[0]):
                    pts3d.append(rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [_y, _x], self.depth_image[_y, _x]))
                    colors3d.append(self.rgb_image[_y, _x])
            if self.use_gazebo:
                realsense_pcl = vedo.Points(points3d_rs)
            else:
                realsense_pcl = vedo.Points(points3d_rs * 1000) # meters to millimeters
            vedo_pcl = vedo.Points(pts3d)
            vedo_pcl.apply_transform(self.T_opencv2rviz)
            vedo_pcl.pointcolors = np.asarray(colors3d)
            if verbosity > 1:
                print('mean point realsense', np.mean(points3d_rs))
                print('mean point vedo_pcl', np.mean(pts3d))
            
            # STEP 2:
            #if self.use_gazebo == False:
            if use_hardcoded_alignment == False:
                # trying the ICP Alignment on-the-fly (to the RealSense point cloud)
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
                if not self.use_gazebo:
                    vd_pt3d.apply_transform(self.T_opencv2rviz)
                
                if use_hardcoded_alignment == False:
                    vd_pt3d.apply_transform(align_to_realsenseT.transformation)
                else:
                    vd_pt3d.apply_transform(self.hardcoded_alignment_matrix)

                transformed_points_in_3d_space.append(vd_pt3d)
                poses.append(pt3d_to_pose(vd_pt3d.vertices[0], rotation=rotation, use_gazebo=self.use_gazebo))


            # if self.use_gazebo:    
            self.publish_pose_array(poses, frame_id=self.optical_frame)

            if verbosity > 0 and use_hardcoded_alignment == False:
                print("\n# ALIGNMENT TO REALSENSE")
                print(align_to_realsenseT)
                print(align_to_realsenseT.transformation)
                print("\n# ALIGNMENT TO REALSENSE")

            if debug == True:
                # vedo.show(vedo_pcl, realsense_pcl, vedo_spheres, transformed_points_in_3d_space, axes=1, interactive=True).close()
                if use_hardcoded_alignment == False:
                    vedo.show(vedo_pcl.apply_transform(align_to_realsenseT.transformation), realsense_pcl, vedo_spheres, points_in_3d_space, axes=1, interactive=True).close()
                else:
                    vedo.show(vedo_pcl.apply_transform(self.hardcoded_alignment_matrix), realsense_pcl, vedo_spheres, points_in_3d_space, axes=1, interactive=True).close()
            
            # STEP 4:
            

            ####################
            # RECOGNITION
            #####################
            # Create the PoseArray and publish
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = rospy.Time.now()
            pose_array_msg.header.frame_id = self.optical_frame  # Use the appropriate frame_id
            # # Add poses to PoseArray
            pose_array_msg.poses = poses
            #pose_array_msg = self.transform_pose_array_to_world(poses)
            # Publish the PoseArray
                        # and the rotation alone for now
            rotation_array_msg = Float32MultiArray()
            rotation_array_msg.data = fragments_rotation
            
            area_array_msg = Float32MultiArray()
            area_array_msg.data = fragments_area

            ################ 
            # ID
            # we publish now also the ids
            id_array_msg = Int32MultiArray()
            id_array_msg.data = fragments_ids

            use_wide_hand_msg = Int32MultiArray()
            use_wide_hand_msg.data = grasping_use_wide_hand

            ####################
            # Publish RECOGNITION
            #####################

            recognition_msg = RecognitionData()
            recognition_msg.header.stamp = rospy.Time.now()
            recognition_msg.pose_array = pose_array_msg  # Populate PoseArray
            recognition_msg.id_array = id_array_msg  # Populate Int32MultiArray for ids
            recognition_msg.rotation_array = rotation_array_msg  # Populate Float32MultiArray for rotations
            recognition_msg.area_array = area_array_msg  # Populate Float32MultiArray for rotations
            recognition_msg.use_wide_hand = use_wide_hand_msg  # Populate Int32MultiArray for wide hand data

            recognition_pub.publish(recognition_msg)

            ####################
            # PLACEMENT
            #####################
            # and the rotation alone for now
            fragments_placement_side_msg = Int32MultiArray()
            fragments_placement_side_msg.data = fragments_placement_side
            # This is the final position (without the Z value)
            placement_position_array_msg = PoseArray()
            placement_position_array_msg.header.stamp = rospy.Time.now()
            placement_position_array_msg.header.frame_id = self.optical_frame  # Use the appropriate frame_id
            # should we put a value for Z or 0?
            placement_pose = fragments_placement_positions
            placement_position_array_msg.poses = placement_pose

            # Rotation for the final placement
            # TODO
            placement_rotation_array_msg = Float32MultiArray()
            placement_rotation_array_msg.data = fragments_placement_rotations



            ####################
            # Publish RECOGNITION
            #####################

            placement_msg = PlacementData()

            placement_msg.header.stamp = rospy.Time.now()
            placement_msg.placement_pose_array = placement_position_array_msg  # Populate PoseArray for placements
            placement_msg.placement_rotation = placement_rotation_array_msg  # Populate Float32MultiArray for placements
            placement_msg.placement_side = fragments_placement_side_msg  # Populate Int32MultiArray for sid

            placement_pub.publish(placement_msg)     
            

def pt3d_to_pose(pt3d, rotation=0, use_gazebo=False):
    """
    Creates a Pose object and fills it (checking sizes, conversion m to mm, and rotation if there is)
    """
    pose = Pose()
    if use_gazebo:
        pose.position.x = pt3d[0]
        pose.position.y = pt3d[1]
        pose.position.z = pt3d[2]
    else:
        pose.position.x = pt3d[0]/1000 
        pose.position.y = pt3d[1]/1000 
        pose.position.z = pt3d[2]/1000
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_gazebo', action='store_true')
    parser.add_argument('-g', '--group', type=str, default='29')
    args = parser.parse_args()
    
    node_name = "Sand_Recognition_with_Polygon"
    rospy.init_node(node_name)
    verbosity_level = 1 # increase value to print debug information in the recognition code

    # Initialize the publisher for PoseArray
    recognition_pub = rospy.Publisher('/recognition/recognition_data', RecognitionData, queue_size=10)
    placement_pub = rospy.Publisher('/recognition/placement_data', PlacementData, queue_size=10)

    ####################
    #    DATA DIRs     #
    ####################
    root_data_folder = '/home/repair/repair_ws/src/repair_ros_robot/repair_interface/sand_detection_models'
    polygon_recognition_folder = os.path.join(root_data_folder, 'polygon')
    model_path = os.path.join(polygon_recognition_folder, f"polygon_rec_g{args.group}.pt")
    print(f"Will use {model_path} for this experiment!")
    placements_folder = '/home/repair/repair_ws/src/repair_ros_robot/repair_interface/placements'
    placement_file = os.path.join(placements_folder, 'demo_placement.json')
    # placement_file = os.path.join(placements_folder, 'int_week_7_piece_center_placements_demo.json')

    # data_folder = rospy.get_param('data_folder', '/home/repair/repair_ws/src/repair_ros_robot/repair_interface/config/weights_mix')  # Default in case not set
    # # data_folder = rospy.get_param('data_folder', '/home/ws/src/repair_ros_robot/repair_interface/sand_detection_models')  # Default in case not set
    # model_name = rospy.get_param('model_name', 'best_mix.pt')  # Default model name
    # # placement_file = rospy.get_param('placement_file', 'int_week_placements_demo.json')  # Default file
    # placement_file = rospy.get_param('placement_file', 'int_week_6_piece_center_placements_demo.json')  # Default file
    # # placement_file = rospy.get_param('placement_file', 'int_week_placements.json')  # Default file

    # Instantiate SandRecognition with the parameters from the ROS parameter server
    recognition = SandRecognition(data_folder=root_data_folder, model_name=model_path, placement_file=placement_file, use_gazebo=args.use_gazebo)

    recognition.recognize_and_publish(recognition_pub, placement_pub, group_num=args.group, 
                                      verbosity=verbosity_level, placed_pieces=recognition.placed_pieces, 
                                      debug=False, show_image_feed=True,
                                      use_hardcoded_alignment=True, only_g15=False)

    rospy.spin()
