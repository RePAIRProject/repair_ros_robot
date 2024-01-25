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
import pyrealsense2 as rs
import matplotlib.pyplot as plt


def get_transform(parent_frame='base_link', child_frame='camera_depth_frame'):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #rate.sleep()
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
    # rospy.sleep(1)

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

def get_point_cloud_from_real_rs(debug=False):
    # Segmentation of Point Cloud
    # xyz = np.asarray(pc)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, rs.format.z16, 30) # 30
    config.enable_stream(rs.stream.color, rs.format.bgr8, 30) # 30

    # Start streaming
    pipeline.start(config)

    # Get stream profile and camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    # Processing blocks
    pc = rs.pointcloud()
    decimate = rs.decimation_filter()
    # decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
    colorizer = rs.colorizer()

    ### Warm-up time discard first 9 frames
    for i in range(10):
        frames = pipeline.wait_for_frames()

    frames = pipeline.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    depth_frame = decimate.process(depth_frame)

    # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(
        depth_frame.profile).get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())

    mapped_frame, color_source = depth_frame, depth_colormap

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(verts)
    print ('Received point cloud')

    if debug:
        o3d.visualization.draw_geometries([pcd])

    return pcd

def segment_table(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.015,
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
    # rospy.sleep(1)
    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1.0))
        return output_pose_stamped.pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def get_pose_from_transform(T):
    quat = pyrot.quaternion_xyzw_from_wxyz(pyrot.quaternion_from_matrix(T[:3, :3]))
    pos = T[:3, 3]
    return np.concatenate((pos, quat))

def get_max_cluster(pcd, debug=True):
    labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))

    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0

    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    if debug:
        o3d.visualization.draw_geometries([pcd])

    # get total amount of points for each cluster label
    u, indices = np.unique(labels, return_inverse=True)
    # get the label id with the highest points
    x = u[np.argmax(np.bincount(indices))]
    #
    cluster_points = np.asarray(pcd.points)[labels == x]

    clustered_cloud = o3d.geometry.PointCloud()
    clustered_cloud.points = o3d.utility.Vector3dVector(cluster_points)

    if debug:
        o3d.visualization.draw_geometries([clustered_cloud])

    return clustered_cloud

def get_number_of_frescos(debug=False, use_pyrealsense=False):

    print('Starting Point Cloud Processing')
    if use_pyrealsense:
        pcd = get_point_cloud_from_real_rs(debug)
    else:
        pcd = get_point_cloud_from_ros(debug)

    # == Transform pointcloud to table frame
    tf_camera_to_world = get_transform(parent_frame="working_surface_link", child_frame="camera_depth_optical_frame")
    tran = np.array([tf_camera_to_world.transform.translation.x, tf_camera_to_world.transform.translation.y, tf_camera_to_world.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_camera_to_world.transform.rotation.w,
                                                                    tf_camera_to_world.transform.rotation.x,
                                                                    tf_camera_to_world.transform.rotation.y,
                                                                    tf_camera_to_world.transform.rotation.z]))
    
    pcd.rotate(rot, center=(0, 0, 0)).translate(tran)
    #o3d.visualization.draw_geometries([pcd], window_name="PCD Transformed table")

    # == Remove points above a certain height
    points = np.asarray(pcd.points)
    pcd = pcd.select_by_index(np.where(points[:, 2] < 0.08)[0])
    #o3d.visualization.draw_geometries([pcd], window_name="PCD Filtered")

    # == Transform back to camera frame
    tf_world_to_camera = get_transform(parent_frame="camera_depth_optical_frame", child_frame="working_surface_link")
    tran = np.array([tf_world_to_camera.transform.translation.x, tf_world_to_camera.transform.translation.y, tf_world_to_camera.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_world_to_camera.transform.rotation.w,
                                                                    tf_world_to_camera.transform.rotation.x,
                                                                    tf_world_to_camera.transform.rotation.y,
                                                                    tf_world_to_camera.transform.rotation.z]))
    pcd.rotate(rot, center=(0, 0, 0)).translate(tran)   

    #rospy.init_node('listener', anonymous=True)
    print ('Detecting Number of Frescos')

    table_cloud, object_cloud = segment_table(pcd)

    voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

    object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)

    labels = np.array(object_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))

    labels_size = np.size(np.asarray(labels))
    if labels_size == 0:
        n_objects = 0
    else:
        max_label = labels.max()
        #print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors = np.zeros((labels.shape[0], 3))
        colors[:, 0] = 1.
        colors[labels < 0] = 0

        n_objects = 0
        for label in np.unique(labels):
            idxs = (labels == label)
            if labels[idxs].shape[0] > 200:
                colors[idxs, 0] = 0
                colors[idxs, 1] = 1.
                n_objects += 1
   
    return n_objects, pcd, table_cloud, object_cloud

def check_frescos_left(debug, use_pyrealsense):

    if use_pyrealsense:
        pcd = get_point_cloud_from_real_rs(debug)
    else:
        pcd = get_point_cloud_from_ros(debug)


    # == Transform pointcloud to table frame
    tf_camera_to_world = get_transform(parent_frame="working_surface_link", child_frame="camera_depth_optical_frame")
    tran = np.array([tf_camera_to_world.transform.translation.x, tf_camera_to_world.transform.translation.y, tf_camera_to_world.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_camera_to_world.transform.rotation.w,
                                                                    tf_camera_to_world.transform.rotation.x,
                                                                    tf_camera_to_world.transform.rotation.y,
                                                                    tf_camera_to_world.transform.rotation.z]))
    
    pcd.rotate(rot, center=(0, 0, 0)).translate(tran)
    # == Remove points above a certain height
    points = np.asarray(pcd.points)
    pcd = pcd.select_by_index(np.where(points[:, 2] < 0.08)[0])

    # == Transform back to camera frame
    tf_world_to_camera = get_transform(parent_frame="camera_depth_optical_frame", child_frame="working_surface_link")
    tran = np.array([tf_world_to_camera.transform.translation.x, tf_world_to_camera.transform.translation.y, tf_world_to_camera.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_world_to_camera.transform.rotation.w,
                                                                    tf_world_to_camera.transform.rotation.x,
                                                                    tf_world_to_camera.transform.rotation.y,
                                                                    tf_world_to_camera.transform.rotation.z]))
    pcd.rotate(rot, center=(0, 0, 0)).translate(tran)   

    table_cloud, object_cloud = segment_table(pcd)

    voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

    object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)

    labels = np.array(object_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))

    labels_size = np.size(np.asarray(labels))
    if labels_size == 0:
        n_objects = 0
    else:
        max_label = labels.max()
        #print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors = np.zeros((labels.shape[0], 3))
        colors[:, 0] = 1.
        colors[labels < 0] = 0

        n_objects = 0
        for label in np.unique(labels):
            idxs = (labels == label)
            if labels[idxs].shape[0] > 200:
                colors[idxs, 0] = 0
                colors[idxs, 1] = 1.
                n_objects += 1

        if debug:
            object_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
            table_cloud.paint_uniform_color([1., 0., 0.])

            o3d.visualization.draw_geometries([object_cloud, table_cloud])
    
    return n_objects, object_cloud, table_cloud

def prepare_scene(pcd, debug=False):
    """
    It takes the point cloud, clean by removing outliers,
    then segment the table, and returns the table and 
    a cleaned version of the objects on the table
    """
    # == Transform pointcloud to table frame
    tf_camera_to_world = get_transform(parent_frame="working_surface_link", child_frame="camera_depth_optical_frame")
    tran = np.array([tf_camera_to_world.transform.translation.x, tf_camera_to_world.transform.translation.y, tf_camera_to_world.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_camera_to_world.transform.rotation.w,
                                                                    tf_camera_to_world.transform.rotation.x,
                                                                    tf_camera_to_world.transform.rotation.y,
                                                                    tf_camera_to_world.transform.rotation.z]))

    pcd.rotate(rot, center=(0, 0, 0)).translate(tran)

    # == Remove points above & below a certain height
    points = np.asarray(pcd.points)
    # pcd = pcd.select_by_index(np.where(points[:, 2] < 0.08)[0])
    # points = np.asarray(pcd.points)
    
    object_cloud = pcd.select_by_index(np.where((points[:, 2] < 0.08) & (points[:, 2] > 0.0001))[0])
    table_cloud = pcd.select_by_index( np.where(((points[:, 2] < 0.0001) & (points[:, 2] > -0.05)))[0])
    # pcd = pcd.select_by_index(np.where(points[:, 2] > -0.04)[0])
   
    if debug:
        object_cloud.paint_uniform_color([1, 1, 0])
        table_cloud.paint_uniform_color([0, 0, 1])
        o3d.visualization.draw_geometries([table_cloud, object_cloud])
        # o3d.visualization.draw_geometries([pcd], window_name="PCD Filtered")

    # == Transform back to camera frame
    tf_world_to_camera = get_transform(parent_frame="camera_depth_optical_frame", child_frame="working_surface_link")
    # tf_world_to_camera = get_transform(parent_frame="world", child_frame="working_surface_link")
    tran = np.array([tf_world_to_camera.transform.translation.x, tf_world_to_camera.transform.translation.y, tf_world_to_camera.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_world_to_camera.transform.rotation.w,
                                                                    tf_world_to_camera.transform.rotation.x,
                                                                    tf_world_to_camera.transform.rotation.y,                                              tf_world_to_camera.transform.rotation.z]))
    # pcd.rotate(rot, center=(0, 0, 0)).translate(tran)
    object_cloud.rotate(rot, center=(0, 0, 0)).translate(tran)
    table_cloud.rotate(rot, center=(0, 0, 0)).translate(tran)
    
    return object_cloud, table_cloud