import open3d as o3d
import numpy as np
from copy import deepcopy,copy
import matplotlib.pyplot as plt
import os, random, pdb
from open3d.utility import Vector3dVector as v3v 
from open3d.visualization import draw_geometries as gdraw
from parameters import *

def load_from_db(folder, names_list):
    """
    Loads the point clouds: either the whole `folder` (if `names_list` is empty)
    or the ones contained in the `names_list`
    """
    pcds = []
    if len(names_list)==0:
        names_list = os.listdir(folder)
        names_list.sort() 
    for name in names_list:
        pcd = o3d.io.read_point_cloud(os.path.join(folder, name))
        pcds.append(pcd)
    return pcds 

def segment_table(pcd, distance_threshold=0.008, ransac_n=5, num_iterations=1000):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
        ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud

def extract_fragments_from_scene(scene_pcd, distance_threshold=0.008, ransac_n=5, num_iterations=1000):
    print('segmenting table')
    table, rest = segment_table(scene_pcd, distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    return table, rest

def cluster_objects(pcd, eps=0.01, min_points=100, print_progress=True):
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=print_progress))
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    objects = []
    for i in range(max_label+1):
        obj = o3d.geometry.PointCloud()
        obj.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[labels == i])
        obj.colors =  o3d.utility.Vector3dVector(colors[labels == i][:, :3])
        objects.append(obj)

    return pcd, labels, objects


def sample_to(pcd, num_points=1000):
    # Downsample the point cloud
    downpcd = pcd.farthest_point_down_sample(num_points)
    return downpcd

def draw_registration_result(source, target, transformation, window_name='icp results'):
    source_temp = deepcopy(source)
    target_temp = deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name=window_name)

def filter_pcd(pcd, z=1):
    cropped_pcd = o3d.geometry.PointCloud()
    xyz = np.asarray(pcd.points)
    idx = np.where(xyz[:, 2] < z)     # Prune point cloud to 0.8 meters from camera in z direction
    xyz = xyz[idx]
    cropped_pcd.points = v3v(xyz)
    return cropped_pcd

def est_bbox_distance(source, target, transformation):

    source_temp = deepcopy(source)
    target_temp = deepcopy(target)
    source_temp.transform(transformation)
    bbox_source = source_temp.get_oriented_bounding_box()
    bbox_target = target_temp.get_oriented_bounding_box()

    center_dist = np.linalg.norm(bbox_source.center - bbox_target.center) 
    center_weight = 1
    extent_dist = np.linalg.norm(bbox_source.extent - bbox_target.extent)
    extent_weight = 1
    return center_dist * center_weight + extent_dist * extent_weight

# def text_3d(text, pos, direction=None, degree=0.0, font_size=16, color=(255, 0, 0)):
#     """
#     Generate a 3D text point cloud used for visualization.
#     :param text: content of the text
#     :param pos: 3D xyz position of the text upper left corner
#     :param direction: 3D normalized direction of where the text faces
#     :param degree: in plane rotation of text
#     :param font: Name of the font - change it according to your system
#     :param font_size: size of the font
#     :return: o3d.geoemtry.PointCloud object
#     """
#     if direction is None:
#         direction = (0., 0., 1.)

#     font_path = '/home/palma/.local/share/fonts/MesloLGS NF Regular.ttf'
#     from PIL import Image, ImageFont, ImageDraw
#     from pyquaternion import Quaternion

#     font_obj = ImageFont.truetype(font_path, font_size)
#     font_dim = font_obj.getsize(text)

#     img = Image.new('RGB', font_dim, color=color)
#     draw = ImageDraw.Draw(img)
#     draw.text((0, 0), text, font=font_obj, fill=(0, 0, 0))
#     img = np.asarray(img)
#     img_mask = img[:, :, 0] < 128
#     indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

#     pcd = o3d.geometry.PointCloud()
#     pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
#     pcd.points = o3d.utility.Vector3dVector(indices / 100.0)

#     raxis = np.cross([0.0, 0.0, 1.0], direction)
#     if np.linalg.norm(raxis) < 1e-6:
#         raxis = (0.0, 0.0, 1.0)
#     trans = (Quaternion(axis=raxis, radians=np.arccos(direction[2])) *
#              Quaternion(axis=direction, degrees=degree)).transformation_matrix
#     trans[0:3, 3] = np.asarray(pos)
#     pcd.transform(trans)
#     return pcd

def recognize_objects(objects):

    print('#' * 70)
    print("loading database fragments")
    db_pcds = load_from_db(folder=db_folder, names_list=names_list)
       
    # print("working on", scene_path)
    # scene_pcd = o3d.io.read_point_cloud(scene_path)
    # gdraw([objects])
    # pdb.set_trace()
    # if debug:
    #     gdraw([scene_pcd])
    #     pdb.set_trace()

    # print("processing")
    # remove far away points
    # scene = filter_pcd(scene_pcd, z=z_cut)
    # extract the objects
    # table, objects = extract_fragments_from_scene(scene_pcd, distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)

    print("filtering")
    # scene_pcd.estimate_normals()
    voxel_pc = objects.voxel_down_sample(voxel_size=voxel_size)
    objects, ind = voxel_pc.remove_radius_outlier(nb_points=nb_points, radius=nb_radius)
    #objects.paint_uniform_color([0, 1, 0])
    # table.paint_uniform_color([1, 0, 0])

    # # get the table orientation 
    # # not really working
    # bbox = table.get_oriented_bounding_box()
    # bbox_s = bbox.scale(0.8, bbox.center)
    # plane_cloud = table.crop(bbox_s)
    # plane_cloud.estimate_normals()
    # plane_cloud.orient_normals_to_align_with_direction(np.array([bbox.center[0], bbox.center[1], 0]))
    # plane_normal_vector = np.mean(np.asarray(plane_cloud.normals), axis=0)

    print("clustering")
    #pdb.set_trace()
    colored_scene, labels, noisy_clustered_objects = cluster_objects(objects)
    print(f'got {len(noisy_clustered_objects)} objects!')
    
    # remove noisy/outliers from the clustered objects 
    # and get the scene center location
    avg_points = 0
    num_objs = 0
    scene_center = np.zeros((1,3))
    clustered_objects = []
    for obj in noisy_clustered_objects:
        print(len(obj.points))
        if len(obj.points) > min_points_detection:
            avg_points += len(obj.points)
            num_objs += 1
            scene_center += obj.get_center()
            if debug:
                print(obj.get_center())
            clustered_objects.append(obj)

    avg_points /= num_objs
    avg_points = np.round(avg_points).astype(int)
    scene_center /= num_objs
    print('average points:', avg_points)
    print('scene center:', scene_center)
    print(f'after filtering: {len(clustered_objects)} filtered objects!')

    # gdraw(clustered_objects)
    # pdb.set_trace()

    # recreate a clean scene by merging pointcloud of the clustered objects
    clean_scene = o3d.geometry.PointCloud()
    objects_data = v3v(clustered_objects[0].points)
    for j in range(1, len(clustered_objects)):
        objects_data.extend(v3v(clustered_objects[j].points))
    clean_scene.points = objects_data

    # rescale the models in the database so that we have simmilar points to 
    # the one in the scene
    print("#" * 70)
    print("Processing database fragment before alignment..")
    rescaled_db_objs = []        
    for k, db_pcd in enumerate(db_pcds):
        print(f"fragment {k}: processing started", end='\r')
        db_pcd.translate(-db_pcd.get_center())
        db_rescaled = deepcopy(db_pcd)
        db_rescaled = db_rescaled.scale(scale=scaling_factor, center=db_rescaled.get_center())
        db_rescaled = sample_to(db_rescaled, avg_points)
        translation = db_rescaled.get_center() - (scene_center)
        db_rescaled.translate(-np.transpose(translation))
        # align_face_up(db_rescaled, plane_normal_vector, np.zeros((3,1)))
        rescaled_db_objs.append(db_rescaled)
        print(f"fragment {k}: processing done    ", end='\r')

    if debug is True:
        print('showing all together')  
        gdraw(rescaled_db_objs+clustered_objects, window_name='all together') 
        pdb.set_trace()

    print("#" * 70)
    print("Now using alignment to recognize the fragments.. (this will take some time!)")
    # here check each fragment detected against each fragment in the db
    tfs = []
    matching_copy_clustered_objects = []
    results = np.zeros((len(rescaled_db_objs), len(clustered_objects)))
    for j, rescaled_obj in enumerate(rescaled_db_objs):
        for k, clustered_obj in enumerate(clustered_objects):

            print(f"{names_list[j]} against clustered object {k}: processing..", end='\r')
            copy_clustered_obj = deepcopy(clustered_obj)

            copy_clustered_obj.translate(-copy_clustered_obj.get_center())
            rescaled_obj.translate(-rescaled_obj.get_center())

            if debug and save_detections:
                o3d.io.write_point_cloud(f'clustered/obj_{k}.ply', copy_clustered_obj)
            
            source = clustered_obj
            target = rescaled_obj
            
            print("icp", source, target)
            icp_solution = align_with_icp(source, target, voxel_size=icp_voxel_size, fast=False)
            solution = icp_solution.transformation
            # print(icp_solution)

            bbox_dist = est_bbox_distance(source, target, solution)
            results[j,k] = bbox_dist
            print(f"{names_list[j]} against clustered object {k}: dist={bbox_dist:.03f}   ", end='\r')
            tfs.append(solution)
            print()


    dist_matrix = results
    print("\nDIST MATRIX")
    print(dist_matrix)

    print("#" * 70)
    print('assigning id')
    # greedy 
    assignments = {}
    assignments_ids = np.zeros((len(db_pcds), 2), dtype=int)
    assig_tf = []
    while len(assignments.keys()) < np.minimum(len(db_pcds), len(clustered_objects)):
        argmin_idx = np.argmin(dist_matrix)
        min_pos = np.unravel_index(argmin_idx, (len(db_pcds), len(clustered_objects)))
        assignments[id_list[min_pos[0]]] = clustered_objects[min_pos[1]]
        # remove possible clones
        dist_matrix[min_pos[0], :] = np.max(dist_matrix) + 1
        dist_matrix[:, min_pos[1]] = np.max(dist_matrix) + 1
        print(f"min_pos: {min_pos} // assigned {id_list[min_pos[0]]} to the detected and clustered object number {min_pos[1]}")
        #gdraw([rescaled_db_objs[min_pos[0]], clustered_objects[min_pos[1]]])
        assignments_ids[len(assignments.keys())-1, 0] = min_pos[0]
        assignments_ids[len(assignments.keys())-1, 1] = min_pos[1]
        assig_tf.append(tfs[argmin_idx])
        #pdb.set_trace()

    print("FINAL ASSIGNMENT")
    print(assignments)
    print("#" * 70)
    for k in range(assignments_ids.shape[0]):
        print(f"{id_list[k]} : {clustered_objects[assignments_ids[k,1]].get_oriented_bounding_box()}")
    print("#" * 70)

    solution_dict = {}
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 127, 127)]
    texts = []
    bboxes = []
    print(assignments_ids.shape[0])
    for j in range(assignments_ids.shape[0]):
        # pcd_text = text_3d(id_list[assignments_ids[j, 0]], clustered_objects[assignments_ids[j, 1]].get_min_bound(), font_size=10, color=colors[j])
        # texts.append(pcd_text)
    
        bbox = clustered_objects[assignments_ids[j, 1]].get_oriented_bounding_box()
        bbox.color = colors[j]
        bboxes.append(bbox)
        solution_dict[id_list[assignments_ids[j, 0]]] = {'bbox': bbox, 'pcd': clustered_objects[assignments_ids[j, 1]]} #@'text':pcd_text, 
                                                                                                             

    if show_solution:
        gdraw([objects]+bboxes) # texts

    if debug and show_pairwise_solutions:
        for j in range(assignments_ids.shape[0]):
            draw_registration_result(rescaled_db_objs[assignments_ids[j, 0]], clustered_objects[assignments_ids[j, 1]], assig_tf[j], f"{names_list[j]}: registered")

    print("done")
    print("#" * 70)
    return solution_dict

def get_R_x(n):
    R_x = np.zeros((3, 3))
    theta = np.arcsin(n[1] / np.linalg.norm(np.asarray([n[1], n[2]])))
    # print(f"rotating {np.rad2deg(theta)} on yz plane")
    R_x[0, 0] = 1
    R_x[1, 1] = np.cos(theta)
    R_x[1, 2] = - np.sin(theta)
    R_x[2, 2] = np.cos(theta)
    R_x[2, 1] = np.sin(theta)
    return R_x


def get_R_y(n):
    R_y = np.zeros((3, 3))
    theta = np.arcsin(n[0] / np.linalg.norm(np.asarray([n[0], n[2]])))
    # print(f"rotating {np.rad2deg(theta)} degrees on xz plane")
    R_y[1, 1] = 1
    R_y[0, 0] = np.cos(theta)
    R_y[0, 2] = np.sin(theta)
    R_y[2, 2] = np.cos(theta)
    R_y[2, 0] = - 1 * np.sin(theta)
    return R_y


def align_face_up(mesh, n, mean_point):
    """
    We want the piece with the flat surface (from normal n)
    looking "up" in the z-axis and in the origin (0, 0, 0).
    So rotation in z is not needed (we don't have yet a solution)
    We just want on xz and yz plane so set it flat
    """
    # move to origin
    mesh.translate(-mean_point)

    # first on yz plane (around x-axis)
    Rx = get_R_x(n)
    mesh.transform(matrix_3x3_to_4x4(Rx))

    # then on xz plane (around y-axis)
    Ry = get_R_y(n)
    mesh.transform(matrix_3x3_to_4x4(Ry))

    # no return, the mesh is transformed
    #print(Rx, Ry, mean_point)

def matrix_3x3_to_4x4(m):

    T = np.zeros((4, 4))
    T[:3, :3] = m
    T[3, 3] = 1
    return T

def create_pos(x, y, noise=0.25):

    pos = np.zeros((3,1))
    pos[0] = x + random.uniform(-noise, noise)
    pos[1] = y + random.uniform(-noise, noise)
    pos[2] = 0 # we don't move on z axis
    return pos

def shake_it(frag):
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(
        [0,
         0,
         np.random.uniform(0, 360)])
    frag.rotate(R=R, center=np.mean(np.array(frag.points), axis=0))
    frag.translate([np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.05, 0.05)])


def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_pcds(source, target, voxel_size=0.001):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh



def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
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

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    # print(":: Apply fast global registration with distance threshold %.3f" \
    #         % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, init_tf=np.eye(4)):
    distance_threshold = voxel_size * 0.2
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, init_tf,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def align_with_icp(source, target, voxel_size=0.001, fast=False):
    
    source_down, target_down, source_fpfh, target_fpfh = prepare_pcds(source, target, voxel_size=voxel_size)
    if fast == False:
        initial_global_transf = execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size)
    else: # fast == True
        initial_global_transf = execute_fast_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size)
    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size, init_tf=initial_global_transf.transformation)
    return result_icp

    
