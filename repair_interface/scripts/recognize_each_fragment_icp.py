#!/usr/bin/env python3
 
import open3d as o3d
from copy import deepcopy,copy
import os 
import pdb 
import numpy as np 
import matplotlib.pyplot as plt 
from open3d.visualization import draw_geometries as gdraw
from open3d.utility import Vector3dVector as v3v 
from align_utils import align_face_up, align_with_icp
from parameters import *

from vision_utils import get_point_cloud_from_ros, get_point_cloud_from_real_rs
import rospy

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
    for i in range(max_label):
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

def text_3d(text, pos, direction=None, degree=0.0, font_size=16, color=(255, 0, 0)):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param pos: 3D xyz position of the text upper left corner
    :param direction: 3D normalized direction of where the text faces
    :param degree: in plane rotation of text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """
    if direction is None:
        direction = (0., 0., 1.)

    font_path = '/home/palma/.local/share/fonts/MesloLGS NF Regular.ttf'
    from PIL import Image, ImageFont, ImageDraw
    from pyquaternion import Quaternion

    font_obj = ImageFont.truetype(font_path, font_size)
    font_dim = font_obj.getsize(text)

    img = Image.new('RGB', font_dim, color=color)
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font_obj, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 100.0)

    raxis = np.cross([0.0, 0.0, 1.0], direction)
    if np.linalg.norm(raxis) < 1e-6:
        raxis = (0.0, 0.0, 1.0)
    trans = (Quaternion(axis=raxis, radians=np.arccos(direction[2])) *
             Quaternion(axis=direction, degrees=degree)).transformation_matrix
    trans[0:3, 3] = np.asarray(pos)
    pcd.transform(trans)
    return pcd

def recognize_objects(scene_pcd):

    print('#' * 70)
    print("loading database fragments")
    db_pcds = load_from_db(folder=db_folder, names_list=names_list)
       
    print("working on", scene_path)
    scene_pcd = o3d.io.read_point_cloud(scene_path)

    if debug:
        gdraw([scene_pcd])
        pdb.set_trace()

    print("processing")
    # remove far away points
    scene = filter_pcd(scene_pcd, z=z_cut)
    # extract the objects
    table, objects = extract_fragments_from_scene(scene_pcd, distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)

    print("filtering")
    voxel_pc = objects.voxel_down_sample(voxel_size=voxel_size)
    objects, ind = voxel_pc.remove_radius_outlier(nb_points=nb_points, radius=nb_radius)
    #objects.paint_uniform_color([0, 1, 0])
    table.paint_uniform_color([1, 0, 0])

    # get the table orientation 
    # not really working
    bbox = table.get_oriented_bounding_box()
    bbox_s = bbox.scale(0.8, bbox.center)
    plane_cloud = table.crop(bbox_s)
    plane_cloud.orient_normals_to_align_with_direction(np.array([bbox.center[0], bbox.center[1], 0]))
    plane_normal_vector = np.mean(np.asarray(plane_cloud.normals), axis=0)

    print("clustering")
    colored_scene, labels, noisy_clustered_objects = cluster_objects(objects)
    print(f'got {len(noisy_clustered_objects)} objects!')
    
    # remove noisy/outliers from the clustered objects 
    # and get the scene center location
    avg_points = 0
    num_objs = 0
    scene_center = np.zeros((1,3))
    clustered_objects = []
    for obj in noisy_clustered_objects:
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
        align_face_up(db_rescaled, plane_normal_vector, np.zeros((3,1)))
        rescaled_db_objs.append(db_rescaled)
        print(f"fragment {k}: processing done    ", end='\r')

    if debug:
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
            
            icp_solution = align_with_icp(source, target, voxel_size=0.001, fast=True)
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
        dist_matrix[min_pos[0], :] = np.max(dist_matrix)
        dist_matrix[:, min_pos[1]] = np.max(dist_matrix)
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
    for j in range(assignments_ids.shape[0]):
        pcd_text = text_3d(id_list[assignments_ids[j, 0]], clustered_objects[assignments_ids[j, 1]].get_min_bound(), font_size=10, color=colors[j])
        texts.append(pcd_text)
    
        bbox = clustered_objects[assignments_ids[j, 1]].get_oriented_bounding_box()
        bbox.color = colors[j]
        bboxes.append(bbox)
        solution_dict[id_list[assignments_ids[j, 0]]] = {'bbox': bbox, 'text':pcd_text, 'pcd': clustered_objects[assignments_ids[j, 1]]}

    if show_solution:
        gdraw([scene_pcd]+texts+bboxes)

    if debug and show_pairwise_solutions:
        for j in range(assignments_ids.shape[0]):
            draw_registration_result(rescaled_db_objs[assignments_ids[j, 0]], clustered_objects[assignments_ids[j, 1]], assig_tf[j], f"{names_list[j]}: registered")

    print("done")
    print("#" * 70)
    return solution_dict


if __name__ == '__main__':
    
    node_name = "save_pcd"
    rospy.init_node(node_name)

    debug = False
    use_pyrealsense = False
    print(f"debug: {debug}, using realsense: {use_pyrealsense}")
    if use_pyrealsense:
        pcd = get_point_cloud_from_real_rs(debug)
    else:
        pcd = get_point_cloud_from_ros(debug)    
    
    recognized_objects = recognize_objects(pcd)
    print("recognized:")
    for rec_obj_name in recognized_objects.keys():
        print("\n", rec_obj_name)
        print("pcd", recognized_objects[rec_obj_name]['pcd'])
        print("bbox", recognized_objects[rec_obj_name]['bbox'])
        #print("bbox", recognized_objects[rec_obj_name]['bbox'])



