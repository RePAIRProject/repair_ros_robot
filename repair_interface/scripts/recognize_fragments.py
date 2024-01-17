import open3d as o3d 
import os 
import pdb 
import numpy as np 
import matplotlib.pyplot as plt 
from open3d.visualization import draw_geometries as gdraw

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
        pcds.append(o3d.io.read_point_cloud(os.path.join(folder, name)))
    return pcds 

def get_iss(pcds, debug=False):
    """
    Get the Intrinsic Shape Signature of each pcd in the pcds 
    """
    isss = []
    for pcd in pcds:
        keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd)
        isss.append(keypoints)
    return isss 

def segment_table(pcd, distance_threshold=0.005, ransac_n=5, num_iterations=1000):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
        ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud

def extract_fragments_from_scene(scene_pcd):
    print('segmenting table')
    table, rest = segment_table(scene_pcd)
    return table, rest

def cluster_objects(pcd):

    labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
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

def fpfh_feats(pcd, max_nn=30):
    r1 = 0.5
    # Estimate normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=r1, max_nn=30))
    r2 = 0.8
    # Compute FPFH feature
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=r2, max_nn=100))
    return fpfh

def normalize_point_cloud(pcd):
    centroid = np.asarray(pcd.get_center())
    pcd.translate(-centroid)
    max_val = np.max(np.abs(np.array(pcd.points)))
    pcd.scale(1/max_val, center=(0,0,0))
    return pcd

def sample_to(pcd, num_points=1000):
    # Downsample the point cloud
    downpcd = pcd.farthest_point_down_sample(num_points)
    return downpcd

def main():

    pc = 'laptop'
    num_points = 1000

    id_list = ['RPf_00123', 'RPf_00124', 'RPf_00125', 'RPf_00126']
    names_list = [f"{id_p}b.ply" for id_p in id_list]
    if pc == 'uni':
        scenes_folder = '/home/lucap/repair_robot_ws/pcds'
        db_folder = '/media/lucap/big_data/datasets/repair/group_16/raw/3D'
    elif pc == 'home':
        scenes_folder = '/home/palma/Unive/RePAIR/int_week_2/RoboticScenes/pcds'
        db_folder = '/home/palma/Unive/RePAIR/Datasets/RePAIR_dataset/group_16/raw/3D'
    elif pc == 'laptop':
        scenes_folder = '/home/palma/repair/int_week_2/RoboticScenes/pcds'
        db_folder = '/home/palma/repair/int_week_2/dataset'
    else:
        scenes_folder = ''
        db_folder = ''

    scenes = os.listdir(scenes_folder)
    for scene_name in scenes:
        print('#' * 50)
        print("working on", scene_name)
        scene_pcd = o3d.io.read_point_cloud(os.path.join(scenes_folder, scene_name))

        print("loading database fragments")
        db_pcds = load_from_db(folder=db_folder, names_list=names_list)

        print('working on the scene')
        table, objects = extract_fragments_from_scene(scene_pcd)

        print("filtering")
        voxel_pc = objects.voxel_down_sample(voxel_size=0.001)
        objects, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)
        #objects.paint_uniform_color([0, 1, 0])
        table.paint_uniform_color([1, 0, 0])
        
        print("clustering")
        colored_scene, labels, clustered_objects = cluster_objects(objects)
        print(f'got {len(clustered_objects)} objects!')

        gdraw(clustered_objects, window_name='clustered')

        # debug 
        print('resampling clustered objects')
        resampled_objs = []
        resampled_objs_feats = []
        for j, cl_obj in enumerate(clustered_objects):
            if len(cl_obj.points) > num_points:
                cl_obj = normalize_point_cloud(cl_obj)
                cl_obj = sample_to(cl_obj, num_points)
                resampled_objs.append(cl_obj)
                fpfh_cl_obj = fpfh_feats(cl_obj)
                resampled_objs_feats.append(fpfh_cl_obj.data)

        print(f"left with {len(resampled_objs)} objects with more than {num_points} points after resampling")
        gdraw(resampled_objs, window_name=f'resampled ({len(resampled_objs)} objects)')
        
        print('resampling db objects')
        resampled_db_objs = []
        resampled_db_objs_feats = []
        for k, db_obj in enumerate(db_pcds):
            db_obj = normalize_point_cloud(db_obj)
            db_obj = sample_to(db_obj, num_points)
            resampled_db_objs.append(db_obj)
            fpfh_db_obj = fpfh_feats(db_obj)
            resampled_db_objs_feats.append(fpfh_db_obj.data)
        
        gdraw(resampled_db_objs, window_name='after down sampling')

        print("distances")
        dist_matrix = np.zeros((len(resampled_objs), len(db_pcds)))
        # iss_cl_objs = get_iss(clustered_objects)
        for j, cl_obj in enumerate(resampled_objs):
            for k, db_obj in enumerate(db_pcds):
                # distance
                dist = np.linalg.norm(resampled_objs_feats[j] - resampled_db_objs_feats[k])
                dist_matrix[j, k] = dist #np.linalg.norm(np.array(ico.points), np.array(ido.points))
        
        print("### DISTANCE MATRIX ###")
        print(dist_matrix)

        print('assigning id')
        # greedy 
        # psuedo code
        assignments = {}
        while len(assignments.keys()) < np.minimum(len(db_pcds), len(resampled_objs)):
            min_pos = np.unravel_index(np.argmin(dist_matrix), (len(resampled_objs), len(db_pcds)))
            assignments[id_list[min_pos[1]]] = resampled_objs[min_pos[0]]
            # remove possible clones
            dist_matrix[min_pos[0], :] = np.max(dist_matrix)
            dist_matrix[:, min_pos[1]] = np.max(dist_matrix)
            print(f"assigned {id_list[min_pos[1]]} with the resampled object number {min_pos[0]}")

        print("FINAL ASSIGNMENT")
        print(assignments)
        pdb.set_trace()

        o3d.visualization.draw_geometries([table, clustered_objects], window_name=f"{scene_name}")
        


if __name__ == '__main__':
    main()