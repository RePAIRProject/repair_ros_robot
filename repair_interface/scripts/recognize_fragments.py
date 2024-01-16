import open3d as o3d 
import os 
import pdb 
import numpy as np 
import matplotlib.pyplot as plt 

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

def fpfh_feats(pcd):
    # Estimate normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Compute FPFH feature
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=100))

    return fpfh

def normalize_point_cloud(pcd):
    # Compute the centroid of the point cloud
    centroid = np.asarray(pcd.get_center())

    # Translate the point cloud to the origin
    pcd.translate(-centroid, relative=False)

    # # Compute the covariance matrix
    # covariance_matrix = np.cov(np.asarray(pcd.points))

    # # Compute the eigenvalues and eigenvectors of the covariance matrix
    # eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

    # # Sort the eigenvalues and eigenvectors
    # sorted_indices = np.argsort(eigenvalues)[::-1]
    # sorted_eigenvalues = eigenvalues[sorted_indices]
    # sorted_eigenvectors = eigenvectors[:, sorted_indices]

    # # Compute the rotation matrix
    # rotation_matrix = np.dot(sorted_eigenvectors, np.diag(np.sqrt(1 / sorted_eigenvalues)))

    # # Rotate the point cloud
    # pcd.rotate(rotation_matrix, center=(0, 0, 0))

    # Scale the point cloud to the unit sphere
    max_distance = np.max(np.asarray(pcd.compute_nearest_neighbor_distance()))
    pcd.scale(1 / max_distance, center=(0, 0, 0))

    return pcd

def sample_to(pcd, num_points=1000):
    # Downsample the point cloud
    downpcd = pcd.farthest_point_down_sample(num_points)
    return downpcd

def main():

    scenes_folder = '/home/lucap/repair_robot_ws/pcds'
    scenes = os.listdir(scenes_folder)
    for scene_name in scenes:
        print('#' * 50)
        print("working on", scene_name)
        scene_pcd = o3d.io.read_point_cloud(os.path.join(scenes_folder, scene_name))

        print("loading database fragments")
        db_pcds = load_from_db(folder='/media/lucap/big_data/datasets/repair/group_16/raw/3D', \
            names_list=['RPf_00123b.ply', 'RPf_00124b.ply', 'RPf_00125b.ply', 'RPf_00126b.ply'])
        # print("extracting features")
        # db_feats = get_iss(db_pcds)

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

        print("distances")
        dist_matrix = np.zeros((len(clustered_objects), len(db_pcds)))
        # iss_cl_objs = get_iss(clustered_objects)
        for j, cl_obj in enumerate(clustered_objects):
            for k, db_obj in enumerate(db_pcds):
                # normalize pcd
                cl_obj = normalize_point_cloud(cl_obj)
                db_obj = normalize_point_cloud(db_obj)
                # down sample pcd
                cl_obj = sample_to(cl_obj)
                db_obj = sample_to(db_obj)
                # compute descriptors
                fpfh_cl_obj = fpfh_feats(cl_obj)
                fpfh_db_obj = fpfh_feats(db_obj)
                # distance
                pdb.set_trace()
                dist = np.linalg.norm(fpfh_cl_obj.data - fpfh_db_obj.data)
                dist_matrix[j, k] = dist #np.linalg.norm(np.array(ico.points), np.array(ido.points))
        

        print(dist_matrix)
        pdb.set_trace()

        o3d.visualization.draw_geometries([table, clustered_objects], window_name=f"{scene_name}")
        


if __name__ == '__main__':
    main()