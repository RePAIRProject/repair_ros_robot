import open3d as o3d 
import os 
import pdb 

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

def segment_table(pcd, distance_threshold=0.015, ransac_n=5, num_iterations=1000):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
        ransac_n=ransac_n, num_iterations=num_iterations)
    [a, b, c, d] = plane_model
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud

def extract_fragments_from_scene(scene_pcd):
    print('segmenting table')
    table, rest = segment_table(scene_pcd)
    table.paint_uniform_color([1, 0, 0])
    rest.paint_uniform_color([0, 0, 1])
    o3d.visualization.draw_geometries([table, rest])
    pdb.set_trace()
    return 1 

def main():

    scene_pcd = o3d.io.read_point_cloud('/home/lucap/repair_robot_ws/pcds/scene_00001.ply')

    print("loading database fragments")
    db_pcds = load_from_db(folder='/media/lucap/big_data/datasets/repair/group_16/raw/3D', \
        names_list=['RPf_00123b.ply', 'RPf_00124b.ply', 'RPf_00125b.ply', 'RPf_00126b.ply'])
    print("extracting features")
    db_feats = get_iss(db_pcds, debug=True)

    print('working on the scene')
    fragments_scene = extract_fragments_from_scene(scene_pcd)

if __name__ == '__main__':
    main()