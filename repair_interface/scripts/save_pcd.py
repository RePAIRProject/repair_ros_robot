#!/usr/bin/env python3

from vision_utils import get_point_cloud_from_ros, get_point_cloud_from_real_rs
import os 
import open3d as o3d
import rospy

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
    print("got pcd")
    folder = '/home/lucap/repair_robot_ws/pcds'
    pcd_names = os.listdir(folder)
    if len(pcd_names) == 0:
        last_id = 0
    else:
        pcd_names.sort()
        last_id = int(pcd_names[-1][-5:-4]) 
    print(f'last scene was {last_id}')
    new_id = last_id + 1
    pcd_path = os.path.join(folder, f"scene_{new_id:05d}.ply")
    o3d.io.write_point_cloud(pcd_path, pcd)
    print("successfully finished")
