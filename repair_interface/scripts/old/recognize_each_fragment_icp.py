#!/usr/bin/env python3
 
import open3d as o3d
from copy import deepcopy,copy
import os 
import pdb 
import numpy as np 
import matplotlib.pyplot as plt 
from open3d.visualization import draw_geometries as gdraw
from open3d.utility import Vector3dVector as v3v 
from align_utils import recognize_objects
from parameters import *

from vision_utils import get_point_cloud_from_ros, get_point_cloud_from_real_rs
import rospy
from vision_utils import prepare_scene, get_point_cloud_from_ros, get_point_cloud_from_real_rs


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

    object_cloud, table = prepare_scene(pcd)

    recognized_objects = recognize_objects(object_cloud)
    print("recognized:")
    for rec_obj_name in recognized_objects.keys():
        print("\n", rec_obj_name)
        print("pcd", recognized_objects[rec_obj_name]['pcd'])
        print("bbox", recognized_objects[rec_obj_name]['bbox'])
        #print("bbox", recognized_objects[rec_obj_name]['bbox'])



