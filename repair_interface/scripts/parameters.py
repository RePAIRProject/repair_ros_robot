# segment plane
distance_threshold=0.008
ransac_n=5
num_iterations=1000
# dbscan 
eps=0.01
min_points=100
# sampling
num_points=1000
# filtering 
voxel_size=0.001
nb_points=40
nb_radius=0.03
min_points_detection=2000
# filter points further away
z_cut=0.3
# scaling to compensate
scaling_factor=1
# paths and db models
pc = 'laptop'
model_pts = 50000
id_list = ['RPf_00123', 'RPf_00124', 'RPf_00125', 'RPf_00126']
version = ''
algorithm = 'icp'
names_list = [f"{id_p}_{model_pts}.ply" for id_p in id_list]
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
# scenes ()
scenes = ['/home/palma/repair/int_week_2/aligned/monday/pcl_frame_00010.ply']
#, '/home/palma/repair/int_week_2/aligned/monday/pcl_frame_00400.ply', '/home/palma/repair/int_week_2/aligned/monday/pcl_frame_00010.ply']

# debug
debug=False
save_detections=False
show_solution=True
show_pairwise_solutions=False

# print stuff
verbosity=1