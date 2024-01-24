import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os, random, pdb

def transform_mesh(m):
    cm = m.centerOfMass()
    m.shift(-cm)
    elli = vd.pcaEllipsoid(m, pvalue=0.5)

    ax1 = vd.versor(elli.axis1)
    ax2 = vd.versor(elli.axis2)
    ax3 = vd.versor(elli.axis3)

    # the transposed matrix is already the inverse
    T = np.array([ax1, ax2, ax3])

    return m.applyTransform(T, reset=True)  # <-- I had to enable reset


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

    
