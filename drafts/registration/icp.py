import os
import open3d as o3d 
import numpy as np
import visualization as visu

BASE_PATH = os.getenv('BASE_PATH', False)
VELODYNE_PATH = os.path.join(BASE_PATH, 'velodyne_points')
PCD_PATH = os.path.join(VELODYNE_PATH, 'data_pcd')
  
def test_plot():
    visu.plot('multiway_registration_111.pcd')

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(file_1, file_2, voxel_size, trans_init):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud(file_1)
    target = o3d.io.read_point_cloud(file_2)
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def prepare_novoxel_dataset(file_1, file_2):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud(file_1)
    target = o3d.io.read_point_cloud(file_2)
    return source, target

def icp_po2po(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    return reg_p2p.transformation

def icp_po2pl(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")
    reg_p2l = o3d.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.registration.TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    return reg_p2l.transformation

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    print(result)
    print("Transformation is:")
    print(result.transformation)
    return result.transformation


def start():
    file_1 = os.path.join(PCD_PATH, os.listdir(PCD_PATH)[0])
    file_2 = os.path.join(PCD_PATH, os.listdir(PCD_PATH)[1])

    threshold = 100
    trans_init = np.identity(4, dtype=float) 
    # source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(file_1, file_2, voxel_size, trans_init)
    source, target = prepare_novoxel_dataset(file_1, file_2)

    print("Initial alignment")
    evaluation = o3d.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation)

    # new_transf = execute_fast_global_registration(
    #     source_down, target_down, source_fpfh, target_fpfh, voxel_size)
    # visu.plot_reg(source, target, new_transf)

    new_transf = icp_po2po(source, target, threshold, trans_init)
    visu.plot_reg(source, target, new_transf)

    # new_transf = icp_po2pl(source_down, target_down, threshold, trans_init)
    # visu.plot_reg(source, target, new_transf)


if __name__ == "__main__":
    # test_plot()
    start()
