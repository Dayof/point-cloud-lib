from pathlib import Path
from tqdm import tqdm
import open3d as o3d 
import numpy as np
import click
import os

BASE_DIR = os.getenv('BASE_DIR', '/home/dayoff/data/2011_09_26_drive_0001')
DATA_SYNC_DIR = os.path.join(BASE_DIR, '2011_09_26_drive_0001_sync')
PC_PATH = os.path.join(DATA_SYNC_DIR, 'velodyne_points', 'data')

BASE_DATA = Path().resolve().parents[0]
DATA_PATH = BASE_DATA / 'data'
GLOBAL_PLY_PATH = DATA_PATH / 'icp_global'


def pairwise_registration(source, target, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            print(f'source: {source_id} - target: {target_id}')
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id], max_correspondence_distance_coarse,
                max_correspondence_distance_fine)
            print("Build o3d.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=True))
    return pose_graph


def idx2filename(idx, format):
    return f'{idx}'.zfill(10) + f'.{format}'


def load_point_clouds(n_files):
    pcds = []
    for i in range(n_files):
        filename = idx2filename(i, 'bin')
        filepath = os.path.join(PC_PATH, filename)
        print(filepath)
        data = np.fromfile(filepath, dtype=np.float32, count=-1).reshape([-1,4])

        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(data[:,:3])

        pcds.append(new_pcd)

    return pcds


def pcd2voxels(pcds, voxel_size=0.0):
    radius_normal = voxel_size * 2
    pcds_down = []
    for pcd in pcds:
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        pcds_down.append(pcd_down)
    return pcds_down


def save_pcd(idx, pcd):
    print('Saving icp points to ply format...')
    global_ply_filename = idx2filename(idx, 'ply')
    global_ply_path = GLOBAL_PLY_PATH / global_ply_filename
    o3d.io.write_point_cloud(str(global_ply_path), pcd, write_ascii=True)


def save_pcds(pcds):
    print('Saving icp points to ply format...')
    for idx, pcd in tqdm(pcds.items()):
        global_ply_filename = idx2filename(idx, 'ply')
        global_ply_path = GLOBAL_PLY_PATH / global_ply_filename
        o3d.io.write_point_cloud(str(global_ply_path), pcd, write_ascii=True)


@click.group(invoke_without_command=True)
@click.option('-n', '--num-pc', required=True, type=int,
              help='Number of Point Clouds to register.')
def start(num_pc):
    voxel_size = 0.1
    all_files = sorted(os.listdir(PC_PATH))
    total_files = num_pc
    print('Total files :', total_files)

    print(f'------- GENERATING GLOBAL WITH {total_files} FILES -------')
    pcds = load_point_clouds(total_files)
    pcds_down = pcd2voxels(pcds, voxel_size)

    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds_down,
                                    max_correspondence_distance_coarse,
                                    max_correspondence_distance_fine)
    
    print("Optimizing PoseGraph ...")
    option = o3d.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.registration.global_optimization(
            pose_graph, o3d.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.registration.GlobalOptimizationConvergenceCriteria(), option)

    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)

    pcds = load_point_clouds(total_files)
    pcds_down = pcd2voxels(pcds, voxel_size)
    pcd_combined = o3d.geometry.PointCloud()
    
    for point_id in range(len(pcds_down)):
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds_down[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    save_pcd(total_files - 1, pcd_combined_down)

    print(f"Finished ICP registration with {num_pc} files.")

if __name__ == "__main__":
    start()