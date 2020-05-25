import os
import open3d as o3d 
import numpy as np
import visualization as visu

BASE_PATH = os.getenv('BASE_PATH', False)
VELODYNE_PATH = os.path.join(BASE_PATH, 'velodyne_points')
PCD_PATH = os.path.join(VELODYNE_PATH, 'data_pcd')

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

def load_point_clouds(all_files, n_files, voxel_size=0.0):
    colours = [[1, 0.706, 0], [0, 0.651, 0.929], [0, 1, 0.5]]
    pcds = []
    radius_normal = voxel_size * 2
    for i in range(n_files):
        file_path = os.path.join(PCD_PATH, all_files[i])
        print(file_path)
        pcd = o3d.io.read_point_cloud(file_path)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        # pcd_down.paint_uniform_color(colours[i])
        pcds.append(pcd_down)
    return pcds

def start():
    voxel_size = 0.1
    all_files = sorted(os.listdir(PCD_PATH))
    total_files = len(all_files)
    print('Total files :', total_files)
    for nr in range(total_files):
        print(f'------- GENERATING GLOBAL WITH {nr} FILES -------')
        pcds_down = load_point_clouds(all_files, nr, voxel_size)
        # o3d.visualization.draw_geometries(pcds_down)
        
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
        # o3d.visualization.draw_geometries(pcds_down)

        pcds = load_point_clouds(all_files, nr, voxel_size)
        pcd_combined = o3d.geometry.PointCloud()
        
        for point_id in range(len(pcds)):
            pcds[point_id].transform(pose_graph.nodes[point_id].pose)
            pcd_combined += pcds[point_id]
        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
        o3d.io.write_point_cloud(f"multiway_registration_{len(pcds)}.pcd", pcd_combined_down)
        # o3d.visualization.draw_geometries([pcd_combined_down])

if __name__ == "__main__":
    start()