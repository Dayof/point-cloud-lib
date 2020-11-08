import open3d as o3d 
import numpy as np
from pathlib import Path
import scipy
import math
from tqdm import tqdm
import itertools
import multiprocessing


# get data base path to collect the point clouds
BASE = Path().resolve().parents[1]
DATA_PATH = BASE / 'data'
print(DATA_PATH)
SHOW_PC = True
THREADS = 10
THRESHOLD = 0.5  # 5 cm
COLORS = {'green': [0, 255, 0], 'red': [255, 0, 0], 'blue': [0, 0, 255],
          'black': [0, 0, 0], 'yellow': [255, 255, 0]}


def calc_eucl_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


def check_threshold(p1, p2):
    if ((p2[0] + THRESHOLD > p1[0]) and (p1[0] > p2[0] - THRESHOLD)) and         ((p2[1] + THRESHOLD > p1[1]) and (p1[1] > p2[1] - THRESHOLD)) and         ((p2[2] + THRESHOLD > p1[2]) and (p1[2] > p2[2] - THRESHOLD)):
        return calc_eucl_dist(p1, p2) <= THRESHOLD


def load_point_cloud(idx, color='blue'):
    # get first point cloud, assign it with a blue color
    filename = f'{idx}'.zfill(10) + '.ply'
    pc_path = DATA_PATH / 'kitti' / filename
    print('Collecting pc: ', pc_path)
    pcd = o3d.io.read_point_cloud(str(pc_path), format='ply')
    pcd_size = len(np.asarray(pcd.points))

    # define point cloud color
    np_colors = np.array([COLORS[color] for _ in range(pcd_size)])
    pcd.colors = o3d.utility.Vector3dVector(np_colors)

    pc_tot_size = len(point_clouds)
    # show point cloud info
    print(f'PCD {pc_tot_size}:\n', pcd, f'\nPCD {pc_tot_size} size: ', pcd_size,
          f'\n\nPCD {pc_tot_size} colors: \n', np.asarray(pcd.colors),
          f'\n\nPCD {pc_tot_size} points:\n', np.asarray(pcd.points))
    return pcd


def show_pc(pc_list):
    if SHOW_PC:
        new_pc_list = []
        pc_1 = pc_list[0]
        # define first point cloud color
        pc_1_size = len(np.asarray(pc_1.points))
        np_colors = np.array([COLORS['red'] for _ in range(pc_1_size)])
        pc_1.colors = o3d.utility.Vector3dVector(np_colors)
        new_pc_list.append(pc_1)

        if len(pc_list) == 2 or len(pc_list) == 3:
            pc_2 = pc_list[1]

            # offset the second point cloud
            offset_pc = o3d.geometry.PointCloud(pc_list[1])
            offset_pc.points = o3d.utility.Vector3dVector(
                np.asarray(pc_2.points) + [0.0, 0.0, -5.0])
            offset_pc_size = len(np.asarray(offset_pc.points))

            # define second point cloud color
            np_colors = np.array([COLORS['green'] for _ in range(offset_pc_size)])
            offset_pc.colors = o3d.utility.Vector3dVector(np_colors)
            
            new_pc_list.append(offset_pc)
        
        if len(pc_list) == 3:
            pc_3 = pc_list[2]

            # offset the third point cloud
            offset_pc = o3d.geometry.PointCloud(pc_3)
            offset_pc.points = o3d.utility.Vector3dVector(
                np.asarray(pc_3.points) + [0.0, 0.0, -10.0])
            offset_pc_size = len(np.asarray(offset_pc.points))

            # define the point cloud color
            np_colors = np.array([COLORS['blue'] for _ in range(offset_pc_size)])
            offset_pc.colors = o3d.utility.Vector3dVector(np_colors)

            new_pc_list.append(offset_pc)

        o3d.visualization.draw_geometries(new_pc_list)


def sort_pc(pc):
    pc_np = np.asarray(pc.points)
    idx_sorted_pc = np.lexsort((pc_np[:,1], pc_np[:,0]))
    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(pc_np[idx_sorted_pc])
    return new_pc


def get_min_max(pc):
    pc_np = np.asarray(pc.points)
    p_max, p_min = np.amax(pc_np, axis=0), np.amin(pc_np, axis=0)
    return p_max, p_min


def split_map(pc):
    pc_np = np.asarray(pc.points)
    new_pc_np = pc_np[(pc_np[:,0] >= -1.5) & (pc_np[:,0] <= 1.5)]
    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(new_pc_np)
    print(f'Reduzed {len(pc_np)} point to {len(new_pc_np)} points.')
    return new_pc


def brute_force_eucl_dist(pc_1, pc_2):  # too slow
    pts_set = set()
    for idx_1, p1 in enumerate(tqdm(np.asarray(pc_1.points))):
        for idx_2, p2 in enumerate(np.asarray(pc_2.points)):
            if check_threshold(p1, p2):
                pts_set.add(idx_1)
    return pts_set


def parallel_brute_force_eucl_dist(pc_1, pc_2):

    def inner_logic(params):
        idx, pcs = params
        print(idx)
        if check_threshold(pcs[0], pcs[1]):
            return idx

    pc_1_np, pc_2_np = np.asarray(pc_1.points), np.asarray(pc_2.points)

    idx_pc_1_np = []
    for x in range(len(pc_1_np)):
        idx_pc_1_np.extend([x]*3)

    params = list(itertools.product(pc_1_np, pc_2_np))
    idx_and_params = zip(idx_pc_1_np, params)

    with multiprocessing.Pool(2) as p:
        res  = list(tqdm.tqdm(p.imap(inner_logic, idx_and_params)))
    
    res_f = list(filter(None.__ne__, res))

    return res_f


def get_pc_from_pts_set(pc_set, pc):
    pts_list = list(pc_set)
    pc_np = np.asarray(pc.points) 
    pc_np_from_pts = pc_np[pts_list]

    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(pc_np_from_pts)
    return new_pc


def scipy_eucl_dist(pc_1, pc_2):
    dist = scipy.spatial.distance.cdist(np.asarray(pc_1.points), np.asarray(pc_2.points))

def seq_clustering():
    cluster_pts = brute_force_eucl_dist(point_clouds[0], point_clouds[1])
    cluster_pc = get_pc_from_pts_set(cluster_pts, point_clouds[0])
    show_pc([point_clouds[0], point_clouds[1], cluster_pc])


def parallel_clustering():
    cluster_pts = parallel_brute_force_eucl_dist(point_clouds[0], point_clouds[1])
    cluster_pc = get_pc_from_pts_set(cluster_pts, point_clouds[0])
    show_pc([point_clouds[0], point_clouds[1], cluster_pc])


if __name__ == '__main__':
    point_clouds = []

    pc = load_point_cloud(0, 'blue')
    sorted_pc = sort_pc(pc)
    partial_pc = split_map(sorted_pc)
    point_clouds.append(partial_pc)
    # show_pc([pc, partial_pc])

    pc = load_point_cloud(10, 'red')
    sorted_pc = sort_pc(pc)
    partial_pc = split_map(sorted_pc)
    point_clouds.append(partial_pc)
    # show_pc([pc, partial_pc])

    cluster_pts = parallel_brute_force_eucl_dist(point_clouds[0], point_clouds[1])
    cluster_pc = get_pc_from_pts_set(cluster_pts, point_clouds[0])
    show_pc([point_clouds[0], point_clouds[1], cluster_pc])


