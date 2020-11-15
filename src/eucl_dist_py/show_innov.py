import open3d as o3d 
import numpy as np
from pathlib import Path

# get data base path to collect the point clouds
BASE = Path().resolve().parents[1]
DATA_PATH = BASE / 'data'
print(DATA_PATH)
COLORS = {'green': [0, 255, 0], 'red': [255, 0, 0], 'blue': [0, 0, 255],
          'black': [0, 0, 0], 'yellow': [255, 255, 0]}
point_clouds = []


def load_point_cloud(filename, color='blue', base='innovations'):
    # get first point cloud, assign it with a blue color
    pc_path = DATA_PATH / base / filename
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
    new_pc_list = []
    pc_1 = pc_list[0]
    # define first point cloud color
    pc_1_size = len(np.asarray(pc_1.points))
    np_colors = np.array([COLORS['red'] for _ in range(pc_1_size)])
    pc_1.colors = o3d.utility.Vector3dVector(np_colors)
    new_pc_list.append(pc_1)

    if len(pc_list) == 2 or len(pc_list) == 3 or len(pc_list) == 4:
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
    
    if len(pc_list) == 3 or len(pc_list) == 4:
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
            
    if len(pc_list) == 4:
        pc_4 = pc_list[3]

        # offset the third point cloud
        offset_pc = o3d.geometry.PointCloud(pc_4)
        offset_pc.points = o3d.utility.Vector3dVector(
            np.asarray(pc_4.points) + [0.0, 0.0, -15.0])
        offset_pc_size = len(np.asarray(offset_pc.points))

        # define the point cloud color
        np_colors = np.array([COLORS['black'] for _ in range(offset_pc_size)])
        offset_pc.colors = o3d.utility.Vector3dVector(np_colors)

        new_pc_list.append(offset_pc)

    o3d.visualization.draw_geometries(new_pc_list)


if __name__ == "__main__":
    pc_1 = load_point_cloud('0000000000.ply', 'red', 'kitti')
    pc_2 = load_point_cloud('0000000001.ply', 'green', 'kitti')
    pc_innov = load_point_cloud('0_1.ply', 'black')
    show_pc([pc_1, pc_2, pc_innov])
