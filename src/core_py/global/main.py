import open3d as o3d 
import numpy as np
from pathlib import Path
from math import sin, cos, sqrt, atan2, radians
import mpu

# get data base path to collect the point clouds
BASE = Path().resolve().parents[2]
DATA_PATH = BASE / 'data'
SHOW_PC = False
COLORS = {'blue': [0, 0, 255], 'red': [255, 0, 0]}

def read_pc(pc_filename, color):
    # get first point cloud, assign it with a blue color
    pc_path = DATA_PATH / 'kitti' / pc_filename
    pcd = o3d.io.read_point_cloud(str(pc_path), format='ply')
    pcd_size = len(np.asarray(pcd.points))

    # define point cloud color
    np_colors = np.array([color for _ in range(pcd_size)])
    pcd.colors = o3d.utility.Vector3dVector(np_colors)

    # show point cloud info
    print('PCD:\n', pcd, '\n', 'PCD size: ', pcd_size, '\n',
          '\nPCD colors: \n', np.asarray(pcd.colors), '\n', '\nPCD points:\n', np.asarray(pcd.points))

    if SHOW_PC:
        o3d.visualization.draw_geometries([pcd])


def calc_haversine():
    # calculate diff meters between pc 1 and pc 2 
    lat1, lon1 = radians(49.015019172077), radians(8.4343357035849)  # pc 1
    lat2, lon2 = radians(49.015012999213), radians(8.4343202373229)  # pc 2

    dist = mpu.haversine_distance((lat1, lon1), (lat2, lon2))
    print("Haversine Result (km):", dist)
    dist_m_1 = dist * 1000
    print("Haversine Result (m):", dist_m_1)
    print("Haversine Result (cm):", dist * 100000)


def get_lat_lon():
    DATA_PATH / 'oxts'


if __name__ == '__main__':
    pc_1 = read_pc('0000000000.ply', COLORS['blue'])
    pc_2 = read_pc('0000000001.ply', COLORS['red'])
    pc_1_lat_lon = get_lat_lon('0000000000')
    pc_2_lat_lon = get_lat_lon('0000000001')
