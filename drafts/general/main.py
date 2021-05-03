from scipy.spatial.transform import Rotation as R
import robopy
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
import open3d as o3d 
import numpy as np
import math
import mpu
import os

# get data base path to collect the point clouds
BASE = Path().resolve().parents[2]
DATA_PATH = BASE / 'data'
GLOBAL_PLY_PATH = DATA_PATH / 'global'
KITTI_PLY_PATH = DATA_PATH / 'kitti_ply'
SHOW_PC = False
COLORS = {'green': [0, 255, 0], 'red': [255, 0, 0], 'blue': [0, 0, 255],
          'black': [0, 0, 0], 'yellow': [255, 255, 0]}
RADIUS = 6378137  # earth radius in meters
ply_format_template = lambda v: f'ply\nformat ascii 1.0\nelement vertex {v}\n'\
                                 'property float x\nproperty float y\n'\
                                 'property float z\nproperty float '\
                                 'reflect_coeff\nend_header\n'


def show_pc(pc_list):
    margin = 8.0
    new_pc_list = []
    pc_1 = pc_list[0][0]
    pc_1_color = pc_list[0][1]
    # define first point cloud color
    pc_1_size = len(np.asarray(pc_1.points))
    np_colors = np.array([COLORS[pc_1_color] for _ in range(pc_1_size)])
    pc_1.colors = o3d.utility.Vector3dVector(np_colors)
    new_pc_list.append(pc_1)

    if len(pc_list) == 2 or len(pc_list) == 3 or len(pc_list) == 4:
        pc_2 = pc_list[1][0]
        pc_2_color = pc_list[1][1]

        # offset the second point cloud
        offset_pc = o3d.geometry.PointCloud(pc_2)
        offset_pc.points = o3d.utility.Vector3dVector(
            np.asarray(pc_2.points) + [0.0, 0.0, -margin])
        offset_pc_size = len(np.asarray(offset_pc.points))

        # define second point cloud color
        np_colors = np.array([COLORS[pc_2_color] for _ in range(offset_pc_size)])
        offset_pc.colors = o3d.utility.Vector3dVector(np_colors)
        
        new_pc_list.append(offset_pc)
    
    if len(pc_list) == 3 or len(pc_list) == 4:
        pc_3 = pc_list[2][0]
        pc_3_color = pc_list[2][1]

        # offset the third point cloud
        offset_pc = o3d.geometry.PointCloud(pc_3)
        offset_pc.points = o3d.utility.Vector3dVector(
            np.asarray(pc_3.points) + [0.0, 0.0, -(margin * 2)])
        offset_pc_size = len(np.asarray(offset_pc.points))

        # define the point cloud color
        np_colors = np.array([COLORS[pc_3_color] for _ in range(offset_pc_size)])
        offset_pc.colors = o3d.utility.Vector3dVector(np_colors)

        new_pc_list.append(offset_pc)
            
    if len(pc_list) == 4:
        pc_4 = pc_list[3][0]
        pc_4_color = pc_list[3][1]

        # offset the third point cloud
        offset_pc = o3d.geometry.PointCloud(pc_4)
        offset_pc.points = o3d.utility.Vector3dVector(
            np.asarray(pc_4.points) + [0.0, 0.0, -(margin * 3)])
        offset_pc_size = len(np.asarray(offset_pc.points))

        # define the point cloud color
        np_colors = np.array([COLORS[pc_4_color] for _ in range(offset_pc_size)])
        offset_pc.colors = o3d.utility.Vector3dVector(np_colors)

        new_pc_list.append(offset_pc)

    o3d.visualization.draw_geometries(new_pc_list)


def read_pc(idx, color):
    pc_filename = f'{idx}'.zfill(10) + '.ply'
    pc_path = KITTI_PLY_PATH / pc_filename
    pcd = o3d.io.read_point_cloud(str(pc_path), format='ply')
    pcd_size = len(np.asarray(pcd.points))

    # define point cloud color
    np_colors = np.array([color for _ in range(pcd_size)])
    pcd.colors = o3d.utility.Vector3dVector(np_colors)

    # show point cloud info
    print('PCD:\n', pcd, '\n', 'PCD size: ', pcd_size, '\n',
          '\nPCD colors: \n', np.asarray(pcd.colors), '\n',
          '\nPCD points:\n', np.asarray(pcd.points))

    if SHOW_PC:
        o3d.visualization.draw_geometries([pcd])

    return pcd


def new_pc(pc_1, new_pc_2):
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(np.asarray(pc_1.points))
    offset_pc_size = len(np.asarray(offset_pc.points))


def calc_haversine(pc_1_lat_lon, pc_2_lat_lon):
    # calculate diff meters between pc 1 and pc 2 
    lat1, lon1 = radians(pc_1_lat_lon[0]), radians(pc_1_lat_lon[1])  # pc 1
    lat2, lon2 = radians(pc_2_lat_lon[0]), radians(pc_2_lat_lon[1])  # pc 2

    dist = mpu.haversine_distance((lat1, lon1), (lat2, lon2))
    print("Haversine Result (km):", dist)
    dist_m = dist * 1000
    print("Haversine Result (m):", dist_m)
    print("Haversine Result (cm):", dist * 100000)

    return dist_m


def calc_mercator(scale, lat_lon_alt):
    lat, lon, alt = lat_lon_alt
    x = - (scale * RADIUS * ((math.pi * lon) / 180))
    y = scale * RADIUS * math.log(math.tan((math.pi*(90 + lat)) / 360))
    return np.array([x, y, alt])


def get_lla(idx):
    oxt_filename = f'{idx}'.zfill(10) + '.txt'
    file_path = DATA_PATH / 'oxts' / oxt_filename
    with open(file_path, 'r') as df:
        cur_line = df.readline().split(' ')
        lat, lon, alt = float(cur_line[0]), float(cur_line[1]), float(cur_line[2])
    return lat, lon, alt


def get_rpy(idx):
    oxt_filename = f'{idx}'.zfill(10) + '.txt'
    file_path = DATA_PATH / 'oxts' / oxt_filename
    with open(file_path, 'r') as df:
        cur_line = df.readline().split(' ')
        roll, pitch, yaw = float(cur_line[3]), float(cur_line[4]), float(cur_line[4])
    print(f'roll(rx): {roll}, pitch(ry): {pitch}, yaw(rz): {yaw}')
    return roll, pitch, yaw  # rx, ry, rz


def rotate_kitti(rpy):
    rx, ry, rz = rpy
    # roll
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    # pitch
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    # yaw
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    return Rz.dot(Ry.dot(Rx))  # Rz*Ry*Rx


def rotate_robopy(rpy):
    return robopy.rpy2r(list(rpy), order='zyx', unit='rad')


def rot_trans(R, P, t):
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)
    return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


def calc_scale(pc_1_lat):
    return math.cos((pc_1_lat * math.pi) / 180)


def gen_new_pc(pc, mat_trans):
    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(np.asarray(pc.points))
    new_pc.transform(mat_trans)
    return new_pc


def gen_map(compl_map_arr):
    new_pc_map = o3d.geometry.PointCloud()
    new_pc_map.points = o3d.utility.Vector3dVector(compl_map_arr)
    return new_pc_map


def mult_to_one(all_pcs):
    return np.concatenate(all_pcs)


def pc2global(idx, pc):
    print('Saving global points to ply format...')
    global_ply_filename = f'{idx}'.zfill(10) + '.ply'
    global_ply_path = GLOBAL_PLY_PATH / global_ply_filename
    o3d.io.write_point_cloud(str(global_ply_path), pc, write_ascii=True)
    print('Saved.')

def plot_trajetory(trajetory):
    plt.plot(trajetory['x'][0], trajetory['y'][0], marker='o', c='tab:green')
    plt.plot(trajetory['x'][1:-1], trajetory['y'][1:-1], marker='o')
    plt.plot(trajetory['x'][-1], trajetory['y'][-1], marker='o', c='tab:red')
    plt.show()

if __name__ == '__main__':
    t0, scale = None, None
    all_pcs_points, all_pcs = [], []
    trajetory = {'x': [], 'y': []}
    num_files = len(os.listdir(KITTI_PLY_PATH))
    print(f'Processing {num_files} files...')
    for idx in range(num_files):
    # for idx in range(4):
        print(f'Calculating pc {idx}...')
        pc = read_pc(idx, COLORS['blue'])
        pc_lla = get_lla(idx)
        if idx == 0:
            scale = calc_scale(pc_lla[0])
            print('scale:', scale)
            t0 = calc_mercator(scale, pc_lla)
            print('x0, y0, z0', t0)

        t = calc_mercator(scale, pc_lla)
        trajetory['x'].append(t[0])
        trajetory['y'].append(t[1])
        print('new x, y, z', t)
        pc_rpy = get_rpy(idx)
        R = rotate_robopy(pc_rpy)
        print(f'rotation: \n{R}')
        transl = t - t0
        print(f'translation: \n{transl}')
        mat_trans = rot_trans(R, np.asarray(pc.points), transl)
        print(f'translation and rotation matrix: \n{mat_trans}')
        new_pc = gen_new_pc(pc, mat_trans)
        pc2global(idx, new_pc)
        all_pcs.append(new_pc)
        all_pcs_points.append(np.asarray(new_pc.points))
    plot_trajetory(trajetory)
    # compl_map_arr = mult_to_one(all_pcs_points)
    # compl_map = gen_map(compl_map_arr)
    # show_pc([(compl_map, 'blue')])
    # show_pc([(all_pcs[0], 'black'), (all_pcs[1], 'red'), (all_pcs[2], 'green'),
    #          (compl_map, 'blue')])
