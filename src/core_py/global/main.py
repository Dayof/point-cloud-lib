import open3d as o3d 
import numpy as np
from pathlib import Path
import math
import mpu

# get data base path to collect the point clouds
BASE = Path().resolve().parents[2]
DATA_PATH = BASE / 'data'
SHOW_PC = False
COLORS = {'green': [0, 255, 0], 'red': [255, 0, 0], 'blue': [0, 0, 255],
          'black': [0, 0, 0], 'yellow': [255, 255, 0]}
RADIUS = 6378137  # earth radius in meters


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
    pc_path = DATA_PATH / 'kitti' / pc_filename
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
    x = scale * RADIUS * ((math.pi * lon) / 180) 
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
    return roll, pitch, yaw  # rx, ry, rz


def rotate(rpy):
    rx, ry, rz = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    return Rz.dot(Ry.dot(Rx))  # Rz*Ry*Rx


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


if __name__ == '__main__':
    t0, scale = None, None
    all_pcs_points, all_pcs = [], []
    for idx in range(3):
        print(f'Calculating pc {idx}...')
        pc = read_pc(idx, COLORS['blue'])
        pc_lla = get_lla(idx)
        if idx == 0:
            scale = calc_scale(pc_lla[0])
            print('scale:', scale)
            t0 = calc_mercator(scale, pc_lla)
            print('x0, y0, z0', t0)

        t = calc_mercator(scale, pc_lla)
        print('new x, y, z', t)
        pc_rpy = get_rpy(idx)
        R = rotate(pc_rpy)
        mat_trans = rot_trans(R, np.asarray(pc.points), t - t0)
        new_pc = gen_new_pc(pc, mat_trans)
        all_pcs.append(new_pc)
        all_pcs_points.append(np.asarray(new_pc.points))
    compl_map_arr = mult_to_one(all_pcs_points)
    compl_map = gen_map(compl_map_arr)
    # show_pc([(compl_map, 'blue')])
    show_pc([(all_pcs[0], 'black'), (all_pcs[1], 'red'), (all_pcs[2], 'green'),
             (compl_map, 'blue')])
