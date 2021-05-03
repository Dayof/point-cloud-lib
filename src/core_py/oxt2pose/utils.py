from pathlib import Path
from tqdm import tqdm
import open3d as o3d
import numpy as np
import os

BASE = Path().resolve().parents[2]
DATA_PATH = BASE / 'data'
GLOBAL_PLY_PATH = DATA_PATH / 'global'


def idx2filename(idx, format):
    return f'{idx}'.zfill(10) + f'.{format}'


def txtlines2lists(file_path):
    all_lines = []
    with open(file_path, 'r') as df:
        cur_line = df.readline().strip().split(' ')
        cur_line_floats = list(map(float, cur_line))
        all_lines.append(cur_line_floats)

    if len(all_lines) == 1:
        return all_lines[0]
    return all_lines


def get_point_clouds(base_dir, num_pcs):
    pcs_path = os.path.join(base_dir, 'velodyne_points', 'data')
    pcs = {}
    for i in range(num_pcs):
        filename = idx2filename(i, 'bin')
        filepath = os.path.join(pcs_path, filename)
        data = np.fromfile(filepath, dtype=np.float32, count=-1).reshape([-1,4])
        pcs[i] = data
    return pcs


def points2open3dpcd(pcs):
    pcds = {}
    for idx, pc in pcs.items():
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(pc[:,:3])
        pcds[idx] = new_pcd
    return pcds


def save_pcds(pcds):
    ply_format_template = lambda v: f'ply\nformat ascii 1.0\nelement vertex {v}\n'\
                                 'property float x\nproperty float y\n'\
                                 'property float z\nproperty float '\
                                 'reflect_coeff\nend_header\n'
    print('Saving global points to ply format...')
    for idx, pcd in tqdm(pcds.items()):
        global_ply_filename = idx2filename(idx, 'ply')
        global_ply_path = GLOBAL_PLY_PATH / global_ply_filename
        o3d.io.write_point_cloud(str(global_ply_path), pcd, write_ascii=True)
