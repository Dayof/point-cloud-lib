import os
import shutil
from tqdm import tqdm
import subprocess

BASE_PATH = os.getenv('BASE_PATH', False)
VELODYNE_PATH = os.path.join(BASE_PATH, 'data')
PLY_PATH = os.path.join(BASE_PATH, 'data_ply')
PCD_PATH = os.path.join(BASE_PATH, 'data_pcd')

ply_format_template = lambda v: f'ply\nformat ascii 1.0\nelement vertex {v}\n'\
                                 'property float x\nproperty float y\n'\
                                 'property float z\nproperty float '\
                                 'reflect_coeff\nend_header\n'

def check_create_folders():
    if not os.path.exists(PLY_PATH):
        os.mkdir(PLY_PATH)
    if not os.path.exists(PCD_PATH):
        os.mkdir(PCD_PATH)

def txt2ply():
    print('Converting txt to ply...')

    for f in tqdm(os.listdir(VELODYNE_PATH)):
        file_path = os.path.join(VELODYNE_PATH, f)

        num_lines = 0
        with open(file_path, 'r') as of:
            lines = of.readlines()
            num_lines = len([l for l in lines if l.strip(' \n') != ''])

        f_no_ext = f.split('.')[0]
        new_file = f_no_ext + '.ply'
        new_file_path = os.path.join(PLY_PATH, new_file)
        shutil.copyfile(file_path, new_file_path) 

        new_header = ply_format_template(num_lines)
        with open(new_file_path, 'r+') as nf:
            content = nf.read()
            nf.seek(0, 0)
            nf.write(new_header + content)

    print('Finished.')

def ply2pcd():
    print('Converting ply to pcd...')

    for cur_file in tqdm(os.listdir(PLY_PATH)):
        file_path = os.path.join(PLY_PATH, cur_file)

        f_no_ext = cur_file.split('.')[0]
        new_file = f_no_ext + '.pcd'
        new_file_path = os.path.join(PCD_PATH, new_file)

        subprocess.run(['pcl_ply2pcd', file_path, new_file_path, '-format 1'])

    print('Finished.')

def start():
    if not BASE_PATH:
        print('Please set the BASE_PATH first.')
        quit()
    check_create_folders()
    txt2ply()
    ply2pcd()

if __name__ == "__main__":
    start()