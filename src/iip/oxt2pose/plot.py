import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np


def plot_trajetory(poses):
    l = 3  # coordinate axis length
    A = np.asarray([[0, 0, 0, 1], [l, 0, 0, 1], [0, 0, 0, 1], [0, l, 0, 1], [0, 0, 0, 1], [0, 0, l, 1]]).T
    print('[UTILS]\nA:', A)
    for idx, pose in poses.items():
        print('[UTILS]\npose:', pose)
        B = pose.dot(A)
        print('[UTILS]\nB:', B)
        if idx == 0:
            plt.plot(B[0][0], B[1][0], marker='o', c='tab:green')
            continue
        if idx == len(pose) - 1:
            plt.plot(B[0][0], B[1][0], marker='o', c='tab:red')
            continue
        plt.plot(B[0][0], B[1][0], marker='o')
    plt.show()


def show_pc(pcd):
    o3d.visualization.draw_geometries([pcd])


def show_pcs(pc_list):
    COLORS = {'green': [0, 255, 0], 'red': [255, 0, 0], 'blue': [0, 0, 255],
              'soft_blue': [0, 96, 255], 'black': [0, 0, 0], 'yellow': [255, 255, 0]}
    margin = 8.0
    new_pc_list = []
    pc_1 = pc_list[0][0]
    pc_1_color = pc_list[0][1]
    # define first point cloud color
    pc_1_size = len(np.asarray(pc_1.points))
    np_colors = np.array([COLORS[pc_1_color] for _ in range(pc_1_size)]).astype(np.float) / 255.0
    print(f'colors: {np_colors}')
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
        np_colors = np.array([COLORS[pc_2_color] for _ in range(offset_pc_size)]).astype(np.float) / 255.0
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
        np_colors = np.array([COLORS[pc_3_color] for _ in range(offset_pc_size)]).astype(np.float) / 255.0
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
        np_colors = np.array([COLORS[pc_4_color] for _ in range(offset_pc_size)]).astype(np.float) / 255.0
        offset_pc.colors = o3d.utility.Vector3dVector(np_colors)

        new_pc_list.append(offset_pc)

    o3d.visualization.draw_geometries(new_pc_list)
