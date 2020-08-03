import copy
import open3d as o3d 

def multiple_plot(cloud_f, cloud_s):
    lcloud_f = o3d.io.read_point_cloud(cloud_f)
    lcloud_s = o3d.io.read_point_cloud(cloud_s)

    lcloud_f.paint_uniform_color([1, 0.706, 0])
    lcloud_s.paint_uniform_color([0, 0.651, 0.929])

    o3d.visualization.draw_geometries([lcloud_f, lcloud_s])

def plot(cloud):
    cloud = o3d.io.read_point_cloud(cloud) # Read the point cloud
    o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud   

def plot_reg(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
