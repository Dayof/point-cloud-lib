import os

import pose_to_points 
import convert_oxt 
import load_oxt 
import utils
import plot

# data base path to collect the point clouds
BASE_DIR = os.getenv('BASE_DIR', '/home/dayoff/data/2011_09_26_drive_0001')
DATA_SYNC_DIR = os.path.join(BASE_DIR, '2011_09_26_drive_0001_sync')


if __name__ == '__main__':
    oxts = load_oxt.get(DATA_SYNC_DIR)
    poses = convert_oxt.get(oxts)
    p_clouds = utils.get_point_clouds(DATA_SYNC_DIR, len(oxts))
    pcds = utils.points2open3dpcd(p_clouds)
    p_clouds_t = pose_to_points.apply(poses, pcds)
    utils.save_pcds(p_clouds_t)
    # plot.plot_trajetory(poses)
    # plot.show_pc(p_clouds_t[0])
    # plot.show_pcs([(p_clouds_t[0], 'red'), (p_clouds_t[len(poses) - 1], 'blue')])
