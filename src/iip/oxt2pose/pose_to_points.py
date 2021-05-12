import numpy as np


def apply(poses, pcds):
    pcds_t = {}
    for idx, pcd in pcds.items():
        pcds_t[idx] = pcd.transform(poses[idx])
    return pcds_t
