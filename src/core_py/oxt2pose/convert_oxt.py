import numpy as np
import math


def lat2scale(lat):
    return math.cos((lat * math.pi) / 180.0)


def latlon2mercator(lat, lon, scale):
    er = 6378137
    mx = scale * lon * math.pi * er / 180
    my = scale * er * math.log(math.tan((90 + lat) * math.pi / 360))
    return mx, my


def get(oxts):
    # converts a list of oxts measurements into metric poses,
    # starting at (0,0,0) meters, OXTS coordinates are defined as
    # x = forward, y = right, z = down (see OXTS RT3000 user manual)
    # afterwards, pose{i} contains the transformation which takes a
    # 3D point in the i'th frame and projects it into the oxts
    # coordinates of the first frame.

    # compute scale from first lat value
    scale = lat2scale(oxts[0][0])

    # init pose
    pose     = {}
    Tr_0_inv = []

    # for all oxts packets do
    for i in range(len(oxts)):
        print('[CONVERT]\nidx:', i)
        # if there is no data => no pose
        if len(oxts[i]) == 0:
            pose[i] = []
            continue

        print('[CONVERT]\nlat, lon, alt, scale:', oxts[i][0], oxts[i][1], oxts[i][2], scale)

        # translation vector
        t = np.array([0.0, 0.0, 0.0])
        t[0], t[1] = latlon2mercator(oxts[i][0], oxts[i][1], scale)
        t[2] = oxts[i][2]
        print('[CONVERT]\nt:', t)

        # rotation matrix (OXTS RT3000 user manual, page 71/92)
        rx = oxts[i][3]; # roll
        ry = oxts[i][4]; # pitch
        rz = oxts[i][5]; # heading 
        print('[CONVERT]\nrx, ry, rz:', rx, ry, rz)

        # base => nav  (level oxts => rotated oxts)
        Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
        # base => nav  (level oxts => rotated oxts)
        Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
        # base => nav  (level oxts => rotated oxts)
        Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
        R = Rz.dot(Ry.dot(Rx))  # Rz*Ry*Rx
        print('[CONVERT]\nR:', R)

        R = R.reshape(3, 3)
        t = t.reshape(3, 1)
        main_mat = np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))

        print('[CONVERT]\nmain_mat:', main_mat)
        
        # normalize translation and rotation (start at 0/0/0)
        if len(Tr_0_inv) == 0:
            Tr_0_inv = np.linalg.inv(main_mat)
 
        # add pose
        pose[i] = Tr_0_inv.dot(main_mat)
        print('[CONVERT]\npose:', pose[i])
    return pose
