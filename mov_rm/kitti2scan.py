import struct
import sys
import os
import numpy as np
import random
from PIL import Image
import click
from tqdm import tqdm

BASE_PATH = os.getenv('BASE_PATH', False)

try:
    import py3dtk
    import py3dtk.utils
except ImportError:
    print("Cannot find py3dtk module. Try recompiling 3dtk with WITH_PYTHON set to ON", file=sys.stderr)
    exit(1)


# WGS84 ellipsoid
earth_semimajor = 6378137
earth_flattening = 1/298.257223563
earth_semiminor = earth_semimajor * (1-earth_flattening)
earth_ecc2 = earth_flattening * (2 - earth_flattening)

# converts WGS-84 Geodetic point (lat, lon, h)
# to Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)
def geo2ecef(lat, lon, h):
    λ = np.deg2rad(lat)
    φ = np.deg2rad(lon)
    s = np.sin(λ)
    N = earth_semimajor / np.sqrt(1 - earth_ecc2 * s * s)
    x = (h + N) * np.cos(λ) * np.cos(φ)
    y = (h + N) * np.cos(λ) * np.sin(φ)
    z = (h + (1 - earth_ecc2) * N) * np.sin(λ)
    return x, y, z

# Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)
# to East-North-Up coordinates in a Local Tangent Plane that is centered at
# the WGS-84) Geodetic point (lat0, lon0, h0)
def ecef2enu(x, y, z, lat0, lon0, h0):
    λ = np.deg2rad(lat0)
    φ = np.deg2rad(lon0)
    s = np.sin(λ)
    N = earth_semimajor / np.sqrt(1 - earth_ecc2 * s * s)
    x0 = (h0 + N) * np.cos(λ) * np.cos(φ)
    y0 = (h0 + N) * np.cos(λ) * np.sin(φ)
    z0 = (h0 + (1 - earth_ecc2) * N) * np.sin(λ)
    xd = x - x0
    yd = y - y0
    zd = z - z0
    xEast = -np.sin(φ) * xd + np.cos(φ) * yd
    yNorth = -np.cos(φ) * np.sin(λ) * xd - np.sin(λ) * np.sin(φ) * yd + np.cos(λ) * zd
    zUp = np.cos(λ) * np.cos(φ) * xd + np.cos(λ) * np.sin(φ) * yd + np.sin(λ) * zd
    return (xEast, yNorth, zUp)

# http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
#
# coordinate systems
#
#  - Camera:   x: right,   y: down,  z: forward
#  - Velodyne: x: forward, y: left,  z: up
#  - GPS/IMU:  x: forward, y: left,  z: up
#  - 3dtk:     x: right,   y: up,    z: forward
def get_transmat(fname):

    with open(os.path.join(BASE_PATH, '2011_09_26', fname)) as f:
        lines = f.readlines()

    assert len(lines) in [3,5]
    assert lines[0].startswith("calib_time: ")
    assert lines[1].startswith("R: ")
    assert lines[2].startswith("T: ")
    vals = [float(v) for v in lines[1][3:].split()]
    assert len(vals) == 9
    R = np.reshape(vals, (3,3))
    vals = [float(v) for v in lines[2][3:].split()]
    assert len(vals) == 3
    t = np.reshape(vals, (3,1))
    Rt = np.identity(4)
    Rt[0:3, 0:3] = R
    Rt[0:3, 3:4] = t
    return Rt


def main():
    numscans = 0
    dirname = os.path.join(BASE_PATH, '2011_09_26',
                           '2011_09_26_drive_0001_sync', 'image_02', 'data')
    
    for f in os.listdir(dirname):
        fname = os.path.join(dirname, f)
        if not os.path.isfile(fname):
            continue
        if not fname.endswith(".png"):
            continue
        numscans += 1

    print('verify the number of scans')
    # verify the number of scans
    for ts in ['velodyne_points', 'image_02', 'oxts']:
        fname = os.path.join(BASE_PATH, '2011_09_26',
                             '2011_09_26_drive_0001_sync', ts, 'timestamps.txt')
        numlines = 0
        with open(fname) as f:
            for line in f:
                numlines += 1
        print(numscans, numlines)
        assert numscans == numlines

    Rt_imu_to_velo = get_transmat('calib_imu_to_velo.txt')
    Rt_velo_to_cam = get_transmat('calib_velo_to_cam.txt')

    with open(os.path.join(BASE_PATH, '2011_09_26', 'calib_cam_to_cam.txt')) as f:
        for line in f:
            if line.startswith('P_rect_02: '):
                vals = [float(v) for v in line[11:].split()]
                assert len(vals) == 12
                P_rect_02 = np.reshape(vals, (3, 4))
            elif line.startswith('R_rect_00: '):
                vals = [float(v) for v in line[11:].split()]
                assert len(vals) == 9
                R_rect_00 = np.identity(4)
                R_rect_00[0:3, 0:3] = np.reshape(vals, (3,3))


    base = os.path.join(BASE_PATH, '2011_09_26',
                        '2011_09_26_drive_0001_sync')
    os.makedirs(base + '/3dtk', exist_ok=True)
    os.makedirs(base + '/changedetection', exist_ok=True)
    # rotation angles from the IMU assume a local coordinate system on a
    # tangent plane -> convert all coordinates to a local cartesian coordinate
    # system
    # This is also needed by bin/show which uses float and thus does not offer
    # sufficient precision to deal with values of magnitude of geo coordinates
    # down to centimeter precision at the same time
    geo0 = None
    allposes = list()
    # keep track of currscan separately to allow skipping scans without data
    currscan = 0
    for j in tqdm(range(numscans)):
        binpath = base + '/velodyne_points/data/%010d.bin' % j
        if not os.path.exists(binpath):
            continue
        with open(base + '/oxts/data/%010d.txt' % j) as f:
            # OxTS GPS/GNSS convention:
            # or: the uncomfortable right-handed system
            # x: forward, y: left, z: up
            # roll: positive => left side up, range: -π .. +π
            # pitch: positive => front down, range: -π/2 .. +π/2
            # yaw: 0 => east, positive => counter clockwise, range: -π .. +π
            lat, lon, alt, roll, pitch, yaw = [float(v) for v in f.readline().split()[:6]]
        if geo0 is None:
            geo0 = (lat, lon, alt)
        tx, ty, tz = ecef2enu(*geo2ecef(lat, lon, alt), *geo0)

        # roll is x rotation
        cx = np.cos(roll)
        sx = np.sin(roll)
        Rx = np.array([[1,  0,   0],
                          [0, cx, -sx],
                          [0, sx,  cx]])
        # pitch is y rotation
        cy = np.cos(pitch)
        sy = np.sin(pitch)
        Ry = np.array([[ cy, 0, sy],
                          [  0, 1,  0],
                          [-sy, 0, cy]])
        # yaw is z rotation
        cz = np.cos(yaw)
        sz = np.sin(yaw)
        Rz = np.array([[cz, -sz, 0],
                          [sz,  cz, 0],
                          [ 0,   0, 1]])
        R = Rz @ Ry @ Rx
        R = R.reshape(3, 3)
        t = np.array([tx, ty, tz])
        t = t.reshape(3, 1)
        Rt = np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))
        # transform from IMU to velodyne
        Rt = Rt_imu_to_velo @ Rt
        R = np.array([[ Rt[0,0], Rt[0,1], Rt[0,2]],
                         [ Rt[1,0], Rt[1,1], Rt[1,2]],
                         [ Rt[2,0], Rt[2,1], Rt[2,2]]])
        t = np.array([Rt[0,3], Rt[1,3], Rt[2,3]])
        # convert rotation from oxts to 3dtk and back
        oxts2slam6d = np.array([[ 0,0,1],
                                   [-1,0,0],
                                   [ 0,1,0]])
        slam6d2oxts = np.array([[ 0,-1,0],
                                   [ 0, 0,1],
                                   [ 1, 0,0]])
        # to move a rotation matrix to a different base we multiply it by
        # a change of basis matrix and its inverse
        R = slam6d2oxts @ R @ oxts2slam6d
        R = R.reshape(3, 3)
        # convert translation from oxts to 3dtk
        t = np.array([-t[1]*100, t[2]*100, t[0]*100])
        t = t.reshape(3, 1)
        # combine into transformation matrix
        Rt = np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))
        Rt_column_major_flattened = Rt.flatten(order='F') # Fortran-style
        with open(base+"/3dtk/scan%03d.pose"% currscan, "w") as f:
            rx, ry, rz, tx, ty, tz = py3dtk.Matrix4ToEuler(tuple(Rt_column_major_flattened.flat))
            f.write("%.16f %.16f %.16f\n%.16f %.16f %.16f\n"%(tx, ty, tz, np.rad2deg(rx), np.rad2deg(ry), np.rad2deg(rz)))
        with open(base+"/3dtk/scan%03d.frames"% currscan, "w") as f:
            f.write(" ".join(["%.16f"%v for v in Rt_column_major_flattened.flat]+["2"]))
        allposes.append((tx, ty, tz))
        scanpath = base+"/3dtk/scan%03d.3d"% currscan
        data = np.fromfile(binpath, dtype='<f4, <f4, <f4, <f4')
        with open(scanpath, "w") as outf:
            for x, y, z, r in data:
                outf.write("%s %s %s %s\n"%tuple([x, y, z, r]))
                #x, y, z = -y*100, z*100, x*100
                #outf.write("%s %s %s %s\n"%tuple([py3dtk.utils.float2hex(float(v)) for v in [x, y, z, r]]))
        mask = np.array(Image.open(base + "/image_02/data/%010d.png" % j).convert("1"))
        cdpath = base+"/changedetection/scan%03d.3d"% currscan
        with open(cdpath, "w") as outf:
            for x, y, z, _ in data:
                if x < 0:
                    # we are only interested in points on the front
                    continue
                img = P_rect_02 @ R_rect_00 @ Rt_velo_to_cam @ np.reshape([x, y, z, 1], (4,1))
                img /= img[2,:]
                coord = np.rint(img)
                if 0 <= coord[0] < mask.shape[1] and 0 <= coord[1] < mask.shape[0]:
                    moving = 0
                    if mask[int(coord[1]), int(coord[0])]:
                        moving = 1
                    # convert from velodyne to 3dtk
                    x, y, z = -y*100, z*100, x*100
                    outf.write("%s %s %s %d\n"%tuple([py3dtk.utils.float2hex(float(v)) for v in [x, y, z]]+[moving]))
        currscan += 1
    minx = minz = float("inf")
    maxx = maxz = -float("inf")
    netedges = list()
    # reproducible randomness
    random.seed(0)
    # squared maximum distance 10 m => 1000 cm => sqrt(1000000)
    maxdist2=1000000
    maxneighbors=10

    print('allposes')
    for i, (x1, y1, z1) in enumerate(allposes):
        minx = min(minx, x1)
        minz = min(minz, z1)
        maxx = max(maxx, x1)
        maxz = max(maxz, z1)
        neighbors = list()
        for j, (x2, y2, z2) in enumerate(allposes):
            if i == j:
                continue
            dist2 = (x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2
            if dist2 > maxdist2:
                continue
            neighbors.append((i, j))
        if len(neighbors) > maxneighbors:
            netedges.extend(random.sample(neighbors, maxneighbors))
        else:
            netedges.extend(neighbors)
    
    with open(base+"/3dtk/slam6d.net", "w") as f:
        f.write("%d\n"%len(allposes))
        f.write("%d\n"%len(netedges))
        for v1, v2 in netedges:
            f.write("%d %d\n"%(v1, v2))
    
    # write a good position to look at the scan with top-view into pose.dat
    with open(base+"/3dtk/pose.dat", "w") as f:
        # a good position centers on the bounding box
        # the y value is an arbitrary value smaller than zero or otherwise
        # points would overlap the trajectory
        f.write("%f %f %f\n" % ((minx+maxx)/2, -4000, (minz+maxz)/2))
        f.write("%f %f %f %f %f %f %f %f\n" % (
            # quaternion for top view
            np.sqrt(0.5), np.sqrt(0.5), 0, 0,
            # mouse rotation
            90, 0, 0,
            # field of view
            60
            ))
        # view mode, mouse nav mode and parallel zoom
        # the right exact parallel zoom would require a factor of 0.5 but with
        # a factor of 0.6 we get some free space around the trajectory
        f.write("1 1 %f\n"%(max(maxx-minx, maxz-minz)*0.6))
        # show_points, show_path, show_cameras, pointsize
        f.write("1 1 1 1\n")
        # show_fog, fogDensity, invert
        f.write("0 0.001 1\n")


if __name__ == '__main__':
    main()