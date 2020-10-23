# Point Cloud Lib for Motion Detection

Detect object's motion on point clouds captured from LIDAR.

## Data

- [KITTI dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php)

## Requirements

- Python 3.7+
- PCL 1.10
- VTK 9.0
- Cython 0.29

## Setup

```bash
$ export BASE_PATH="[YOUR_BASE_PATH]"  # e.g. /home/user/data/2011_09_26/2011_09_26_drive_0001_extract
$ python3 -m venv venv
$ . venv/bin/activate
```

## Run

```bash
$ python kitti2ply2pcd.py  # transform all kitti format to pcd data format
$ python kitti_struct.py  # get lat and lon from timestamps from velodyne's data
```

## File Structure

- Before running `kitti2ply2pcd`:

```bash
└── [DATE]
    ├── [DATE]_drive_0001_extract
    │   ├── image_00
    │   │   └── data
    │   ├── image_01
    │   │   └── data
    │   ├── image_02
    │   │   └── data
    │   ├── image_03
    │   │   └── data
    │   ├── oxts
    │   │   └── data
    │   └── velodyne_points
    │       ├── data
    └── [DATE]_drive_0001_sync
        ├── image_00
        │   └── data
        ├── image_01
        │   └── data
        ├── image_02
        │   └── data
        ├── image_03
        │   └── data
        ├── oxts
        │   └── data
        └── velodyne_points
            └── data
```

- After running `kitti2ply2pcd`:

```bash
└── [DATE]
    ├── [DATE]_drive_0001_extract
    │   ├── vtx.txt  # total vertexes from velodyne
    │   ├── image_00
    │   │   └── data
    │   ├── image_01
    │   │   └── data
    │   ├── image_02
    │   │   └── data
    │   ├── image_03
    │   │   └── data
    │   ├── oxts
    │   │   └── data
    │   └── velodyne_points
    │       ├── data  # original data points
    │       ├── data_pcd  # kitti data in pcd format 
    │       └── data_ply  # kitti data in ply format
    └── [DATE]_drive_0001_sync
        ├── image_00
        │   └── data
        ├── image_01
        │   └── data
        ├── image_02
        │   └── data
        ├── image_03
        │   └── data
        ├── oxts
        │   └── data
        └── velodyne_points
            └── data
```