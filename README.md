# Global Points Cloud

Project to generate a global points cloud from LIDAR data.

## Data

- [KITTI dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php)

## Requirements

- Python 3.7+
- PCL 1.10
- VTK 9.0
- Cython 0.29

## Setup

```bash
$ export BASE_PATH="[YOUR_VELODYNE_FULLPATH]"
$ python3 -m venv venv
$ . venv/bin/activate
```

## Run

```bash
$ python kitti_to_pcl.py
```

## File Structure

- Before running `kitti_to_pcl`:

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

- After running `kitti_to_pcl`:

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
    │       ├── data_pcd
    │       └── data_ply
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