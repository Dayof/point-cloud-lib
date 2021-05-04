# Point Cloud Registration

Method to perform point cloud registration with reduce total points length.

## Data

- [KITTI dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php)

## Requirements

- GCC 9.3.0+
- Python 3.7+
- PCL 1.10
- VTK 9.0

## Install and Run (CPP)

```bash
$ cd src/core_cpp
$ make mkdirs
$ mkdir build;cd build
$ cmake ..
$ make
$ ./pc_encoder
```

## Install and Run (Python)

```bash
$ python3 -m venv venv
$ . venv/bin/activate
$ pip install -r requirements.txt
$ export BASE_DIR="[YOUR_BASE_PATH]"  # e.g. /home/user/data/2011_09_26_drive_0001
$ cd src/core_py/oxt2pose
$ python main.py
```

## Expected File Structure on Kitti

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
