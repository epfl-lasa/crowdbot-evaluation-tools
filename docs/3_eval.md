## Evaluation

### Configuring rosbag data for data conversion

#### prepare the data and dependency for code

```shell
# download the rosbag and put them under rosbag
cd path/to/crowdbot_tools/data
mkdir rosbag && cd rosbag
# (recommended) create symbolic links of rosbag folder instead of copying data
# ref: https://stackoverflow.com/questions/9587445/how-to-create-a-link-to-a-directory
ln -s /hdd/data_qolo/lausanne_2021/nocam_rosbags/
ln -s  /hdd/data_qolo/lausanne_2021/24_04_2021/shared_control 0424_shared_control
ln -s  /hdd/data_qolo/lausanne_2021/24_04_2021/RDS/detector 0424_rds_detector

# install tf2-sensor-msgs to get transform functions
sudo apt-get install ros-$ROS_DISTRO-tf2-sensor-msgs
```

#### Extracting source data from rosbag

#### Pointcloud & LiDAR Timestamp

```shell
python3 qolo/gen_lidar_from_rosbags.py -f 0424_shared_control
```

- input:
- output:
- detailed description:

#### tf_qolo information

```shell
python3 qolo/tfqolo2npy.py -f 0424_shared_control
```

- input:
- output:
- detailed description:
  TODO: include the method to filter the signal

#### twist information

```shell
python3 qolo/twist2npy.py -f 0424_shared_control
```

- input:
- output:
- detailed description:

#### pose information

```shell
python3 qolo/pose2d2npy.py -f 0424_shared_control
```

- input:
- output:
- detailed description:

#### command information

```shell
python3 qolo/commands2npy.py -f 0424_shared_control
```

- input:
- output:
- detailed description:

### Running detector with Person-MinkUNet

1. prepare checkpoints: downloads [checkpoints](https://github.com/VisualComputingInstitute/Person_MinkUNet/releases) into `qolo/Person_MinkUNet/models` for  erson_MinkUNet

  - Person_MinkUNet/models structure

    ```
    ├── models
    │   ├── ckpt_e40_train.pth
    │   ├── ckpt_e40_train_val.pth
    │   ├── unet_bl_voxel_jrdb_0.05_0.1_train_vel.yaml
    │   └── unet_bl_voxel_jrdb_0.05_0.1_train.yaml
    ```


2. run the detection code

  ```shell
  python3 qolo/gen_detection_res.py -f 0424_shared_control
  ```

- input:
- output:
- detailed description:

### Running tracker with AB3DMOT

1. add `PYTHONPATH` variable (optional)

    As indicated [here](https://github.com/xinshuoweng/AB3DMOT#dependencies), to load the library appropriately, please remember to append PYTHONPATH variable in each terminal or add the following to your ~/.profile

    ```shell
    # conda activate torch38_cu110
    export PYTHONPATH="${PYTHONPATH}:${PWD}/qolo/AB3DMOT"
    echo $PYTHONPATH
    ```

    Inspired by these two stackoverflow posts, [[In Python script, how do I set PYTHONPATH?](https://stackoverflow.com/a/3108307)] and [[How do I get the full path of the current file's directory?](https://stackoverflow.com/a/3430395)], we can append the AB3DMOT library in python scripts as follows:

    ```python
    import os
    import sys
    # export PYTHONPATH="${PYTHONPATH}:${PWD}/qolo/AB3DMOT"
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "AB3DMOT"))
    ```

2. run the tracking code

  ```shell
  python3 qolo/gen_tracking_res.py -f 0424_shared_control
  ```

- input:
- output:
- detailed description:

### Visualizing scans, detections, and tracking results

#### viz with Open3D

- running scripts

```shell
python3 qolo/gen_viz_img.py -f 0424_shared_control
```

#### video & gifs generated from image sequence

```shell
python3 qolo/gen_animation.py -f 0424_shared_control
```

### Metrics & crowd characteristics

#### Path efficiency-related metrics

```shell
python3 qolo/eval_qolo_path.py -f 0424_shared_control
```

- input:
- output:
- parameters: (in table way)
- detailed description:

#### Qolo control performance-related metrics

```shell
python3 qolo/eval_qolo_ctrl.py -f 0424_shared_control
```

- input:
- output:
- parameters: (in table way)
- detailed description:

#### Crowd-related metrics

```shell
python3 qolo/eval_crowd.py -f 0424_shared_control
```

- input:
- output:
- parameters: (in table way)
- detailed description:

### Other utility functions

###

### Dataset structure

> taking `0424_mds_processed/` as an example

```shellell
$ tree -L 2
.
├── alg_res
│   ├── detections
│   └── tracks
├── lidars
│   ├── 2021-04-24-12-04-04
│   ├── 2021-04-24-12-07-57
│   ├── 2021-04-24-12-10-45
│   ├── 2021-04-24-12-54-04
│   ├── 2021-04-24-12-56-59
│   └── 2021-04-24-13-03-39
├── metrics
│   ├── 2021-04-24-12-04-04
│   ├── 2021-04-24-12-07-57
│   ├── 2021-04-24-12-10-45
│   ├── 2021-04-24-12-54-04
│   ├── 2021-04-24-12-56-59
│   └── 2021-04-24-13-03-39
└── source_data
    ├── commands
    ├── pose2d
    ├── tf_qolo
    ├── timestamp
    └── twist
```
