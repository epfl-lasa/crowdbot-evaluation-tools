# Quickstart

- Minimal environment for evaluation

    ```shell
    conda create -n crowdbot_eval python=3.8
    conda activate crowdbot_eval
    ## or `source activate crowdbot_eval` in ubuntu
    python -m pip install -U pip
    sudo apt-get install ros-$ROS_DISTRO-ros-numpy
    sudo apt-get install ros-$ROS_DISTRO-tf2-sensor-msgs

    ## install using requirements (recommended)
    pip3 install -r requirements_eval.txt

    ## install using scripts (optional)
    python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
    python -m pip install --user rospkg pycryptodomex python-gnupg
    # https://github.com/moble/quaternion
    python -m pip install --upgrade --force-reinstall numba==0.53.1
    python -m pip install --upgrade --force-reinstall numpy-quaternion
    ```

## Simplified starting commands

1. Create symbolic link of shared_test folder (including two rosbags) in current workspace

    ```shell
    cd path/to/crowdbot-evaluation-tools
    ln -s  /hdd/data_qolo/crowd_qolo_recordings/shared_test/ data/rosbag/shared_test
    ```

2. data conversion from original rosbags

    > Before exporting lidar files, please check whether PyKDL is installed successfully. If not, please check [pykdl_installation_guide.md](./pykdl_installation_guide.md)

    - export pointcloud and lidar timestamp

        ```shell
        python3 qolo/gen_lidar_from_rosbags.py --overwrite -f shared_test
        ```

      - `--overwrite` flag is used to overwrite existing data
      - `--compressed` flag is used to save pointcloud as compressed pcd files (.pcd). If not, save with npy-like format
      - `-f` flag is used to specify data folder

    - export qolo status

        ```shell
        python3 qolo/tfqolo2npy.py --overwrite -f shared_test
        python3 qolo/twist2npy.py --overwrite -f shared_test
        python3 qolo/pose2d2npy.py --overwrite -f shared_test
        ```

3. apply algorithms to extracted data

    ```sh
    python3 qolo/gen_detection_res.py -f shared_test
    python3 qolo/gen_tracking_res.py -f shared_test
    ```

4. visualization of current evaluation result

    ```shell
    python3 qolo/gen_viz_img_o3d.py -f shared_test
    python3 qolo/gen_video.py -f shared_test
    ```

    The visualization results can be found in `./data/shared_test_processed/media/`

5. evaluate the qolo and crowd data from algorithm result and extracted data

    ```shell
    python3 qolo/eval_crowd.py --overwrite -f shared_test
    python3 qolo/eval_qolo.py --overwrite -f shared_test
    ```

    The visualization results can be found in `./data/shared_test_processed/metrics/`

## Resulted data structure

```shell
shared_test_processed$ tree -L 2
.
├── alg_res
│   ├── detections
│   └── tracks
├── lidars
│   ├── 2021-04-10-12-36-29
│   ├── 2021-04-10-12-36-29_stamped.npy
│   ├── 2021-04-10-12-38-25
│   └── 2021-04-10-12-38-25_stamped.npy
├── media
│   ├── videos
│   └── viz_imgs
├── metrics
│   ├── 2021-04-10-12-36-29_crowd_density.png
│   ├── 2021-04-10-12-36-29_crowd_eval.npy
│   ├── 2021-04-10-12-36-29_min_dist.png
│   ├── 2021-04-10-12-36-29_path.png
│   ├── 2021-04-10-12-36-29_qolo_eval.npy
│   ├── 2021-04-10-12-36-29_twist_acc_jerk.png
│   ├── 2021-04-10-12-38-25_crowd_density.png
│   ├── 2021-04-10-12-38-25_crowd_eval.npy
│   ├── 2021-04-10-12-38-25_min_dist.png
│   ├── 2021-04-10-12-38-25_path.png
│   ├── 2021-04-10-12-38-25_qolo_eval.npy
│   └── 2021-04-10-12-38-25_twist_acc_jerk.png
└── source_data
    ├── acc
    ├── pose2d
    ├── tf_qolo
    └── twist
```
