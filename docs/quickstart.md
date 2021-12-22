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

1. Set direcroty in `data/data_path.yaml`

    ```yaml
    bagbase_dir: /hdd/data_qolo/crowd_qolo_recordings/
    outbase_dir: /hdd/data_qolo/crowdbot_data_analysis/
    ```

    - `bagbase_dir` is where rosbags are stored

    - `outbase_dir` is where extracted lidar, qolo_state, algorithm result, and evaluation results are stored, In terms of example, please check [outbase_dir](#outbase_dir)

    ~~Create symbolic link of 0410_shared_control folder (including two rosbags) in current workspace~~

    ~~`cd path/to/crowdbot-evaluation-tools`~~

    ~~`ln -s  /hdd/data_qolo/crowd_qolo_recordings/0410_shared_control/ data/rosbag/0410_shared_control`~~

2. data conversion from original rosbags

    > Before exporting lidar files, please check whether PyKDL is installed successfully. If not, please check [pykdl_installation_guide.md](./pykdl_installation_guide.md)

    - export pointcloud and lidar timestamp

        ```shell
        python3 qolo/gen_lidar_from_rosbags.py --overwrite -f 0410_shared_control
        ```

      - `--overwrite` flag is used to overwrite existing data
      - `--compressed` flag is used to save pointcloud as compressed pcd files (.pcd). If not, save with npy-like format
      - `-f` flag is used to specify data folder

    **Tips**: For following steps, you can easily execute a single shell script as:

    ```shell
    cd sh_scripts
    sh data_export_eval_source_data.sh -e=py38cuda110 -t=0410_shared_control
    ```

    - export qolo status

        ```shell
        python3 qolo/commands2npy.py --overwrite -f 0410_shared_control
        python3 qolo/tfqolo2npy.py --overwrite -f 0410_shared_control
        python3 qolo/twist2npy.py --overwrite -f 0410_shared_control
        python3 qolo/pose2d2npy.py --overwrite -f 0410_shared_control
        ```

3. apply algorithms to extracted data

    ```sh
    python3 qolo/gen_detection_res.py -f 0410_shared_control
    python3 qolo/gen_tracking_res.py -f 0410_shared_control
    ```

4. visualization of current evaluation result

    ```shell
    python3 qolo/gen_viz_img.py -f 0410_shared_control
    python3 qolo/gen_animation.py -f 0410_shared_control
    ```

    The visualization results can be found in `./data/0410_shared_control_processed/media/`

5. evaluate the qolo and crowd data from algorithm result and extracted data

    - default: skip the sequences that have been analyzed

    - overwrite all produced data

        ```shell
        python3 qolo/eval_qolo_path.py --overwrite -f 0410_shared_control
        python3 qolo/eval_crowd.py --overwrite -f 0410_shared_control
        python3 qolo/eval_qolo_ctrl.py --overwrite -f 0410_shared_control
        ```

        - `--overwrite` flag is used to overwrite existing data

    - replot all images

        ```shell
        python qolo/eval_qolo_path.py --replot -f 0410_shared_control
        python qolo/eval_crowd.py --replot -f 0410_shared_control
        python qolo/eval_qolo_ctrl.py --replot -f 0410_shared_control
        ```

        - `--replot` flag is used to replot resulting data

    The visualization results can be found in `./data/0410_shared_control_processed/metrics/`

## outbase_dir

```shell
$0424_mds_processed$ tree -L 2
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
