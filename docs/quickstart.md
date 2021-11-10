# Quickstart

- two rosbag fromshared_control

    ```sh
    ln -s  /hdd/data_qolo/crowd_qolo_recordings/shared_test/ data/rosbag/shared_test
    ```

- data conversion

    ```sh
    python3 qolo/gen_lidar_from_rosbags.py -f shared_test

    python3 qolo/tfqolo2npy.py -f shared_test --overwrite
    python3 qolo/twist2npy.py -f shared_test --overwrite
    python3 qolo/pose2d2npy.py -f shared_test --overwrite
    ```

- algorithms

    ```sh
    python3 qolo/gen_detection_res.py -f shared_test
    python3 qolo/gen_tracking_res.py -f shared_test
    ```

- viz

    ```sh
    python3 qolo/gen_viz_img_o3d.py -f shared_test
    python3 qolo/gen_video.py -f shared_test
    ```

- eval

    ```sh
    python3 qolo/eval_crowd.py --overwrite -f shared_test 
    python3 qolo/eval_qolo.py --overwrite -f shared_test 
    ```

# resulted data structure

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
    ├── qolo_tf
    └── twist
```