## Evaluation

### Configuring rosbag data for data conversion

#### prepare the data and dependency for code

```sh
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

#### run the data conversion code

```sh
# convert the data format from rosbag
cd path/to/crowdbot_tools
python3 qolo/gen_lidar_from_rosbags.py -f nocam_rosbags
python3 qolo/gen_lidar_from_rosbags.py -f 0424_shared_control
python3 qolo/gen_lidar_from_rosbags.py -f 0424_rds_detector
```

- resulted structure in `data/` after data conversion

  ```sh
  data$ tree -L 2
  .
  ├── rosbag
  │   ├── 0424_shared_control -> /hdd/data_qolo/lausanne_2021/24_04_2021/shared_control
  │   └── nocam_rosbags -> /hdd/data_qolo/lausanne_2021/nocam_rosbags/
  ├── 0424_shared_control_processed
  │   └── lidar
  │       ├── 2021-04-24-13-07-54
  │       │   ├── 00000.nby
  │       │   ├── ...
  │       │   └── 01878.nby
  │       └── 2021-04-24-13-11-03
  └── nocam_rosbags_processed
      └── lidar
          ├── nocam_2021-04-24-11-48-21
          ├── nocam_2021-04-24-12-56-59
          ├── nocam_2021-04-24-13-16-58
          └── nocam_2021-05-08-11-32-47
  ```

### Extracting pose_stamped from rosbag

#### run the detection code

```sh
python3 qolo/gen_pose_with_timestamp.py -f nocam_rosbags
python3 qolo/gen_pose_with_timestamp.py -f 0424_shared_control
python3 qolo/gen_pose_with_timestamp.py -f 0424_rds_detector
```

### Running detector with Person-MinkUNet

#### prepare checkpoints

downloads [checkpoints](https://github.com/VisualComputingInstitute/Person_MinkUNet/releases) into `qolo/Person_MinkUNet/models` # for Person_MinkUNet

- Person_MinkUNet/models structure

  ```
  ├── models
  │   ├── ckpt_e40_train.pth
  │   ├── ckpt_e40_train_val.pth
  │   ├── unet_bl_voxel_jrdb_0.05_0.1_train_vel.yaml
  │   └── unet_bl_voxel_jrdb_0.05_0.1_train.yaml
  ```


#### run the detection code

```
python3 qolo/gen_detection_res.py -f 0424_shared_control
python3 qolo/gen_detection_res.py -f nocam_rosbags
python3 qolo/gen_detection_res.py -f 0424_rds_detector
```

### Running tracker with AB3DMOT

- add PYTHONPATH variable (optional)

    As indicated [here](https://github.com/xinshuoweng/AB3DMOT#dependencies), to load the library appropriately, please remember to append PYTHONPATH variable in each terminal or add the following to your ~/.profile

    ```sh
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

#### run the tracking code

```sh
python3 qolo/gen_tracking_res.py -f 0424_shared_control
python3 qolo/gen_tracking_res.py -f nocam_rosbags
python3 qolo/gen_tracking_res.py -f 0424_rds_detector
```

### Visualizing scans, detections, and tracking results

#### viz with Open3D

- running scripts

```
# generate images
python3 qolo/gen_viz_img.py -f nocam_rosbags
python3 qolo/gen_viz_img.py -f 0424_shared_control
python3 qolo/gen_viz_img.py -f 0424_rds_detector
```

- **TODO: headless rendering: http://www.open3d.org/docs/latest/tutorial/visualization/headless_rendering.html **

```sh
cmake -DENABLE_HEADLESS_RENDERING=ON \
                 -DBUILD_GUI=OFF \
                 -DBUILD_WEBRTC=OFF \
                 -DUSE_SYSTEM_GLEW=OFF \
                 -DUSE_SYSTEM_GLFW=OFF \
                 ..
```

#### viz with mayavi

```sh
# generate images
python3 qolo/gen_viz_img.py -f nocam_rosbags
python3 qolo/gen_viz_img.py -f 0424_shared_control
python3 qolo/gen_viz_img.py -f 0424_rds_detector
```

#### video making from image sequence

```sh
# generate video
# TODO: check correctly or not
# ffmpeg -y -r 15 -pattern_type glob -i "tmp/*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p "tmp/frames.mp4"
# ffmpeg -y -r 15 -pattern_type glob -i "viz_imgs/nocam_2021-04-24-11-48-21/*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p "videos/nocam_2021-04-24-11-48-21.mp4"
python3 qolo/gen_animation.py -f nocam_rosbags
python3 qolo/gen_animation.py -f 0424_shared_control
python3 qolo/gen_animation.py -f 0424_rds_detector
```

- resulted structure in `data/` after generating detection results, images, and videos

  ```sh
  data$ tree -L 2
  .
  ├── 0424_shared_control_processed
  │   ├── detections
  │   ├── lidar
  │   ├── tracks
  │   ├── videos
  │   └── viz_imgs
  ├── nocam_rosbags_processed
  │   ├── detections
  │   ├── lidar
  │   ├── tracks
  │   ├── videos
  │   └── viz_imgs
  └── rosbag
      ├── 0424_shared_control -> /hdd/data_qolo/lausanne_2021/24_04_2021/shared_control
      └── nocam_rosbags -> /hdd/data_qolo/lausanne_2021/nocam_rosbags/
  ```
