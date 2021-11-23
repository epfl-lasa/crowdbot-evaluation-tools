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

- possible error caused by incomplete installation of numba

  ```
  $ python -c 'import numba'
  Traceback (most recent call last):
    File "<string>", line 1, in <module>
    File "/home/crowdbot/.local/lib/python3.8/site-packages/numba/__init__.py", line 25, in <module>
      from .decorators import autojit, cfunc, generated_jit, jit, njit, stencil
    File "/home/crowdbot/.local/lib/python3.8/site-packages/numba/decorators.py", line 12, in <module>
      from .targets import registry
    File "/home/crowdbot/.local/lib/python3.8/site-packages/numba/targets/registry.py", line 5, in <module>
      from . import cpu
    File "/home/crowdbot/.local/lib/python3.8/site-packages/numba/targets/cpu.py", line 9, in <module>
      from numba import _dynfunc, config
  ImportError: /home/crowdbot/.local/lib/python3.8/site-packages/numba/_dynfunc.cpython-38-x86_64-linux-gnu.so: undefined symbol: _PyObject_GC_UNTRACK
  ```

  **solution**: `python -m pip install --upgrade --force-reinstall numba==0.53.1`


#### run the tracking code

```sh
python3 qolo/gen_tracking_res.py -f 0424_shared_control
python3 qolo/gen_tracking_res.py -f nocam_rosbags
python3 qolo/gen_tracking_res.py -f 0424_rds_detector
```

### Visualizing scans, detections, and tracking results

#### installing Open3D

> http://www.open3d.org/docs/release/getting_started.html
>
> http://www.open3d.org/docs/release/compilation.html


```sh
## METHOD1: install from conda directly
conda install -c open3d-admin -c conda-forge open3d


## METHOD2: compile and install
git clone --recursive https://github.com/intel-isl/Open3D

# You can also update the submodule manually
git submodule update --init --recursive

# update cmake (use following command to check whether version > 3.18)
cmake --version
# https://blog.csdn.net/i6101206007/article/details/113113633#21_snap__26
# sudo snap install cmake --classic
# sudo snap remove cmake
sudo apt install build-essential libssl-dev
wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz
tar -zxvf cmake-3.20.0.tar.gz
cd cmake-3.20.0
./bootstrap
make
sudo make install
export PATH=/path/to/cmake-3.20.0/bin:$PATH

# install deps
sudo apt install xorg-dev libglu1-mesa-dev python3-dev \
libsdl2-dev libc++-7-dev libc++abi-7-dev ninja-build libxi-dev \
libtbb-dev libosmesa6-dev libudev-dev autoconf libtool

# config
mkdir installed
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../installed \
      -DPYTHON_EXECUTABLE=/home/crowdbot/miniconda3/envs/py38cuda110/bin/python \
      ..
make -j$(nproc)
make install

# conda install conda-build
make conda-package

python -c "import open3d"
```

- other dependancy

```
conda install -c conda-forge scipy==1.4.0
# pip install scipy==1.4.0 --user
# validate by `python -c "import scipy; print(scipy.__version__)"`
```

#### viz with Open3D

- running scripts

```
# generate images
python3 qolo/gen_viz_img_o3d.py -f nocam_rosbags
python3 qolo/gen_viz_img_o3d.py -f 0424_shared_control
python3 qolo/gen_viz_img_o3d.py -f 0424_rds_detector
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
python3 qolo/gen_video.py -f nocam_rosbags
python3 qolo/gen_video.py -f 0424_shared_control
python3 qolo/gen_video.py -f 0424_rds_detector
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
