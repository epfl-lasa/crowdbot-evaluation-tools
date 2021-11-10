[toc]

# crowdbot-evaluation-tools

## Basics

### Test environment

- Ubuntu 20.04
- CUDA 11.0
- PyTorch 1.7.0
- torchsparse 1.2.0

### Related repos

> main repo: https://github.com/danjia21/crowdbot_tools
- https://github.com/VisualComputingInstitute/Person_MinkUNet
- https://github.com/mit-han-lab/torchsparse
- https://github.com/xinshuoweng/AB3DMOT/tree/53ae98951d1128bfbffa9cc3b9888d7ffe0212ef
- https://github.com/xinshuoweng/Xinshuo_PyToolbox

## Dev environment configuration

### Installing additional CUDA 11.0

#### Installing scripts

> https://developer.nvidia.com/cuda-11.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=2004&target_type=runfilelocal

```sh
wget http://developer.download.nvidia.com/compute/cuda/11.0.2/local_installers/cuda_11.0.2_450.51.05_linux.runsudo
sh cuda_11.0.2_450.51.05_linux.run
```

#### switch between different versions

> https://stackoverflow.com/questions/45477133/how-to-change-cuda-version

```sh
ll /usr/local/cuda
# use symbolic link to connect cuda-10.1 or cuda-11.2
sudo ln -sfT /usr/local/cuda-10.1 /usr/local/cuda
sudo ln -sfT /usr/local/cuda-11.0 /usr/local/cuda

# use stat to check connection
stat /usr/local/cuda
```

### Installing Person_MinkUNet & AB3DMOT

#### Installing scripts

```sh
git clone --recursive https://github.com/danjia21/crowdbot_tools
cd crowdbot_tools

# install Person_MinkUNet
cd qolo
git clone https://github.com/VisualComputingInstitute/Person_MinkUNet.git
conda create -n torch38_cu110 python=3.8
conda activate py38cuda110
conda activate torch38_cu110

## Person_MinkUNet dependency
# PyTorch==1.7.0
pip3 install torch==1.7.0+cu110 torchvision==0.8.1+cu110 torchaudio==0.7.0 -f https://download.pytorch.org/whl/torch_stable.html
# torchsparse==1.2.0 (link)
sudo apt-get install libsparsehash-dev
# Attribute Error: install_layout -- https://meejah.ca/blog/pip-install-layout
pip3 install --upgrade setuptools pip
pip3 install --upgrade git+https://github.com/mit-han-lab/torchsparse.git@v1.2.0
# ModuleNotFoundError: No module named 'shapely'
conda install shapely
# ModuleNotFoundError: No module named 'mayavi'
# https://github.com/enthought/mayavi
pip install --upgrade --force-reinstall vtk==9.0.1 numpy --user
pip install --upgrade --force-reinstall mayavi pyside2 --user
pip install --upgrade --force-reinstall pyqt5 --user
# warning: not compatible with the dependance of`ros-noetic-pcl-conversions libpcl-dev`
# i.e., `vtk7 libvtk7-dev`

## compile Person_MinkUNet
# install lidar_det project
cd qolo/Person_MinkUNet/
python setup.py develop
# build libraries
cd lib/iou3d
python setup.py develop
cd ../jrdb_det3d_eval
python setup.py develop

# install AB3DMOT
cd path/to/crowdbot_tools
mkdir library && cd library
git clone https://github.com/xinshuoweng/Xinshuo_PyToolbox
cd Xinshuo_PyToolbox
pip3 install -r requirements.txt
cd ../../qolo/AB3DMOT/
pip3 install -r requirements.txt
```

- if missing any header files when compiling torchsparse, you can

  ```
  # fatal error: cusparse.h: No such file or directory: https://github.com/facebookresearch/maskrcnn-benchmark/issues/685#issuecomment-596144324
  sudo apt-get install cuda-cusparse-dev-10-1
  # fatal error: curand.h: No such file or directory
  sudo apt-get install cuda-curand-dev-10-1

- errors when installing opencv-python with 3.8

    ```
    # ERROR: Could not find a version that satisfies the requirement opencv-python==3.4.3.18 (from versions: 3.4.8.29, 3.4.9.31, 3.4.9.33, 3.4.10.35, 3.4.10.37, 3.4.11.39, 3.4.11.41, 3.4.11.43, 3.4.11.45, 3.4.13.47, 3.4.14.51, 3.4.14.53, 3.4.15.55, 4.1.2.30, 4.2.0.32, 4.2.0.34, 4.3.0.36, 4.3.0.38, 4.4.0.40, 4.4.0.42, 4.4.0.44, 4.4.0.46, 4.5.1.48, 4.5.2.52, 4.5.2.54, 4.5.3.56)
    # ERROR: No matching distribution found for opencv-python==3.4.3.18
    ```

    change opencv-python==3.4.3.18 to opencv-python==3.4.8.29 in `qolo/AB3DMOT/requirements.txt`

#### resulted folder structure

```
.
├── data
│   └── rosbag
├── qolo
│   ├── README.md
│   ├── AB3DMOT
│   ├── Person_MinkUNet
│   ├── common.py
│   ├── extract_lidar_from_rosbag.py
│   ├── extract_lidar_from_rosbag_yujie.py
│   ├── generate_detections.py
│   ├── generate_tracks.py
│   └── transformation_utils.py
├── readme_crowdbot_tools.md
├── run.sh
└── Xinshuo_PyToolbox
```

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

### Extracting source data from rosbag

#### run the extraction code

- tfqolo2npy

```sh
python3 qolo/tfqolo2npy.py -f nocam_rosbags

# with --overwrite flag to regenerate
python3 qolo/tfqolo2npy.py -f nocam_rosbags --overwrite
```

- twist2npy

```sh
python3 qolo/twist2npy.py -f nocam_rosbags

# with --overwrite flag to regenerate
python3 qolo/twist2npy.py -f nocam_rosbags --overwrite
```

- pose2d2npy

```sh
python3 qolo/pose2d2npy.py -f nocam_rosbags

# with --overwrite flag to regenerate
python3 qolo/pose2d2npy.py -f nocam_rosbags --overwrite
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

```sh
python3 qolo/gen_detection_res.py -f nocam_rosbags
python3 qolo/gen_detection_res.py -f 0424_shared_control
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

  ```sh
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

  **solution**: `pip install --upgrade --force-reinstall numba==0.53.1`
  

#### run the tracking code

```sh
python3 qolo/gen_tracking_res.py -f nocam_rosbags
python3 qolo/gen_tracking_res.py -f 0424_shared_control
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

### Evaluating crowd density and min. dist. from detecion/tracking results

#### run the crowd evaluation code

```sh
python3 qolo/eval_crowd.py -f nocam_rosbags
python3 qolo/eval_crowd.py -f 0424_shared_control
python3 qolo/eval_crowd.py -f 0424_rds_detector

python3 qolo/eval_crowd.py -f nocam_rosbags --overwrite
python3 qolo/eval_crowd.py -f nocam_rosbags --replot
```

results will be saved into `metrics/` subfolder

#### run the crowd evaluation code

```sh
python3 qolo/eval_qolo.py -f nocam_rosbags
python3 qolo/eval_qolo.py -f 0424_shared_control
python3 qolo/eval_qolo.py -f 0424_rds_detector

python3 qolo/eval_qolo.py -f nocam_rosbags --overwrite
python3 qolo/eval_qolo.py -f nocam_rosbags --replot
```

results will be saved into `metrics/` subfolder

## TODO

### Main

- [ ] Implement test and style-check with LASA python guide

- [x] Crowd evaluation code (1104)
  - [x] compute crowd density
  - [x] compute min. dist. from robot

- [ ] reimplement viz with open3d
  - [x] basic viz  (1026)
  - [x] display within dist (1026)
  - [x] extract robot pose in lidar frame (1027)
  - [x] add robot pose into each frame viz (1028)
  - [ ] headless image rendering

- [ ] visualize in the world frame

  - [x] extract from bag (1026)
  - [x] store as npy (1026)

    `tf_qolo_world` `tf_qolo`

    ref: https://github.com/epfl-lasa/qolo-evaluation/blob/main/scripts/live_trajectory_plot.py

    - timestamp of tf
    - timestamp of robot

  - [x] organize documentation (1028)

- [ ] other topic data processing

    - [ ] synchronize all data
    - [x] visual odom -> robot trajectory (1026)
    - [x] pose 3d viz (1027)
    - [ ] robot velocity
    - [ ] compact data storage, reference: other 3D detection dataset!
    - [ ] control input (X and U) amd its derivative

- [ ] visualize bbox. traj, velo in rviz (orientation)

    ref: spencer viz: https://github.com/spencer-project/spencer_people_tracking/tree/master/visualization

- [ ] deface the rosbag data before open=source the dataset

### Others

- [ ] refactor AllFrames class

  - [x] refactor and rename as "viz_util.py"
  - [x] image generation (1026)
  - [x] video generation (1026)
  - [x] pose extraction (1027)
  - [ ] detection
  - [ ] tracking

- [ ] consider using mutliprocessing when generating images

- [ ] consider using [ffmpeg-python](https://github.com/kkroening/ffmpeg-python) to generate videos

    ```
    import os
    os.system("ffmpeg -y -r 15 -pattern_type glob -i "tmp/*.png" -c:v libx264 -vf fps=30 -pix_fmt yuv420p "tmp/frames.mp4"")
    ```

