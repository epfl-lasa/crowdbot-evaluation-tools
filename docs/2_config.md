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
  ```

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
