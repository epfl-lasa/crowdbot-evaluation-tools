## Dev environment configuration

### Installing additional CUDA 11.0

#### Installing scripts

> Resources can be found in <https://developer.nvidia.com/cuda-11.0-download-archive>

```sh
wget http://developer.download.nvidia.com/compute/cuda/11.0.2/local_installers/cuda_11.0.2_450.51.05_linux.runsudo
sh cuda_11.0*.run
```

#### switch between different versions

> reference: <https://stackoverflow.com/questions/45477133/how-to-change-cuda-version>

```sh
ll /usr/local/cuda
# example: use symbolic link to connect cuda-10.1 or cuda-11.2
sudo ln -sfT /usr/local/cuda-10.1 /usr/local/cuda
sudo ln -sfT /usr/local/cuda-11.0 /usr/local/cuda

# use stat to check connection
stat /usr/local/cuda
```

### Installing Person_MinkUNet & AB3DMOT

#### Installing commands

```sh
git clone --recursive https://github.com/epfl-lasa/crowdbot-evaluation-tools.git
cd crowdbot-evaluation-tools

# install Person_MinkUNet
cd qolo
git clone https://github.com/VisualComputingInstitute/Person_MinkUNet.git
conda create -n torch38_cu110 python=3.8
conda activate py38cuda110
conda activate torch38_cu110

## Person_MinkUNet dependency
# PyTorch==1.7.0
python -m pip install torch==1.7.0+cu110 torchvision==0.8.1+cu110 -f https://download.pytorch.org/whl/torch_stable.html
# torchsparse==1.2.0 (link)
sudo apt-get install libsparsehash-dev
# Attribute Error: install_layout -- https://meejah.ca/blog/pip-install-layout
python -m pip install --upgrade setuptools pip
python -m pip install --upgrade git+https://github.com/mit-han-lab/torchsparse.git@v1.2.0
conda install shapely

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
# modified version in https://github.com/hibetterheyj/AB3DMOT/tree/crowdbot
cd path/to/crowdbot-evaluation-tools
mkdir libs && cd libs
git clone https://github.com/xinshuoweng/Xinshuo_PyToolbox
cd Xinshuo_PyToolbox
python -m pip install -r requirements.txt
cd ../../qolo/AB3DMOT/
python -m pip install -r requirements.txt
```

#### Possible errors

- if missing any header files when compiling torchsparse, you can

  ```
  # fatal error: cusparse.h: No such file or directory: https://github.com/facebookresearch/maskrcnn-benchmark/issues/685#issuecomment-596144324
  sudo apt-get install cuda-cusparse-dev-10-1
  # fatal error: curand.h: No such file or directory
  sudo apt-get install cuda-curand-dev-10-1
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

- **TODO: headless rendering: http://www.open3d.org/docs/latest/tutorial/visualization/headless_rendering.html**

  ```sh
  cmake -DENABLE_HEADLESS_RENDERING=ON \
                  -DBUILD_GUI=OFF \
                  -DBUILD_WEBRTC=OFF \
                  -DUSE_SYSTEM_GLEW=OFF \
                  -DUSE_SYSTEM_GLFW=OFF \
                  ..
  ```

#### resulted folder structure

```shell
$ tree -L 2 -I tmp
.
├── data/
├── docs/
├── example/
├── libs
│   └── Xinshuo_PyToolbox
├── LICENSE
├── notebooks/
├── qolo/
│   ├── AB3DMOT/
│   ├── commands2npy.py
│   ├── crowdbot_data.py
│   ├── eval_crowd.py
│   ├── eval_qolo_ctrl.py
│   ├── eval_qolo_path.py
│   ├── eval_res_plot.py
│   ├── external/
│   ├── gen_animation.py
│   ├── gen_detection_res.py
│   ├── gen_lidar_from_rosbags.py
│   ├── gen_tracking_res.py
│   ├── gen_trk_data.py
│   ├── gen_viz_img.py
│   ├── metric_crowd.py
│   ├── metric_qolo_perf.py
│   ├── notebook_util.py
│   ├── o3d_util.py
│   ├── Person_MinkUNet
│   ├── pose2d2npy.py
│   ├── process_util.py
│   ├── saved_view.pkl
│   ├── tfqolo2npy.py
│   ├── twist2npy.py
│   └── viz_util.py
├── README.md
├── requirements_all.txt
├── requirements_eval.txt
└── sh_scripts
    ├── eval_all.sh
    ├── export_all.sh
    ├── export_eval_all.sh
    ├── export_eval_qolo.sh
    ├── parse_yaml.py
    ├── parse_yml_test.sh
    └── readme.md
```
