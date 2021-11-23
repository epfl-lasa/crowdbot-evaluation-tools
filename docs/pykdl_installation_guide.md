# PyKDL installation

## TODO

- make it to be a install_pykdl.sh

## Errors

```shell
$ python -c "import PyKDL"
Traceback (most recent call last):
  File "<string>", line 1, in <module>
ModuleNotFoundError: No module named 'PyKDL'
```

## Guide & Problems

- <https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/INSTALL.md>
- Problems: existing method will only lead to installation with default Python

  ```shell
  $ locate PyKDL.so
  /usr/local/lib/python3/dist-packages/PyKDL.so
  ```

## Modified guide & Example

> use `test` as targeted conda environment

1. check env location

   ```shell
   $ conda activate test # or source activate test
   $ python -c "import site; print(''.join(site.getsitepackages()))"
   /home/he/miniconda3/envs/test/lib/python3.8/site-packages
   ```

   As a result, the targeted package installation path: `/home/he/miniconda3/envs/test/lib/python3.8/site-packages`

2. Follow the [guide](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/INSTALL.md#without-catkin) until step 5

   1. Clone the repository where you want
   2. Initialize the PyBind11 submodule: `git submodule update --init`
   3. Follow the mandatory instruction to compile the C++ library from [orocos_kdl/INSTALL.md](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md#without-catkin)
   4. Create a new build folder (it is always better not to build in the source folder): `mkdir build`
   5. Go to the build folder `cd build`

   ```shell
   # step1
   git clone https://github.com/hibetterheyj/orocos_kinematics_dynamics.git
   # step2
   cd orocos_kinematics_dynamics
   git submodule update --init
   # step3
   sudo apt-get update
   sudo apt-get install libeigen3-dev libcppunit-dev
   sudo apt-get install python3-psutil python3-future
   # step4&5
   cd python_orocos_kdl
   mkdir build
   cd build
   ```

3. Execute cmake with specific python environment

     - `DPYTHON_EXECUTABLE=/home/he/miniconda3/envs/test/bin/python`

     - `DPYTHON_INCLUDE_DIR=/home/he/miniconda3/envs/test/include/python3.8`

     - `DCMAKE_INSTALL_PREFIX=/home/he/miniconda3/envs/test`

   ```shell
   $ pip install empy
   $ cmake build \
   -DPYTHON_EXECUTABLE=/home/he/miniconda3/envs/test/bin/python3.8 \
   -DPYTHON_INCLUDE_DIR=/home/he/miniconda3/envs/test/include/python3.8 \
   -DCMAKE_INSTALL_PREFIX=/home/he/miniconda3/envs/test \
   -ROS_PYTHON_VERSION=3 \
   ..
   $ make
   $ sudo make install
   ```

4. Finals

   - Add `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib` into `.bashrc`

   - Execute ldconfig: `sudo ldconfig`

   - check `PyKDL` installation

     ```shell
     $ python -c "import PyKDL; print(PyKDL.Vector(1,2,3))"
     [           1,           2,           3]
     ```


