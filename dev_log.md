# DEV Logs

## Plan

- Fix pose integration bug in visualization code
- Store all the lidars, detections, trackings in compact format (such as csv, npy, tar.gz, etc.)
    should refer to kitti or jrdb dataset
- Implement test and style-check with LASA python guide
- Deployment on Jeston AGX

## Week4

- THu, 2021/11/04

    - reorganize the rosbag folder into three main catagories: RDS, MDS, shared control
    - continue integrate the evalCrowdDensity into pipeline => `qolo/gen_crowd_eval.py` can be used for evaluate the min. dist. from qolo and crowd density within 10m of qolo and save corresponding images

- Tue, 2021/11/02

    - test the evalMetricCrowdDensity.py code in `qolo-evaluation/`, countered and fixed a bug related to the changed API in numpy package. The original code is based on bag2Npy.py to extract the online detection of DR-SPAAM-Detector while our implementation is based on offline detection using Person_MinkUNet
    - implement the evalMetricCrowdDensity in my current framework

## Week3

- Thu, 2021/10/28

    - fix bugs in `gen_viz_img_o3d.py` and `o3d_viz_util.py`
    - add `eval/` folder to visualize the pose in 3d with [pytransform3d](https://github.com/rock-learning/pytransform3d) for `o3d_viz_util.py` debugging
    - push all the modified code into crowdbot-evaluation-tools by logic

        ```sh
        $ git log --pretty=format:"%h%x09%an%x09%ad%x09%s"
        67b191c hibetterheyj    Thu Oct 28 16:34:57 2021 +0200  update README
        ba07ee8 hibetterheyj    Thu Oct 28 16:26:58 2021 +0200  rewrite viz_img code with open3d
        f119f2c hibetterheyj    Thu Oct 28 16:26:06 2021 +0200  add pose extraction & interpolation code
        5e5bf9f hibetterheyj    Thu Oct 28 16:24:48 2021 +0200  update modified code based on original toolkit & finish corresponding documentation
        98874a6 hibetterheyj    Thu Oct 28 16:21:30 2021 +0200  update pose eval code
        db29ef6 hibetterheyj    Thu Oct 28 16:21:02 2021 +0200  update docs
        a2e3801 hibetterheyj    Wed Oct 27 16:16:03 2021 +0200  update submodules
        2a1e9d4 hibetterheyj    Wed Oct 27 16:03:46 2021 +0200  update docs folder
        e011a40 hibetterheyj    Wed Oct 27 15:38:41 2021 +0200  archive crowdbot_tools folder
        ca4e2b3 Diego F. Paez G Tue Oct 26 13:30:52 2021 +0200  Initial commit
        ```

- Wed, 2021/10/27

    - organize the documentation
    - finish basic viz (`gen_viz_img_o3d.py` and `o3d_viz_util.py`) in the world frame by adding robot pose

- Thu, 2021/10/26

    - got stuck in mayavi dependency (needed pyqt, vtk, or pyside as backend), easy to conflict with other package
    - realize basic visualization with Open3D
    - add `gen_pose_with_timestamp.py` to extract tf_qolo to tf_qolo_world pose_stamped info and try to interpolate poses in lidar from with linear interpolation to translation and slerp interpolation to quaternion with scipy
    - add additional subfolder in data called pose_stamped to store the pose data corresponding to each rosbag. All pose_stamped is stored in `.npy` format

## Week2

- Wed, 2021/10/20

    - try to run all the code from https://github.com/danjia21/crowdbot_tools and update development docs
    - rewrite all the code with `argparse` lib
    - create the data folder and mount the rosbag data using `ln -s` to avoid copying duplicate data
    - establish the processed data for each catagory of data and include subfolders such as lidars, detections, trackings, viz_imgs, and videos

- Tue, 2021/10/19

    - install CUDA11.0 on crowdbot@samurai so that we can successfully configure all the dependencies, the full documentation can be shown on `docs/`


## Week1

Mainly explore the rosbag and try to configure the environment but encountering conflicts and failure due to CUDA
