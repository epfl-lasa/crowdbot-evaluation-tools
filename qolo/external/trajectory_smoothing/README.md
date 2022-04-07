# trajectory-smoothing-ros

## Note for [crowdbot-evaluation-tools](https://github.com/epfl-lasa/crowdbot-evaluation-tools)

> Note: we mainly use this package as static python library

### Installation

- scipy
- numpy
- mpl_toolkits
- sklearn
- csaps

```shell
python -m pip install numpy scipy csaps scikit-learn
```

---

This package integrates trajectory smoothing methods in the ROS framework.

## Description

The input is a list of 3D points which correspond to a movement with initial and final velocity equal to zero (reaching motion).
The functionality of the package is to use the selected smoothing function (Bezier Curves or Smoothing Splines) in order
to extrat a smooth trajectory from the input 3D points. In addition, an empirical statistical method is used based on the
profiles of a reaching motion in order to excude the points that are concentrated in the beginning and ending of the input movement
which do not offer any usefull information about the actual trajectory.

## Implementation

In order to implement the smoothing function this repo is using soucre based on the following repos:
1) Bezier Curves smoothing: https://github.com/Hrisi/Python---Spline-curves
2) Smoothing splines: a) Initial off-ROS implementaton (in Python3): https://github.com/espdev/csaps
                      b) Sklearn-based implementation (+ Polunomial smoothing): https://github.com/madrury/basis-expansions

The above above smoothing fuctions are offered as a ros-service by the smooth_server.py script (server). Default option is the
Bezier Curve based smoothing.

## Test cases

This package was tested using as input 3D points that represent reaching motions of the human wrist inside the workspace of a UR3
robotic arm which where tracked by a RGB-D camera using Openpose as part of the following pipeline (currently not integrated):

https://github.com/Roboskel-Manipulation/openpose_utils

