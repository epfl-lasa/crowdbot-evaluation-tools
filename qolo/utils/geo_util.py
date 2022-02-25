#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   geo_util.py
@Date created  :   2022/02/25
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides utility functions to perform coordinate transformation,
angle description conversion, and etc.
"""
# =============================================================================

import numpy as np
from scipy.spatial.transform import Rotation as R


def get_pc_tranform(pc, pos, quat):
    scipy_rot = R.from_quat(quat)
    rot_mat = scipy_rot.as_matrix()  # 3x3
    rot_pc = np.matmul(rot_mat, pc.T)  #  (3x3) (3xn)
    return rot_pc.T + pos


def yaw2quat(yaw, base_quat=None):
    """convert theta in to quaternion format and transfer to global coordinate"""
    rot_euler = [yaw, 0, 0]
    abs_rot = R.from_euler('zyx', rot_euler)
    if base_quat is not None:
        base_rot = R.from_quat([base_quat])
        abs_rot = base_rot.reduce(left=abs_rot)
    return abs_rot.as_quat()


def quat2yaw(quat):
    scipy_rot = R.from_quat(quat)
    rot_zyx = scipy_rot.as_euler('zyx')
    return rot_zyx[0]
