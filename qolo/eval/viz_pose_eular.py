# -*-coding:utf-8 -*-
'''
@File    :   viz_pose_eular.py
@Time    :   2021/10/28 09:38:46
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Test
'''

import numpy as np
from numpy.core.shape_base import hstack

import matplotlib.pyplot as plt
import seaborn as sns
sns.set_theme()

import scipy
from scipy.spatial.transform import Rotation as R
# https://stackoverflow.com/questions/11887762/how-do-i-compare-version-numbers-in-python/21065570
from packaging import version

pose_stamped=np.load("./nocam_2021-05-08-11-32-47_all_pose_stamped.npy", allow_pickle=True)
lidar_pose_stamped=np.load("./nocam_2021-05-08-11-32-47_lidar_pose_stamped.npy", allow_pickle=True)
# abs_pose_pos = pose_stamped.item().get('position')
abs_pose_ori = pose_stamped.item().get('orientation')
# lidar_abs_pose_pos = lidar_pose_stamped.item().get('position')
lidar_abs_pose_ori = lidar_pose_stamped.item().get('orientation')

def toTransMat(trans, rot_quat):
    # [-0.015268456878646017, -2.564143964217997e-05, 0.20917331914570264]
    # [-0.0004825070209335536, 0.011764172464609146, -0.00042640912579372525, 0.9999306201934814]
    scipy_rot = R.from_quat(rot_quat)
    # https://github.com/scipy/scipy/issues/11685
    if version.parse(scipy.__version__) >= version.parse("1.4.0"):
        rot_mat = scipy_rot.as_matrix()
    else:
        rot_mat = scipy_rot.as_dcm()
    trans_mat = np.zeros((4,4))
    trans_mat[:3, :3] = rot_mat
    trans_mat[3, 3] = 1
    trans_mat[:3, 3:] = np.reshape(trans, (3,1))
    return trans_mat

def toEularZYX(rot_quat):
    # [-0.0004825070209335536, 0.011764172464609146, -0.00042640912579372525, 0.9999306201934814]
    scipy_rot = R.from_quat(rot_quat)
    rot_zyx = scipy_rot.as_euler('zyx', degrees=True)
    return rot_zyx

rot_zyx = toEularZYX(abs_pose_ori)
# print(rot_zyx[:5,:])

alpha_yaw = rot_zyx[:,0] # Z
beta_pitch = rot_zyx[:,1] # Y
gamma_roll = rot_zyx[:,2] # X 
ts = pose_stamped.item().get('timestamp')
ts_np = np.asarray(ts) - min(ts)

fig, ax = plt.subplots(figsize=(12, 4))
yaw, = ax.plot(ts_np, alpha_yaw, color='orange', linewidth=3, label='yaw', alpha=0.8)
pitch, = ax.plot(ts_np, beta_pitch, color='grey', linewidth=3, label='pitch', alpha=0.8)
roll, = ax.plot(ts_np, gamma_roll, color='skyblue', linewidth=3, label='roll', alpha=0.8)
ax.legend(handles=[yaw, pitch, roll])

fig.tight_layout()

# save
plt.savefig("test_pose_euler.png", dpi=300) # png, pdf
plt.show()
