# -*-coding:utf-8 -*-
'''
@File    :   viz_pose_pytrans3d.py
@Time    :   2021/10/28 09:37:07
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Test
'''

import numpy as np
import matplotlib.pyplot as plt
from numpy.core.shape_base import hstack
# pip3 install --user pytransform3d
from pytransform3d.rotations import (
    matrix_from_axis_angle, quaternion_from_matrix, quaternion_slerp)
from pytransform3d.trajectories import plot_trajectory

# https://stackoverflow.com/a/31364297
def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# ref: https://github.com/rock-learning/pytransform3d/blob/master/examples/plots/plot_quaternion_slerp.py
# pose_stamped=np.load("./nocam_2021-04-24-11-48-21_all_pose_stamped.npy", allow_pickle=True)
pose_stamped=np.load("./nocam_2021-05-08-11-32-47_all_pose_stamped.npy", allow_pickle=True)
lidar_pose_stamped=np.load("./nocam_2021-05-08-11-32-47_lidar_pose_stamped.npy", allow_pickle=True)
abs_pose_pos = pose_stamped.item().get('position')
print(max(abs_pose_pos))
print(min(abs_pose_pos))
abs_pose_ori = pose_stamped.item().get('orientation')
lidar_abs_pose_pos = lidar_pose_stamped.item().get('position')
lidar_abs_pose_ori = lidar_pose_stamped.item().get('orientation')
T = hstack((abs_pose_pos, abs_pose_ori))
T_slerp = hstack((lidar_abs_pose_pos, lidar_abs_pose_ori))

fig, ax = plt.subplots(figsize=(12, 6))
ax = plot_trajectory(
    P=T[:, :7], show_direction=True, n_frames=20, s=0.8, ax_s=0.7, linewidth=2)

# ax = plot_trajectory(
#     P=T_slerp[:, :7], show_direction=True, n_frames=20, s=0.5, ax_s=0.7, linewidth=1)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 20)
ax.set_zlim(-2, 0)

ax.view_init(elev=90, azim=90)
set_axes_equal(ax)
plt.savefig("test_pose_3d_topview.png", dpi=300) # png, pdf

# save
ax.view_init(elev=40, azim=-50)
set_axes_equal(ax)
plt.savefig("test_pose_3d.png", dpi=300) # png, pdf

plt.show()
