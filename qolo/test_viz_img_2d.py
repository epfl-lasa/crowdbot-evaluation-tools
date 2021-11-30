#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   test_viz_img_2d.py
@Date created  :   2021/11/30
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides ...
"""
# =============================================================================
"""
TODO:
1.
"""
# =============================================================================

import os
import sys

import pandas as pd
import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from crowdbot_data import CrowdBotData, CrowdBotDatabase
from viz_util import (
    boxes3d_to_corners3d_lidar,
    filter_detection_tracking_res,
    filter_pointcloud_distance,
)

#%%
import inspect

# derived from https://stackoverflow.com/a/18425523/7961693
def retrieve_name(var):
    callers_local_vars = inspect.currentframe().f_back.f_locals.items()
    return [var_name for var_name, var_val in callers_local_vars if var_val is var]


def mod_retrieve_name(var):
    callers_local_vars = inspect.currentframe().f_back.f_back.f_locals.items()
    return [var_name for var_name, var_val in callers_local_vars if var_val is var]


def plen(array):
    print("Length of {}: {}".format(mod_retrieve_name(array)[0], len(array)))


def pshape(array):
    print("Shape of {}: {}".format(mod_retrieve_name(array)[0], array.shape))


#%% Variables

colors = {
    'Car': 'b',
    'Tram': 'r',
    'Cyclist': 'g',
    'Van': 'c',
    'Truck': 'm',
    'Pedestrian': 'y',
    'Sitter': 'k',
}

gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

axes_limits = [
    [-20, 80],  # X axis range
    [-20, 20],  # Y axis range
    [-3, 10],  # Z axis range
]

#%% plotting function


def draw_box(pyplot_axis, vertices, axes=[0, 1, 2], color='black'):
    """
    Draws a bounding 3D box in a pyplot axis.

    Parameters
    ----------
    pyplot_axis : Pyplot axis to draw in.
    vertices    : Array 8 box vertices containing x, y, z coordinates.
    axes        : Axes to use. Defaults to `[0, 1, 2]`, e.g. x, y and z axes.
    color       : Drawing color. Defaults to `black`.
    """
    vertices = vertices[axes, :]
    # fmt: off
    connections = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Lower plane parallel to Z=0 plane
        [4, 5], [5, 6], [6, 7], [7, 4],  # Upper plane parallel to Z=0 plane
        [0, 4], [1, 5], [2, 6], [3, 7]  # Connections between upper and lower planes
    ]
    # fmt: on
    for connection in connections:
        pyplot_axis.plot(*vertices[:, connection], c=color, lw=0.5)


axes_str = ['X', 'Y', 'Z']


def display_frame(
    pc,
    axes_limits=axes_limits,
    bbox3d=None,
    point_sz=0.5,
    points=0.2,
    plotting=False,
    proj='xy',
):
    """
    Displays statistics for a single frame. Draws camera data, 3D plot of the lidar point cloud data and point cloud
    projections to various planes.

    Parameters
    ----------
    pc              : raw pointcloud
    point_sz        : point size to display
    points          : Fraction of lidar points to use. Defaults to `0.2`, e.g. 20%.
    plotting        : Plot the final image or not
    """

    def draw_point_cloud(
        ax,
        axes=[0, 1, 2],
        xlim3d=None,
        ylim3d=None,
        zlim3d=None,
        title="",
        equalax=False,
    ):
        """
        Convenient method for drawing various point cloud projections as a part of frame statistics.
        """

        # ax.scatter(*pc[:, axes], s=point_sz, c=pc[:, 3], cmap='gray')
        # ax.scatter(pc[:, axes], c=gs_blue)
        if len(axes) == 3:
            ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], s=point_sz, cmap='gray')
        elif len(axes) == 2:
            ax.scatter(pc[:, axes[0]], pc[:, axes[1]], s=point_sz, cmap='gray')
        ax.set_title(title)
        ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
        ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
        if len(axes) > 2:
            ax.set_xlim3d(*axes_limits[axes[0]])
            ax.set_ylim3d(*axes_limits[axes[1]])
            ax.set_zlim3d(*axes_limits[axes[2]])
            ax.set_zlabel('{} axis'.format(axes_str[axes[2]]))
            # ax.grid(False)
        else:
            ax.set_xlim(*axes_limits[axes[0]])
            ax.set_ylim(*axes_limits[axes[1]])
        # User specified limits
        if xlim3d != None:
            ax.set_xlim3d(xlim3d)
        if ylim3d != None:
            ax.set_ylim3d(ylim3d)
        if zlim3d != None:
            ax.set_zlim3d(zlim3d)

        if bbox3d is not None:
            for n in range(len(bbox3d)):
                b = bbox3d[n].T
                draw_box(ax, b, axes=axes, color='r')

        if equalax:
            ax.axis('equal')

    # Draw point cloud data as 3D plot
    fig3d = plt.figure(figsize=(15, 8))
    ax3d = fig3d.add_subplot(111, projection='3d')
    draw_point_cloud(ax3d, title='Pointcloud')
    if plotting:
        plt.show()

    # Draw point cloud data as plane projections
    if proj == 'xy':
        fig_proj, ax_proj = plt.subplots(figsize=(15, 10))
        draw_point_cloud(
            ax_proj,
            axes=[0, 1],  # X and Y axes
            title='Pointcloud in XY projection (Z = 0)',
            equalax=True,
        )
        if plotting:
            plt.show()
    elif proj == 'all':
        fig_proj, ax_proj = plt.subplots(3, 1, figsize=(15, 25))
        draw_point_cloud(
            ax_proj[0],
            axes=[0, 1],  # X and Y axes
            title='Pointcloud in XY projection (Z = 0)',
            equalax=True,
        )
        draw_point_cloud(
            ax_proj[1],
            axes=[0, 2],  # X and Z axes
            title='Pointcloud in XZ projection (Y = 0)',
            equalax=True,
        )
        draw_point_cloud(
            ax_proj[2],
            axes=[1, 2],  # Y and Z axes
            title='Pointcloud in YZ projection (X = 0)',
            equalax=True,
        )
        if plotting:
            plt.show()
    return fig3d, fig_proj, ax3d, ax_proj


def get_limits(pc):
    x_max, x_min = np.max(pc[:, 0]), np.min(pc[:, 0])
    y_max, y_min = np.max(pc[:, 1]), np.min(pc[:, 1])
    z_max, z_min = np.max(pc[:, 2]), np.min(pc[:, 2])

    axes_limits = [
        [x_min, x_max],  # X axis range
        [y_min, y_max],  # Y axis range
        [z_min, z_max],  # Z axis range
    ]
    return axes_limits


def update_limits(pc, axes_limits):
    flatten_ = np.array(axes_limits).flatten()

    x_max, x_min = np.max(pc[:, 0]), np.min(pc[:, 0])
    y_max, y_min = np.max(pc[:, 1]), np.min(pc[:, 1])
    z_max, z_min = np.max(pc[:, 2]), np.min(pc[:, 2])

    new_axes_limits = [
        [np.min([x_min, flatten_[0]]), np.max([x_max, flatten_[1]])],  # X axis range
        [np.min([y_min, flatten_[2]]), np.max([y_max, flatten_[3]])],  # Y axis range
        [np.min([z_min, flatten_[4]]), np.max([z_max, flatten_[5]])],  # Z axis range
    ]
    return new_axes_limits


def main():
    qolo_dataset = CrowdBotData()
    print("rosbag database:", qolo_dataset.bagbase_dir)
    print("output database:", qolo_dataset.outbase_dir)

    example_class = '0424_mds'
    example_seq = '2021-04-24-12-04-04'

    cb_data = CrowdBotDatabase(example_class)

    seq_idx, fr_idx = 0, 0
    seq = cb_data.seqs[seq_idx]

    print(
        "({}/{}): {} with {} frames".format(
            0 + 1, cb_data.nr_seqs(), seq, cb_data.nr_frames(seq_idx)
        )
    )

    lidar, _, _, trks = cb_data[0, 0]

    pc = lidar.T

    pshape(pc)
    pshape(trks)

    boxes = trks
    filtering = True
    filter_dist = 8
    verbose = False

    if filtering:
        pc = filter_pointcloud_distance(pc, filter_dist, verbose)
        pshape(pc)

    ids = boxes[:, -1]

    if len(boxes):
        if filtering:
            boxes = filter_detection_tracking_res(boxes, filter_dist, verbose)
        # from detection
        if boxes.shape[1] == 7:
            corners_xyz = boxes3d_to_corners3d_lidar(boxes, bottom_center=False)
        # from tracking
        elif boxes.shape[1] == 8:
            ids = boxes[:, -1]
            boxes = boxes[:, :-1]
            corners_xyz = boxes3d_to_corners3d_lidar(boxes, bottom_center=False)

    fig3d, fig_proj, ax3d, ax_proj = display_frame(
        pc, axes_limits=get_limits(pc), bbox3d=corners_xyz
    )
    fig_proj.savefig('test_proj.png', bbox_inches='tight')
    fig3d.savefig('test_3d.png', bbox_inches='tight')


if __name__ == "__main__":
    main()
