#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   viz_traj.py
@Date created  :   2022/02/25
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
1. visualize the robot traj
2. visualize longest 3-5 traj
3. plot vx/vy or linear/angular velocity
"""
# =============================================================================

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def draw_arrow(axes, startx, starty, orient, arrow_len=5.0, color='purple', lw=2.0):
    """Draw arrow on the axes handle"""
    xy = (startx, starty)
    dxy = (np.cos(orient) * arrow_len, np.sin(orient) * arrow_len)
    xytext = tuple(map(sum, zip(xy, dxy)))
    axes.annotate(
        "",
        xy=xy,
        xytext=xytext,
        arrowprops=dict(arrowstyle="<-", lw=lw),
        color=color,
    )


def draw_coordinate(axes, startx, starty, arrow_len=5.0, lw=2.0):
    """Draw x-y coordinate on the axes handle"""
    xy = (startx, starty)
    dx = (arrow_len, 0)
    xtext = tuple(map(sum, zip(xy, dx)))
    axes.annotate(
        "",
        xy=xy,
        xytext=xtext,
        arrowprops=dict(arrowstyle="<-", lw=lw),
        color='red',
    )

    dy = (0, arrow_len)
    ytext = tuple(map(sum, zip(xy, dy)))
    axes.annotate(
        "",
        xy=xy,
        xytext=ytext,
        arrowprops=dict(arrowstyle="<-", lw=lw),
        color='green',
    )


def plot_ped_traj(axes, xy_list, ped_id, color='red', lw=1):
    axes.plot(
        xy_list[:, 0],
        xy_list[:, 1],
        color=color,
        lw=lw,
        linewidth=2,
        label="Ped {}".format(ped_id),
    )


def save_qolo_ped_traj(
    base_dir, seq_name, qolo_pose2d, ped_traj_dict, ped_num=3, color_list=None
):
    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(
        qolo_pose2d.get("x"),
        qolo_pose2d.get("y"),
        "tomato",
        linewidth=2,
        label="Qolo trajectory",
    )

    traj_data = pd.DataFrame.from_dict(
        ped_traj_dict, orient='index', columns=['start_idx', 'end_idx', 'length']
    )

    top_ids = traj_data.nlargest(ped_num, 'length').index.values

    for id in top_ids:
        xyz = np.array(ped_traj_dict[id]['abs_pose_list'])
        plot_ped_traj(ax, xyz[:, :2], id)

    ax.set_title("Qolo & Pedestrian Trajectories")
    fig.tight_layout()
    path_img_path = os.path.join(base_dir, seq_name, seq_name + "_path.png")
    plt.savefig(path_img_path, dpi=300)  # png, pdf

    plt.close()
