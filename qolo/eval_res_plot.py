#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   eval_res_plot.py
@Date created  :   2021/11/16
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides plotting functions to visualize evaluation results
"""
# =============================================================================
"""
TODO:
1. cannot show correctly when `MAX=nan, MIN=-nan` occurs
"""
# =============================================================================

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def save_cd_img(crowd_eval_dict, path_eval_dict, base_dir, seq_name):
    """save crowd_density plotting"""

    # unpack md data from eval_dict
    ts = crowd_eval_dict.get("timestamp")
    cd5 = crowd_eval_dict.get("crowd_density5")
    cd10 = crowd_eval_dict.get("crowd_density10")
    start_ts = path_eval_dict.get("start_command_ts")
    duration2goal = path_eval_dict.get("duration2goal")

    duration = np.max(ts) - np.min(ts)

    fig, ax = plt.subplots(figsize=(8, 4))

    # crowd_density chart
    (l1,) = ax.plot(ts - np.min(ts), cd5, linewidth=1, color="coral", label="x = 5")
    (l2,) = ax.plot(ts - np.min(ts), cd10, linewidth=1, color="navy", label="x = 10")

    # start_ts vertical line
    new_start_ts = np.max([start_ts - np.min(ts), 0.0])
    ax.axvline(x=new_start_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_start_ts + 1,
        y=0.40,
        s="$t_s$={0:.1f}s".format(new_start_ts),
        horizontalalignment="left",
        fontsize=10,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_end_ts + 1,
        y=0.40,
        s="$t_e$={0:.1f}s".format(new_end_ts),
        horizontalalignment="left",
        fontsize=10,
    )

    ax.legend(handles=[l1, l2], ncol=2, loc="upper right", fontsize="x-small")
    ax.set_title(
        "Crowd Density within x [m] of qolo ({0:.1f}s)".format(duration), fontsize=15
    )
    _ = ax.set_xlabel("t [s]")
    _ = ax.set_ylabel("Density [1/$m^2$]")

    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0, top=0.5)

    fig.tight_layout()
    cd_img_path = os.path.join(base_dir, seq_name + "_crowd_density.png")
    plt.savefig(cd_img_path, dpi=300)  # png, pdf


def save_md_img(crowd_eval_dict, path_eval_dict, base_dir, seq_name):
    """save min_dist plotting"""

    # unpack md data from eval_dict
    ts = crowd_eval_dict.get("timestamp")
    md = crowd_eval_dict.get("min_dist")
    start_ts = path_eval_dict.get("start_command_ts")
    duration2goal = path_eval_dict.get("duration2goal")

    duration = np.max(ts) - np.min(ts)

    fig, ax = plt.subplots(figsize=(8, 4))

    # min_dist chart
    ax.plot(ts - np.min(ts), md, linewidth=1, color="coral")

    # start_ts vertical line
    new_start_ts = np.max([start_ts - np.min(ts), 0.0])
    ax.axvline(x=new_start_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_start_ts + 1,
        y=4.2,
        s="$t_s$={0:.1f}s".format(new_start_ts),
        horizontalalignment="left",
        fontsize=10,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_end_ts + 1,
        y=4.2,
        s="$t_e$={0:.1f}s".format(new_end_ts),
        horizontalalignment="left",
        fontsize=10,
    )

    # y=0.3 horizontal line (if consider the qolo capsule)
    # ax.plot((0.0, duration), (0.3, 0.3), linestyle="--", color="navy")
    # plt.text(
    #     x=duration / 2,
    #     y=0.4,
    #     s=r"$\mathrm{dist}_{\mathrm{limit}}=0.3$",
    #     horizontalalignment="center",
    #     verticalalignment="baseline",
    #     fontsize=10,
    # )

    ax.set_title(
        "Min. Distance of Pedestrain from qolo ({0:.1f}s)".format(duration), fontsize=15
    )
    _ = ax.set_xlabel("t [s]")
    _ = ax.set_ylabel("Distance [m]")

    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0, top=5.0)

    fig.tight_layout()
    md_img_path = os.path.join(base_dir, seq_name + "_min_dist.png")
    plt.savefig(md_img_path, dpi=300)  # png, pdf


def save_motion_img(qolo_command_dict, path_eval_dict, base_dir, seq_name, suffix):
    ts = qolo_command_dict["timestamp"]
    start_ts = path_eval_dict.get("start_command_ts")
    duration2goal = path_eval_dict.get("duration2goal")

    new_start_ts = np.max([start_ts - np.min(ts), 0.0])
    new_end_ts = new_start_ts + duration2goal

    plot_attr = ("x_vel", "zrot_vel", "x_acc", "zrot_acc", "x_jerk", "zrot_jerk")
    unit = (
        "$V$ [$m/s$]",
        "$V_w$ [$rad/s$]",
        "$a$ [$m/s^2$]",
        "$a_w$ [$rad/s^2$]",
        "$J$ [$m/s^3$]",
        "$J_w$ [$rad/s^3$]",
    )

    # ref: https://jakevdp.github.io/PythonDataScienceHandbook/04.08-multiple-subplots.html
    fig, ax = plt.subplots(3, 2, sharex="col", figsize=(10, 4))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    # ref: https://matplotlib.org/stable/gallery/subplots_axes_and_figures/axes_zoom_effect.html#sphx-glr-gallery-subplots-axes-and-figures-axes-zoom-effect-py
    # ref: https://matplotlib.org/stable/gallery/shapes_and_collections/artist_reference.html#sphx-glr-gallery-shapes-and-collections-artist-reference-py

    for i in range(3):
        for j in range(2):
            xx = ts - np.min(ts)
            yy = qolo_command_dict[plot_attr[i * 2 + j]]
            ax[i, j].plot(xx, yy, linewidth=0.8, color="purple")
            ax[i, j].axvline(x=new_start_ts, linestyle="--", linewidth=1.5, color="red")
            ax[i, j].axvline(x=new_end_ts, linestyle="--", linewidth=1.5, color="red")

            # ref: https://stackoverflow.com/questions/50753721/can-not-reset-the-axes
            # print(type(np.max(yy)))
            # print(type(np.abs(np.min(yy))))
            # TypeError: 'numpy.float64' object cannot be interpreted as an integer
            # y_lim = np.max(np.max(yy), np.abs(np.min(yy)))
            if np.max(yy) >= np.abs(np.min(yy)):
                y_lim = np.max(yy)
            else:
                y_lim = np.abs(np.min(yy))
            rect_up = mpatches.Rectangle(
                (new_start_ts, 0),
                duration2goal,
                100.0 * y_lim,
                facecolor="red",
                alpha=0.2,
            )  # xy, width, height
            ax[i, j].add_patch(rect_up)
            rect_down = mpatches.Rectangle(
                (new_start_ts, -100.0 * y_lim),
                duration2goal,
                100.0 * y_lim,
                facecolor="red",
                alpha=0.2,
            )
            ax[i, j].add_patch(rect_down)
            # TODO: cannot show correctly when `MAX=nan, MIN=-nan` occurs
            # print("MAX={}, MIN=-{}".format(max_y, min_y))

            ax[i, j].set_ylabel(unit[i * 2 + j])
            if i == 2:
                ax[i, j].set_xlabel("t [s]")
            if i == 0:  # only nominal velocity is always nonnegative
                if i == 0 and j == 0:
                    ax[i, j].set_ylim(bottom=-1.5, top=1.5)
                elif i == 0 and j == 1:
                    ax[i, j].set_ylim(bottom=-2, top=2)

    fig.tight_layout()
    qolo_img_path = os.path.join(
        base_dir, seq_name + suffix + ".png"
    )  # "_qolo_command"
    plt.savefig(qolo_img_path, dpi=300)  # png, pdf

    # https://stackoverflow.com/questions/21884271/warning-about-too-many-open-figures
    plt.close()


def save_path_img(qolo_pose2d, path_eval_dict, base_dir, seq_name):
    pose_x = qolo_pose2d.get("x")
    pose_y = qolo_pose2d.get("y")
    pose_theta = qolo_pose2d.get("theta")

    duration2goal = path_eval_dict['duration2goal']
    path_length2goal = path_eval_dict['path_length2goal']
    start_idx = path_eval_dict['start_idx']
    end_idx = path_eval_dict['end_idx']
    min_dist2goal = path_eval_dict['min_dist2goal']
    goal_loc = path_eval_dict['goal_loc']

    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(
        pose_x[:end_idx],
        pose_y[:end_idx],
        "orange",
        linewidth=2,
        label="path (l=%.1f m, t=%.1f s)" % (path_length2goal, duration2goal),
    )
    ax.plot(
        pose_x[end_idx:],
        pose_y[end_idx:],
        "skyblue",
        linewidth=2,
        label="remaining path",
    )
    ax.plot([goal_loc[0]], [goal_loc[1]], "kx", label="goal")
    ax.legend(fontsize="x-small")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # start arrow
    arrow_len = 5.0
    avg_num = 10
    init_yaw = np.sum(pose_theta[start_idx : start_idx + avg_num]) / avg_num
    # init_yaw = np.sum(pose_theta[9:19]) / 10.0  # pose_theta[0]
    xy = (pose_x[0], pose_y[0])
    dxy = (np.cos(init_yaw) * arrow_len, np.sin(init_yaw) * arrow_len)
    xytext = tuple(map(sum, zip(xy, dxy)))

    ax.annotate(
        "",
        xy=xy,
        xytext=xytext,
        arrowprops=dict(arrowstyle="<-", lw=2.0),
        color='purple',
    )

    # adjust plots with equal axis aspect ratios
    ax.axis("equal")

    ax.set_title("Path. Closest distance to the goal={0:.1f}m".format(min_dist2goal))
    fig.tight_layout()
    path_img_path = os.path.join(base_dir, seq_name + "_path.png")
    plt.savefig(path_img_path, dpi=300)  # png, pdf

    # https://stackoverflow.com/questions/21884271/warning-about-too-many-open-figures
    plt.close()
