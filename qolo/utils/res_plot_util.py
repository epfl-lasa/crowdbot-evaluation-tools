#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   res_plot_util.py
@Date created  :   2021/11/16
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides plotting functions to visualize evaluation results
"""
# =============================================================================

import os
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def save_cd_img(
    crowd_eval_dict,
    path_eval_dict,
    base_dir,
    seq_name,
    fmt='pdf',
    add_title=False,
    use_serif=False,
    color_list=["tomato", "limegreen", "violet"],
    color_vertical='red',
):
    """crowd_density (2.5m, 5m, and 10m) plot function"""

    if use_serif:
        mpl.rcParams['font.family'] = ['serif']
        mpl.rcParams['font.serif'] = ['Times New Roman']
        mpl.rcParams['mathtext.fontset'] = 'stix'

    # unpack md data from eval_dict
    ts = crowd_eval_dict.get("timestamp")
    cd2_5 = crowd_eval_dict.get("crowd_density2_5")
    cd5 = crowd_eval_dict.get("crowd_density5")
    cd10 = crowd_eval_dict.get("crowd_density10")
    start_ts = path_eval_dict.get("start_command_ts")
    duration2goal = path_eval_dict.get("duration2goal")

    duration = np.max(ts) - np.min(ts)

    fig, ax = plt.subplots(figsize=(8, 4.5))

    # crowd_density chart
    norm_ts = ts - np.min(ts)
    (l1,) = ax.plot(norm_ts, cd2_5, lw=1, color=color_list[0], label="2.5 m")
    (l2,) = ax.plot(norm_ts, cd5, lw=1, color=color_list[1], label="5 m")
    (l3,) = ax.plot(norm_ts, cd10, lw=1, color=color_list[2], label="10 m")

    # start_ts vertical line
    new_start_ts = np.max([start_ts - np.min(ts), 0.0])
    y_max = max(round(max(cd2_5) * 10) / 10, 0.5)
    ax.axvline(x=new_start_ts, linestyle="--", linewidth=2, color=color_vertical)
    plt.text(
        x=new_start_ts + 1,
        y=y_max - 0.1,
        s="$t_s$ = {0:.1f} s".format(new_start_ts),
        horizontalalignment="left",
        fontsize=10,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color=color_vertical)
    plt.text(
        x=new_end_ts - 1,
        y=y_max - 0.1,
        s="$t_e$ = {0:.1f} s".format(new_end_ts),
        horizontalalignment="right",
        fontsize=10,
    )

    ax.legend(handles=[l1, l2, l3], ncol=1, loc="upper right", fontsize="x-small")
    if add_title:
        ax.set_title(
            "Crowd Density within x [m] of qolo ({0:.1f}s)".format(duration),
            fontsize=15,
        )
    _ = ax.set_xlabel("Time [$s$]", fontsize=14)
    _ = ax.set_ylabel("Density [$1/m^2$]", fontsize=14)

    ax.set_xlim(left=0.0, right=duration)
    ax.set_ylim(bottom=0.0, top=y_max)
    ax.tick_params(axis='x', labelsize=10)
    ax.tick_params(axis='y', labelsize=10)

    fig.tight_layout()
    cd_img_path = os.path.join(
        base_dir, seq_name, "{}_crowd_density.{}".format(seq_name, fmt)
    )
    plt.savefig(cd_img_path, dpi=300)  # png, pdf

    plt.close()


def save_cd_img_two(crowd_eval_dict, path_eval_dict, base_dir, seq_name):
    """crowd_density (5m and 10m) plot function"""

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
        s="$t_s$ = {0:.1f} s".format(new_start_ts),
        horizontalalignment="left",
        fontsize=6,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_end_ts + 1,
        y=0.40,
        s="$t_e$ = {0:.1f} s".format(new_end_ts),
        horizontalalignment="left",
        fontsize=6,
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
    cd_img_path = os.path.join(base_dir, seq_name, seq_name + "_crowd_density.png")
    plt.savefig(cd_img_path, dpi=300)  # png, pdf

    plt.close()


def save_cd_img_single(
    crowd_eval_dict,
    path_eval_dict,
    base_dir,
    seq_name,
    dist=2.5,
    fmt='pdf',
    linecolor="navy",
    add_title=False,
    use_serif=False,
):
    """crowd_density plot function"""

    key_dict = {2.5: "crowd_density2_5", 5: "crowd_density5", 10: "crowd_density10"}

    # unpack md data from eval_dict
    ts = crowd_eval_dict.get("timestamp")
    cd = crowd_eval_dict.get(key_dict[dist])
    start_ts = path_eval_dict.get("start_command_ts")
    duration2goal = path_eval_dict.get("duration2goal")

    duration = np.max(ts) - np.min(ts)

    if use_serif:
        mpl.rcParams['font.family'] = ['serif']
        mpl.rcParams['font.serif'] = ['Times New Roman']
        mpl.rcParams['mathtext.fontset'] = 'stix'

    fig, ax = plt.subplots(figsize=(10, 8))

    # crowd_density chart
    (l1,) = ax.plot(
        ts - np.min(ts), cd, linewidth=1, color=linecolor, label="x = {}".format(dist)
    )

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

    # ax.legend(handles=[l1], loc="upper right", fontsize="x-small")
    if add_title:
        ax.set_title(
            "Crowd Density within {} [m] of qolo ({0:.1f}s)".format(dist, duration),
            fontsize=16,
        )
    _ = ax.set_xlabel("Time [$s$]", fontsize=14)
    _ = ax.set_ylabel("Density [$1/m^2$]", fontsize=14)

    ax.set_xlim(left=0.0)
    y_max = max(round(max(cd) * 10) / 10, 0.5)
    ax.set_ylim(bottom=0.0, top=y_max)
    ax.tick_params(axis='x', labelsize=10)
    ax.tick_params(axis='y', labelsize=10)

    fig.tight_layout()
    cd_img_path = os.path.join(
        base_dir, seq_name, "{}_{}.{}".format(seq_name, key_dict[dist], fmt)
    )
    plt.savefig(cd_img_path, dpi=300)  # png, pdf

    plt.close()


def save_md_img(crowd_eval_dict, path_eval_dict, base_dir, seq_name):
    """min_dist plot function"""

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
        fontsize=6,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_end_ts - 1,
        y=4.2,
        s="$t_e$={0:.1f}s".format(new_end_ts),
        horizontalalignment="right",
        fontsize=6,
    )

    # y=0.0 horizontal line (if consider the qolo capsule)
    ax.plot((0.0, duration), (0.0, 0.0), linestyle="--", color="navy")

    ax.set_title(
        "Min. Distance of Pedestrain from qolo ({0:.1f}s)".format(duration), fontsize=15
    )
    _ = ax.set_xlabel("t [s]")
    _ = ax.set_ylabel("Distance [m]")

    ax.set_xlim(left=0.0, right=duration)
    # decrease the min value to negative value
    ax.set_ylim(bottom=-0.5, top=5.0)

    fig.tight_layout()
    md_img_path = os.path.join(base_dir, seq_name, seq_name + "_min_dist.png")
    plt.savefig(md_img_path, dpi=300)  # png, pdf

    plt.close()


def save_twist_cmd_img(
    qolo_twist, cmd_raw_dict, base_dir, seq_name, suffix="_twist_toGui_comp"
):
    """qolo command (twist msg) plot function"""

    # Only for debug:
    # linear vel in twist = corrected_linear in rds_to_gui
    # angular vel in twist = corrected_angular in rds_to_gui

    twist_ts = qolo_twist["timestamp"]
    cmd_ts = cmd_raw_dict["timestamp"]
    min_ts = np.min([twist_ts.min(), cmd_ts.min()])
    new_twist_ts = twist_ts - min_ts
    new_cmd_ts = cmd_ts - min_ts
    fig, ax = plt.subplots(1, 2, sharex="col", figsize=(10, 4))
    ax[0].plot(new_twist_ts, qolo_twist["x"], 'r', linewidth=2)
    ax[0].plot(new_cmd_ts, cmd_raw_dict["nominal_linear"], 'g:')
    ax[0].plot(new_cmd_ts, cmd_raw_dict["corrected_linear"], 'b--', linewidth=2)

    ax[1].plot(new_twist_ts, qolo_twist["zrot"], 'r', linewidth=2)
    ax[1].plot(new_cmd_ts, cmd_raw_dict["nominal_angular"], 'g:')
    ax[1].plot(new_cmd_ts, cmd_raw_dict["corrected_angular"], 'b--', linewidth=2)

    fig.tight_layout()
    qolo_img_path = os.path.join(base_dir, seq_name + suffix + ".png")
    plt.savefig(qolo_img_path, dpi=300)  # png, pdf

    plt.close()


def save_motion_img(
    qolo_command_dict, path_eval_dict, base_dir, seq_name, suffix, command=True
):
    """qolo state (pose, velocity, accleration, & jerk) plot function"""
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

    fig, ax = plt.subplots(3, 2, sharex="col", figsize=(10, 4))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)

    for i in range(3):
        for j in range(2):
            xx = ts - np.min(ts)
            yy = qolo_command_dict[plot_attr[i * 2 + j]]
            ax[i, j].plot(xx, yy, linewidth=0.8, color="purple")
            ax[i, j].axvline(x=new_start_ts, linestyle="--", linewidth=1.5, color="red")
            ax[i, j].axvline(x=new_end_ts, linestyle="--", linewidth=1.5, color="red")

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

            ax[i, j].set_ylabel(unit[i * 2 + j])
            if i == 2:
                ax[i, j].set_xlabel("t [s]")
                if j == 0:
                    if command:
                        ax[i, j].set_ylim(bottom=-10, top=10)
                    else:
                        ax[i, j].set_ylim(bottom=-50, top=50)
                elif j == 1:
                    if command:
                        ax[i, j].set_ylim(bottom=-20, top=20)
                    else:
                        ax[i, j].set_ylim(bottom=-50, top=50)
            elif i == 1:
                if j == 0:
                    if command:
                        ax[i, j].set_ylim(bottom=-1, top=1)
                    else:
                        ax[i, j].set_ylim(bottom=-2, top=2)
                elif j == 1:
                    if command:
                        ax[i, j].set_ylim(bottom=-1, top=1)
                    else:
                        ax[i, j].set_ylim(bottom=-2, top=2)
            elif i == 0:  # only nominal velocity is always nonnegative
                if j == 0:
                    ax[i, j].set_ylim(bottom=-1.5, top=1.5)
                elif j == 1:
                    ax[i, j].set_ylim(bottom=-2, top=2)

    fig.tight_layout()
    qolo_img_path = os.path.join(
        base_dir, seq_name, seq_name + suffix + ".png"
    )  # "_qolo_command"
    plt.savefig(qolo_img_path, dpi=300)  # png, pdf

    plt.close()


def save_path_img(qolo_pose2d, path_eval_dict, base_dir, seq_name):
    """qolo trajectory plot function"""
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
        "tomato",
        linewidth=2,
        label="Goal path (l=%.1f m, t=%.1f s)" % (path_length2goal, duration2goal),
    )
    ax.plot(
        pose_x[end_idx:],
        pose_y[end_idx:],
        "navy",
        linewidth=2,
        label="Return path",
    )
    # ax.plot([goal_loc[0]], [goal_loc[1]], "kx", label="goal")
    ax.plot([goal_loc[0]], [goal_loc[1]], "kx")
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

    goal_circle = mpatches.Circle(
        tuple(goal_loc), 3, color='navy', fill=True, label="Goal area"
    )
    ax.add_patch(goal_circle)

    ax.legend(fontsize="x-small")

    # adjust plots with equal axis aspect ratios
    ax.axis("equal")

    ax.set_title("Path. Closest distance to the goal={0:.1f}m".format(min_dist2goal))
    fig.tight_layout()
    path_img_path = os.path.join(base_dir, seq_name, seq_name + "_path.png")
    plt.savefig(path_img_path, dpi=300)  # png, pdf

    plt.close()


def draw_arrow(axes, startx, starty, orient, arrow_len=5.0, color='black', lw=2.0):
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
        label="Ped {}".format(ped_id),
    )


def get_nlongest_peds(ped_traj_dict, ped_num=3):
    traj_data = pd.DataFrame.from_dict(
        ped_traj_dict, orient='index', columns=['start_idx', 'end_idx', 'length']
    )
    top_ids = traj_data.nlargest(ped_num, 'length').index.values

    return top_ids


def viz_qolo_ped_traj_full(
    path_img_path, qolo_pose, ped_traj_dict, viz_ids, color_list=None
):
    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(
        qolo_pose["x"],
        qolo_pose["y"],
        "red",
        linewidth=2,
        label="Qolo trajectory",
    )

    # top_ids = get_nlongest_peds(ped_traj_dict, ped_num=ped_num)

    for ii, id in enumerate(viz_ids):
        xyz = np.array(ped_traj_dict[id]['abs_pose_list'])
        if color_list is not None:
            plot_ped_traj(ax, xyz[:, :2], id, color_list[ii])
        else:
            plot_ped_traj(ax, xyz[:, :2], id)

    ax.legend(fontsize="x-small")

    ax.set_title("Qolo & Pedestrian Trajectories")
    fig.tight_layout()
    plt.savefig(path_img_path, dpi=300)  # png, pdf

    plt.close()

def viz_qolo_ped_traj_frame(
    path_img_path, frame_id, qolo_pose, ped_traj_dict, viz_ids, color_list=None
):
    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(
        qolo_pose["x"][: frame_id + 1],
        qolo_pose["y"][: frame_id + 1],
        "red",
        linewidth=2,
        label="Qolo trajectory",
    )

    # top_ids = get_nlongest_peds(ped_traj_dict, ped_num=ped_num)

    for ii, id in enumerate(viz_ids):
        xyz = np.array(ped_traj_dict[id]['abs_pose_list'])
        if color_list is not None:
            plot_ped_traj(ax, xyz[:, :2], id, color_list[ii])
        else:
            plot_ped_traj(ax, xyz[:, :2], id)

    ax.legend(fontsize="x-small")

    ax.set_title("Qolo & Pedestrian Trajectories")
    fig.tight_layout()
    plt.savefig(path_img_path, dpi=300)  # png, pdf

    plt.close()

def viz_ped_speed(ped_vel_img_path, ped_vel_dict, viz_ids, color_list=None):
    """pedestrian motion (velocity) plot function"""
    # print("To be implemented")
    fig, ax = plt.subplots(2, 1, figsize=(8, 10))

    for ii, ped_id in enumerate(viz_ids):
        lin_vel = np.array(ped_vel_dict[ped_id]['lin_vel'])
        start_idx = ped_vel_dict[ped_id]['start_idx']
        end_idx = ped_vel_dict[ped_id]['end_idx']
        color = color_list[ii]

        ax[0].plot(
            lin_vel[:,0],
            color,
            linewidth=1,
            label="Ped {} ({}-{})".format(ped_id, start_idx, end_idx),
        )
        ax[1].plot(
            lin_vel[:,1],
            color,
            linewidth=1,
            label="Ped {} ({}-{})".format(ped_id, start_idx, end_idx),
        )

    ax[0].legend(fontsize="x-small", loc='upper right')
    ax[0].set_title("Pedestrian Velocities (x)")
    ax[0].set_xlabel("Frame")
    ax[0].set_ylabel("Speed")

    ax[1].legend(fontsize="x-small", loc='upper right')
    ax[1].set_title("Pedestrian Velocities (y)")
    ax[1].set_xlabel("Frame")
    ax[1].set_ylabel("Speed")

    fig.tight_layout()
    plt.savefig(ped_vel_img_path, dpi=300)  # png, pdf

    plt.close()

