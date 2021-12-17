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

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def save_cd_img(crowd_eval_dict, path_eval_dict, base_dir, seq_name):
    """save crowd_density (3m, 5m, and 10m) plotting"""

    # unpack md data from eval_dict
    ts = crowd_eval_dict.get("timestamp")
    cd3 = crowd_eval_dict.get("crowd_density3")
    cd5 = crowd_eval_dict.get("crowd_density5")
    cd10 = crowd_eval_dict.get("crowd_density10")
    start_ts = path_eval_dict.get("start_command_ts")
    duration2goal = path_eval_dict.get("duration2goal")

    duration = np.max(ts) - np.min(ts)

    fig, ax = plt.subplots(figsize=(8, 4))

    # crowd_density chart
    (l1,) = ax.plot(ts - np.min(ts), cd3, linewidth=1, color="tomato", label="3 m")
    (l2,) = ax.plot(ts - np.min(ts), cd5, linewidth=1, color="limegreen", label="5 m")
    (l3,) = ax.plot(ts - np.min(ts), cd10, linewidth=1, color="violet", label="10 m")

    # start_ts vertical line
    new_start_ts = np.max([start_ts - np.min(ts), 0.0])
    ax.axvline(x=new_start_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_start_ts + 1,
        y=0.60,
        s="$t_s$={0:.1f}s".format(new_start_ts),
        horizontalalignment="left",
        fontsize=6,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_end_ts - 1,
        y=0.60,
        s="$t_e$={0:.1f}s".format(new_end_ts),
        horizontalalignment="right",
        fontsize=6,
    )

    ax.legend(handles=[l1, l2, l3], ncol=1, loc="upper right", fontsize="x-small")
    ax.set_title(
        "Crowd Density within x [m] of qolo ({0:.1f}s)".format(duration), fontsize=15
    )
    _ = ax.set_xlabel("t [s]")
    _ = ax.set_ylabel("Density [1/$m^2$]")

    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0, top=0.8)

    fig.tight_layout()
    cd_img_path = os.path.join(base_dir, seq_name + "_crowd_density.png")
    plt.savefig(cd_img_path, dpi=300)  # png, pdf

    plt.close()


def save_cd_img_two(crowd_eval_dict, path_eval_dict, base_dir, seq_name):
    """save crowd_density (5m and 10m)  plotting"""

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
        fontsize=6,
    )
    new_end_ts = new_start_ts + duration2goal
    ax.axvline(x=new_end_ts, linestyle="--", linewidth=2, color="red")
    plt.text(
        x=new_end_ts + 1,
        y=0.40,
        s="$t_e$={0:.1f}s".format(new_end_ts),
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
    cd_img_path = os.path.join(base_dir, seq_name + "_crowd_density.png")
    plt.savefig(cd_img_path, dpi=300)  # png, pdf

    plt.close()


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
    md_img_path = os.path.join(base_dir, seq_name + "_min_dist.png")
    plt.savefig(md_img_path, dpi=300)  # png, pdf

    plt.close()


def save_twist_cmd_img(
    qolo_twist, cmd_raw_dict, base_dir, seq_name, suffix="_twist_toGui_comp"
):
    """
    Only for debug:
    linear vel in twist = corrected_linear in rds_to_gui
    angular vel in twist = corrected_angular in rds_to_gui
    """
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
        base_dir, seq_name + suffix + ".png"
    )  # "_qolo_command"
    plt.savefig(qolo_img_path, dpi=300)  # png, pdf

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
        tuple(goal_loc), 3, color='skyblue', fill=True, label="Goal area"
    )
    ax.add_patch(goal_circle)

    ax.legend(fontsize="x-small")

    # adjust plots with equal axis aspect ratios
    ax.axis("equal")

    ax.set_title("Path. Closest distance to the goal={0:.1f}m".format(min_dist2goal))
    fig.tight_layout()
    path_img_path = os.path.join(base_dir, seq_name + "_path.png")
    plt.savefig(path_img_path, dpi=300)  # png, pdf

    plt.close()
