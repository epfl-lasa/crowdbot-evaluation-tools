# -*-coding:utf-8 -*-
"""
@File    :   eval_crowd.py
@Time    :   2021/11/02
@Author  :   Yujie He
@Version :   1.1
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

"""
evaluate the min. dist. from qolo and crowd density within 10m of qolo and save corresponding images
TODO: compare with detected pedestrain from the rosbag!
"""

import os
import argparse

import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from crowdbot_data import CrowdBotDatabase
from eval_qolo import compute_time_path

#%% utility functions to calculate the distance from detected pedestrian to qolo
# borrow from https://github.com/epfl-lasa/qolo-evaluation/blob/main/src/crowd_evaluation/crowd_evaluation.py
class Capsule:
    def __init__(self, y_front, y_back, r):
        self.y_front = y_front
        self.y_back = y_back
        self.r = r

    def distanceLocal(self, x, y):
        if y > self.y_front:
            return math.sqrt(x * x + (y - self.y_front) * (y - self.y_front)) - self.r
        elif y < self.y_back:
            return math.sqrt(x * x + (y - self.y_back) * (y - self.y_back)) - self.r
        else:
            return math.fabs(x) - self.r

    def distanceGlobal(self, x_obs, y_obs, x_rob, y_rob, phi_rob):
        Rinv = [
            [np.cos(phi_rob - np.pi / 2), np.sin(phi_rob - np.pi / 2)],
            [-np.sin(phi_rob - np.pi / 2), np.cos(phi_rob - np.pi / 2)],
        ]

        p_rel_global = np.array([x_obs - x_rob, y_obs - y_rob])
        p_rel_local = np.matmul(np.array(Rinv), p_rel_global)
        return self.distanceLocal(p_rel_local[0], p_rel_local[1])


#%% utility functions to evaluate crowd data
def compute_crowd_metrics(bbox):
    """compute crowd density and min_dist from qolo"""

    # 0. all_det
    all_det = np.shape(bbox)[0]

    # 1. all_dist: all pedestrain from qolo
    # all_dist = np.linalg.norm(bbox[:, [0, 1]], axis=1)
    bbox_xy = bbox[:, [0, 1]]
    all_dist = np.linalg.norm(bbox_xy, axis=1)
    # consider qolo capsule
    # all_dist_capsuled
    capsule_qolo = Capsule(0.18, -0.5, 0.45)
    all_dist_capsuled = np.ones([all_det]) * np.inf
    for idx in range(all_det):
        x_dist = bbox_xy[idx, 0]
        y_dist = bbox_xy[idx, 1]
        all_dist_capsuled[idx] = capsule_qolo.distanceLocal(x_dist, y_dist)

    # 2. within_det
    within_det5 = np.sum(np.less(all_dist, 5.0))
    within_det10 = np.sum(np.less(all_dist, 10.0))

    # 3. min_dist/proximity: all_dist_capsuled or all_dist
    min_dist = min(all_dist_capsuled)

    # 4. crowd_density
    area_local5 = np.pi * 5.0 ** 2
    crowd_density5 = within_det5 / area_local5
    area_local10 = np.pi * 10.0 ** 2
    crowd_density10 = within_det10 / area_local10

    return (
        all_det,
        within_det5,
        within_det10,
        crowd_density5,
        crowd_density10,
        min_dist,
    )


def save_cd_img(eval_dict, base_dir, seq_name):
    """save crowd_density plotting"""

    # unpack md data from eval_dict
    ts = eval_dict.get("timestamp")
    cd5 = eval_dict.get("crowd_density5")
    cd10 = eval_dict.get("crowd_density10")
    start_ts = eval_dict.get("start_command_ts")
    duration2goal = eval_dict.get("duration2goal")

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


def save_md_img(eval_dict, base_dir, seq_name):
    """save min_dist plotting"""

    # unpack md data from eval_dict
    ts = eval_dict.get("timestamp")
    md = eval_dict.get("min_dist")
    start_ts = eval_dict.get("start_command_ts")
    duration2goal = eval_dict.get("duration2goal")

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


#%% main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="nocam_rosbags",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--save_img",
        dest="save_img",
        action="store_true",
        help="plot and save crowd density image",
    )
    parser.set_defaults(save_img=True)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    parser.add_argument(
        "--replot",
        dest="replot",
        action="store_true",
        help="Whether to re-plot existing images (default: false)",
    )
    parser.set_defaults(replot=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    print("Starting evaluating crowd from {} sequences!".format(cb_data.nr_seqs()))

    eval_res_dir = os.path.join(cb_data.metrics_dir)

    if not os.path.exists(eval_res_dir):
        print("Result images and npy will be saved in {}".format(eval_res_dir))
        os.makedirs(eval_res_dir, exist_ok=True)

    for seq_idx in range(cb_data.nr_seqs()):

        seq = cb_data.seqs[seq_idx]
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, cb_data.nr_seqs(), seq, cb_data.nr_frames(seq_idx)
            )
        )

        # load twist, pose2d
        twist_dir = os.path.join(cb_data.source_data_dir, "twist")
        qolo_twist_path = os.path.join(twist_dir, seq + "_twist_raw.npy")  # _twist
        if not os.path.exists(qolo_twist_path):
            print("ERROR: Please extract twist_stamped by using twist2npy.py")
        qolo_twist = np.load(qolo_twist_path, allow_pickle=True).item()

        pose2d_dir = os.path.join(cb_data.source_data_dir, "pose2d")
        qolo_pose2d_path = os.path.join(pose2d_dir, seq + "_pose2d.npy")
        if not os.path.exists(qolo_pose2d_path):
            print("ERROR: Please extract pose2d by using pose2d2npy.py")
        qolo_pose2d = np.load(qolo_pose2d_path, allow_pickle=True).item()

        # compute (start_ts, end_idx, end_ts, duration2goal, path_length2goal)
        time_path_computed = compute_time_path(qolo_twist, qolo_pose2d)

        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        crowd_eval_npy = os.path.join(eval_res_dir, seq + "_crowd_eval.npy")

        # only for plotting function update!
        if args.replot:
            crowd_eval_dict = np.load(crowd_eval_npy, allow_pickle=True).item()

            # figure1: crowd density
            save_cd_img(crowd_eval_dict, eval_res_dir, seq)

            # figure2: min. dist.
            save_md_img(crowd_eval_dict, eval_res_dir, seq)

            print("Replot images!")
        else:
            if (not os.path.exists(crowd_eval_npy)) or (args.overwrite):

                # timestamp can be read from lidars/ folder
                stamp_file_path = os.path.join(cb_data.lidar_dir, seq + "_stamped.npy")
                lidar_stamped_dict = np.load(stamp_file_path, allow_pickle=True)
                ts = lidar_stamped_dict.item().get("timestamp")

                # targeted metrics and correspoding dtype
                attrs = (
                    "all_det",
                    "within_det5",
                    "within_det10",
                    "crowd_density5",
                    "crowd_density10",
                    "min_dist",
                )
                dtypes = (
                    np.uint8,
                    np.uint8,
                    np.uint8,
                    np.float32,
                    np.float32,
                    np.float32,
                )

                crowd_eval_list_dict = {k: [] for k in attrs}

                num_msgs_between_logs = 5
                nr_frames = cb_data.nr_frames(seq_idx)

                for fr_idx in range(nr_frames):

                    _, _, _, trks = cb_data[seq_idx, fr_idx]

                    metrics = compute_crowd_metrics(trks)

                    if fr_idx % num_msgs_between_logs == 0 or fr_idx >= nr_frames - 1:
                        print(
                            "Seq {}/{} - Frame {}/{}: filtered/overall boxes within 10m: {}/{}".format(
                                seq_idx + 1,
                                cb_data.nr_seqs(),
                                fr_idx + 1,
                                nr_frames,
                                metrics[2],
                                metrics[0],
                            )
                        )

                    # update value for each attr
                    for idx, val in enumerate(metrics):
                        crowd_eval_list_dict[attrs[idx]].append(val)

                crowd_eval_dict = {
                    name: np.asarray(crowd_eval_list_dict[attrs[idx]], dtype=dtype)
                    for idx, (name, dtype) in enumerate(zip(attrs, dtypes))
                }

                # normalized proximity
                min_dist_list = metrics[5]
                normalized_proximity = np.std(min_dist_list) / np.mean(min_dist_list)
                crowd_eval_dict.update({"normalized_proximity": normalized_proximity})

                # avg_min_dict = np.average(crowd_eval_dict['min_dict'])
                for attr in attrs:
                    avg_attr = "avg_" + attr
                    crowd_eval_dict.update(
                        {avg_attr: np.average(crowd_eval_dict[attr])}
                    )

                # other attributes from time_path_computed
                crowd_eval_dict.update({"timestamp": ts})
                crowd_eval_dict.update({"start_command_ts": time_path_computed[0]})
                crowd_eval_dict.update({"end_command_ts": time_path_computed[1]})
                crowd_eval_dict.update({"duration2goal": time_path_computed[2]})
                crowd_eval_dict.update({"path_lenth2goal": time_path_computed[3]})
                crowd_eval_dict.update({"goal_reached": time_path_computed[5]})

                np.save(crowd_eval_npy, crowd_eval_dict)

                if args.save_img:
                    # figure1: crowd density
                    save_cd_img(crowd_eval_dict, eval_res_dir, seq)

                    # figure2: min. dist.
                    save_md_img(crowd_eval_dict, eval_res_dir, seq)

            else:
                print(
                    "Detecting the generated {} already existed!".format(crowd_eval_npy)
                )
                print(
                    "Will not overwrite. If you want to overwrite, use flag --overwrite"
                )
                continue

    print("Finish crowd evaluation!")
