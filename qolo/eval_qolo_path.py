#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   eval_qolo_path.py
@Date created  :   2021/12/07
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides the evaluation pipeline to compute path efficiency-related
metrics (relative time to goal, relative time to goal, and corresponding
visualization.
The emulation results is exported with suffix as "_path_eval.npy".
"""
# =============================================================================
"""
TODO:
1. change goal_dist according to `data_params.yaml`
"""
# =============================================================================

import os
import argparse

import numpy as np

from crowdbot_data import CrowdBotDatabase
from eval_res_plot import save_path_img
from metric_qolo_perf import compute_time_path

#%% main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate path efficiency")

    parser.add_argument(
        "-b",
        "--base",
        default="/home/crowdbot/Documents/yujie/crowdbot_tools",
        type=str,
        help="base folder, i.e., the path of the current workspace",
    )
    parser.add_argument(
        "-d",
        "--data",
        default="data",
        type=str,
        help="data folder, i.e., the name of folder that stored extracted raw data and processed data",
    )
    parser.add_argument(
        "-f",
        "--folder",
        default="0421_mds",
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
    parser.add_argument(
        "--goal_dist", default=20.0, type=float, help="The length to travel in the test"
    )
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    print("Starting evaluating qolo from {} sequences!".format(cb_data.nr_seqs()))

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

        # load pose2d
        pose2d_dir = os.path.join(cb_data.source_data_dir, "pose2d")
        qolo_pose2d_path = os.path.join(pose2d_dir, seq + "_pose2d.npy")
        if not os.path.exists(qolo_pose2d_path):
            print("ERROR: Please extract pose2d by using pose2d2npy.py")
        qolo_pose2d = np.load(qolo_pose2d_path, allow_pickle=True).item()

        # load twist, qolo_command
        twist_dir = os.path.join(cb_data.source_data_dir, "twist")
        qolo_twist_path = os.path.join(twist_dir, seq + "_twist_raw.npy")
        if not os.path.exists(qolo_twist_path):
            print("ERROR: Please extract twist_stamped by using twist2npy.py")
        qolo_twist = np.load(qolo_twist_path, allow_pickle=True).item()

        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        path_eval_npy = os.path.join(eval_res_dir, seq + "_path_eval.npy")

        # only for plotting function update!
        if args.replot:
            path_eval_dict = np.load(path_eval_npy, allow_pickle=True).item()

            save_path_img(qolo_pose2d, path_eval_dict, eval_res_dir, seq)

            print("Replot images!")
        else:
            if (not os.path.exists(path_eval_npy)) or (args.overwrite):

                """
                time_path_computed = (
                start_ts,
                end_ts,
                duration2goal,
                path_length2goal,
                end_idx,
                goal_reached,
                min_dist2goal,
                goal_loc,
                rel_duration2goal,
                rel_path_length2goal,
                )
                """
                time_path_computed = compute_time_path(
                    qolo_twist, qolo_pose2d, args.goal_dist
                )

                path_eval_dict = dict()

                # timestamp can be read from lidars/ folder
                stamp_file_path = os.path.join(cb_data.lidar_dir, seq + "_stamped.npy")
                lidar_stamped_dict = np.load(stamp_file_path, allow_pickle=True)
                lidar_ts = lidar_stamped_dict.item().get("timestamp")

                path_eval_dict.update({"start_command_ts": time_path_computed[0]})
                path_eval_dict.update({"end_command_ts": time_path_computed[1]})
                path_eval_dict.update({"duration2goal": time_path_computed[2]})
                path_eval_dict.update({"path_length2goal": time_path_computed[3]})
                path_eval_dict.update({"end_idx": time_path_computed[4]})
                path_eval_dict.update({"goal_reached": time_path_computed[5]})
                path_eval_dict.update({"min_dist2goal": time_path_computed[6]})
                path_eval_dict.update({"goal_loc": time_path_computed[7]})
                path_eval_dict.update({"rel_duration2goal": time_path_computed[8]})
                path_eval_dict.update({"rel_path_length2goal": time_path_computed[9]})

                np.save(path_eval_npy, path_eval_dict)

                if args.save_img:
                    save_path_img(qolo_pose2d, path_eval_dict, eval_res_dir, seq)
            else:
                print(
                    "Detecting the generated {} already existed!".format(path_eval_npy)
                )
                print(
                    "Will not overwrite. If you want to overwrite, use flag --overwrite"
                )
                continue

    print("Finish qolo evaluation!")
