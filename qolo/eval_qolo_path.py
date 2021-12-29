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
usage: eval_qolo_path.py [-h] [-f FOLDER] [--params_path PARAMS_PATH] [--save_img] [--overwrite]
                         [--replot]

Evaluate path efficiency

optional arguments:
  -h, --help            show this help message and exit
  -f FOLDER, --folder FOLDER
                        different subfolder in rosbag/ dir
  --params_path PARAMS_PATH
                        path to dataset parameters
  --save_img            plot and save crowd density image (default: true)
  --overwrite           Whether to overwrite existing rosbags (default: false)
  --replot              Whether to re-plot existing images (default: false)
"""
# =============================================================================

import os
import argparse
from pathlib import Path

import numpy as np

from crowdbot_data import CrowdBotDatabase, CrowdbotExpParam, CROWDBOT_EVAL_TOOLKIT_DIR
from eval_res_plot import save_path_img
from metric_qolo_perf import compute_time_path

#%% main function
if __name__ == "__main__":

    data_params_path = os.path.join(
        CROWDBOT_EVAL_TOOLKIT_DIR, "data", "data_params.yaml"
    )

    parser = argparse.ArgumentParser(description="Evaluate path efficiency")

    parser.add_argument(
        "-f",
        "--folder",
        default="0424_mds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--params_path",
        default=data_params_path,
        type=str,
        help="path to dataset parameters",
    )
    parser.add_argument(
        "--save_img",
        dest="save_img",
        action="store_true",
        help="plot and save crowd density image (default: true)",
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

    print("Starting evaluating qolo from {} sequences!".format(cb_data.nr_seqs()))

    all_data_params = CrowdbotExpParam(args.params_path)
    date = args.folder[:4]
    control_type = args.folder[5:]
    data_params = all_data_params.get_params(date, control_type)
    # {'goal_dist': float, 'vel_user_max': float, 'omega_user_max': float}
    print("# Experiment data:", date)
    print("# Experiment control type:", control_type)
    print("# Experiment settings:", data_params)

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
        eval_seq_dir_path = os.path.join(eval_res_dir, seq)
        if not os.path.exists(eval_seq_dir_path):
            os.makedirs(eval_seq_dir_path)
        path_eval_npy = os.path.join(eval_seq_dir_path, seq + "_path_eval.npy")

        # only for plotting function update!
        if args.replot:
            path_eval_dict = np.load(path_eval_npy, allow_pickle=True).item()

            save_path_img(qolo_pose2d, path_eval_dict, eval_res_dir, seq)

            print("Replot images!")
        else:
            if (not os.path.exists(path_eval_npy)) or (args.overwrite):

                path_eval_dict = compute_time_path(
                    qolo_twist, qolo_pose2d, data_params["goal_dist"]
                )

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
