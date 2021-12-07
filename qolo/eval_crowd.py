#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   eval_crowd.py
@Date created  :   2021/11/02
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides the evaluation pipeline includeing the computation of
crowd-related metrics (min_dist, crowd density within 5/10m, nomalized
proximity and corresponding visualization.
The emulation results is exported with suffix as "_crowd_eval.npy".
"""
# =============================================================================
"""
TODO:
1. compare with detected pedestrain from the rosbag!
2. speed up `compute_crowd_metrics`
3. use try/except when loading files
"""
# =============================================================================

import os
import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from crowdbot_data import CrowdBotDatabase
from eval_res_plot import save_cd_img, save_md_img
from metric_crowd import compute_crowd_metrics, compute_norm_prox


#%% main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate crowd characteristics")

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
    parser.add_argument(
        "--goal_dist", default=30.0, type=float, help="The length to travel in the test"
    )
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

        # load _path_eval.npy
        path_eval_filepath = os.path.join(eval_res_dir, seq + "_path_eval.npy")
        if not os.path.exists(path_eval_filepath):
            print("ERROR: Please extract twist_stamped by using eval_qolo_path.py")
        path_eval_dict = np.load(path_eval_filepath, allow_pickle=True).item()

        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        crowd_eval_npy = os.path.join(eval_res_dir, seq + "_crowd_eval.npy")

        # only for plotting function update!
        if args.replot:
            crowd_eval_dict = np.load(crowd_eval_npy, allow_pickle=True).item()

            # figure1: crowd density
            save_cd_img(crowd_eval_dict, path_eval_dict, eval_res_dir, seq)

            # figure2: min. dist.
            save_md_img(crowd_eval_dict, path_eval_dict, eval_res_dir, seq)

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
                    "within_det3",
                    "within_det5",
                    "within_det10",
                    "crowd_density3",
                    "crowd_density5",
                    "crowd_density10",
                    "min_dist",
                )
                dtypes = (
                    np.uint8,
                    np.uint8,
                    np.uint8,
                    np.uint8,
                    np.float32,
                    np.float32,
                    np.float32,
                    np.float32,
                )

                crowd_eval_list_dict = {k: [] for k in attrs}

                num_msgs_between_logs = 100
                nr_frames = cb_data.nr_frames(seq_idx)

                for fr_idx in range(nr_frames):

                    _, _, _, trks = cb_data[seq_idx, fr_idx]

                    metrics = compute_crowd_metrics(trks)

                    if fr_idx % num_msgs_between_logs == 0 or fr_idx >= nr_frames - 1:
                        print(
                            "Seq {}/{} - Frame {}/{}: Crowd density within 3/5/10m: {:.2f} / {:.2f} / {:.2f}".format(
                                seq_idx + 1,
                                cb_data.nr_seqs(),
                                fr_idx + 1,
                                nr_frames,
                                metrics[4],
                                metrics[5],
                                metrics[6],
                            )
                        )

                    # update value for each attr
                    for idx, val in enumerate(metrics):
                        crowd_eval_list_dict[attrs[idx]].append(val)

                # evaluate the metrics through the sequence
                crowd_eval_dict = {
                    name: np.asarray(crowd_eval_list_dict[attrs[idx]], dtype=dtype)
                    for idx, (name, dtype) in enumerate(zip(attrs, dtypes))
                }

                # normalized proximity
                crowd_eval_dict.update(
                    {"normalized_proximity": compute_norm_prox(metrics[5])}
                )

                # avg_min_dict = np.average(crowd_eval_dict['min_dict'])
                for attr in attrs:
                    avg_attr = "avg_" + attr
                    crowd_eval_dict.update(
                        {avg_attr: np.average(crowd_eval_dict[attr])}
                    )

                # max and std of crowd density within 5m
                crowd_eval_dict.update(
                    {'max_crowd_density5': np.max(crowd_eval_dict['crowd_density5'])}
                )
                crowd_eval_dict.update(
                    {'std_crowd_density5': np.std(crowd_eval_dict['crowd_density5'])}
                )

                # other attributes
                crowd_eval_dict.update({"timestamp": ts})

                np.save(crowd_eval_npy, crowd_eval_dict)

                if args.save_img:
                    # figure1: crowd density
                    save_cd_img(crowd_eval_dict, path_eval_dict, eval_res_dir, seq)

                    # figure2: min. dist.
                    save_md_img(crowd_eval_dict, path_eval_dict, eval_res_dir, seq)

            else:
                print(
                    "Detecting the generated {} already existed!".format(crowd_eval_npy)
                )
                print(
                    "Will not overwrite. If you want to overwrite, use flag --overwrite"
                )
                continue

    print("Finish crowd evaluation!")
