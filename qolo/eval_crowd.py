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
2. use try/except when loading files
"""
# =============================================================================

import os
import argparse
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from qolo.core.crowdbot_data import (
    CrowdBotDatabase,
    CrowdbotExpParam,
    CROWDBOT_EVAL_TOOLKIT_DIR,
)
from qolo.utils.notebook_util import values2color_list
from qolo.utils.res_plot_util import save_cd_img, save_md_img, save_cd_img_single
from qolo.metrics.metric_crowd import compute_crowd_metrics, compute_norm_prox

# cross-zero checking: https://stackoverflow.com/a/29674950/7961693
def zero_crossing_check(data):
    crossing_idx = np.where(np.diff(np.signbit(data)))[0]
    less_than_zero = math.ceil(float(crossing_idx.shape[0]) / 2)
    return less_than_zero


#%% main function
if __name__ == "__main__":

    data_params_path = os.path.join(
        CROWDBOT_EVAL_TOOLKIT_DIR, "data", "data_params.yaml"
    )

    parser = argparse.ArgumentParser(description="Evaluate crowd characteristics")

    parser.add_argument(
        "-f",
        "--folder",
        default="0424_shared_control",
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
        path_eval_filepath = os.path.join(eval_res_dir, seq, seq + "_path_eval.npy")
        if not os.path.exists(path_eval_filepath):
            print("ERROR: Please extract twist_stamped by using eval_qolo_path.py")
        path_eval_dict = np.load(path_eval_filepath, allow_pickle=True).item()

        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        crowd_eval_npy = os.path.join(eval_res_dir, seq, seq + "_crowd_eval.npy")

        # only for plotting function update!
        if args.replot:
            crowd_eval_dict = np.load(crowd_eval_npy, allow_pickle=True).item()

            # recalculate virtual_collision
            min_dist_list = np.array(crowd_eval_dict["min_dist"])
            start_idx_ = path_eval_dict['start_idx']
            end_idx_ = path_eval_dict['end_idx']
            min_dist_list = min_dist_list[start_idx_ : end_idx_ + 1]
            # only consider the point that more than zero
            min_dist_list_ = min_dist_list[min_dist_list > 0]
            avg_min_dist = np.sum(min_dist_list_) / min_dist_list_.shape[0]

            # consider the number of crossing zero from positive to negative
            virtual_collision = zero_crossing_check(min_dist_list)

            crowd_eval_dict['avg_min_dist'] = avg_min_dist
            crowd_eval_dict['virtual_collision'] = virtual_collision
            print("virtual_collision:", crowd_eval_dict['virtual_collision'])
            np.save(crowd_eval_npy, crowd_eval_dict)

            # figure1: crowd density
            _, color_unique = values2color_list(
                [0, 1, 2],
                cmap_name='hot',
                given_values=[0.2, 0.55, 0.66],
                reverse=False,
            )

            save_cd_img(
                crowd_eval_dict,
                path_eval_dict,
                eval_res_dir,
                seq,
                fmt='pdf',
                add_title=False,
                use_serif=True,
                color_list=color_unique,
                color_vertical='navy',
            )
            # with single plots
            # save_cd_img_single(
            #     crowd_eval_dict,
            #     path_eval_dict,
            #     eval_res_dir,
            #     seq,
            #     dist=2.5,
            #     fmt='pdf',
            #     linecolor="navy",
            #     use_serif=True,
            # )

            # figure2: min. dist.
            save_md_img(crowd_eval_dict, path_eval_dict, eval_res_dir, seq)

            print("Replot images!")
        else:
            if (not os.path.exists(crowd_eval_npy)) or (args.overwrite):

                # timestamp can be read from lidars/ folder
                lidar_stamp_dir = os.path.join(cb_data.source_data_dir, "timestamp")
                stamp_file_path = os.path.join(lidar_stamp_dir, seq + "_stamped.npy")
                lidar_stamped = np.load(
                    stamp_file_path,
                    allow_pickle=True,
                ).item()
                ts = lidar_stamped.get("timestamp")

                # targeted metrics and correspoding dtype
                attrs = (
                    "all_det",
                    "within_det2_5",
                    "within_det3",
                    "within_det5",
                    "within_det10",
                    "crowd_density2_5",
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
                    np.uint8,
                    np.float32,
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

                    metrics = compute_crowd_metrics(
                        trks, virtual_radius=data_params["virtual_radius"]
                    )

                    if fr_idx % num_msgs_between_logs == 0 or fr_idx >= nr_frames - 1:
                        print(
                            "Seq {}/{} - Frame {}/{}: Crowd density within 2.5/3/5/10m: {:.2f} / {:.2f} / {:.2f} / {:.2f}".format(
                                seq_idx + 1,
                                cb_data.nr_seqs(),
                                fr_idx + 1,
                                nr_frames,
                                metrics[5],
                                metrics[6],
                                metrics[7],
                                metrics[8],
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

                # calculate the avg for each matrix except avg_min_dist
                for attr in attrs:
                    if attr == "min_dist":
                        continue
                    avg_attr = "avg_" + attr
                    crowd_eval_dict.update(
                        {avg_attr: np.average(crowd_eval_dict[attr])}
                    )

                # calculate avg_min_dist
                min_dist_list = np.array(crowd_eval_dict["min_dist"])
                start_idx_ = path_eval_dict['start_idx']
                end_idx_ = path_eval_dict['end_idx']
                min_dist_list = min_dist_list[start_idx_ : end_idx_ + 1]
                # only consider the point that more than zero
                min_dist_list_ = min_dist_list[min_dist_list > 0]
                avg_min_dist = np.sum(min_dist_list_) / min_dist_list_.shape[0]

                # consider the number of crossing zero from positive to negative
                virtual_collision = zero_crossing_check(min_dist_list)

                crowd_eval_dict.update({'avg_min_dist': avg_min_dist})
                crowd_eval_dict.update({'virtual_collision': virtual_collision})
                print("virtual_collision:", crowd_eval_dict['virtual_collision'])

                # max and std of crowd density within 2.5m/5m
                crowd_eval_dict.update(
                    {
                        'max_crowd_density2_5': np.max(
                            crowd_eval_dict['crowd_density2_5']
                        )
                    }
                )
                crowd_eval_dict.update(
                    {
                        'std_crowd_density2_5': np.std(
                            crowd_eval_dict['crowd_density2_5']
                        )
                    }
                )

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
