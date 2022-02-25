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
The module provides script to visualize the robot trajectory and the longest k
(default:3) trajectories of the pedestrians around Qolo robot.
"""
# =============================================================================
"""
TODO:
1. plot vx/vy or linear/angular velocity
"""
# =============================================================================

import os
import sys
import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from qolo.core.crowdbot_data import CrowdBotDatabase
from qolo.utils.geo_util import quat2yaw
from qolo.utils.file_io_util import (
    load_json2dict,
    load_pkl2dict,
)
from qolo.utils.res_plot_util import viz_qolo_ped_traj_full, viz_qolo_ped_traj_frame

color_list = ['navy', 'blue', 'slateblue', 'violet', 'skyblue']


def main():
    parser = argparse.ArgumentParser(
        description="visualize trajectories of pedestrians around Qolo"
    )

    parser.add_argument(
        "-f",
        "--folder",
        default="0410_rds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--seq",
        default="2021-04-10-11-11-30",  # 2021-04-10-10-38-36  2021-04-10-10-41-17
        type=str,
        help="specific sequence in the subfolder",
    )
    parser.add_argument(
        "--all",
        dest="process_all",
        action="store_true",
        help="Process all sequences and disable single sequences",
    )
    parser.set_defaults(process_all=False)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing output (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    if args.seq is None or args.process_all:
        seqs = [cb_data.seqs[seq_idx] for seq_idx in range(cb_data.nr_seqs())]
    else:
        seqs = [args.seq]

    for seq_idx, seq in enumerate(seqs):

        sq_idx = cb_data.seqs.index(seq)
        seq_len = cb_data.nr_frames(sq_idx)

        print("({}/{}): {} with {} frames".format(seq_idx + 1, len(seqs), seq, seq_len))

        # dest: path_img_path
        eval_res_dir = os.path.join(cb_data.metrics_dir)

        if not os.path.exists(eval_res_dir):
            print("Result images and npy will be saved in {}".format(eval_res_dir))
            os.makedirs(eval_res_dir, exist_ok=True)

        path_img_path = os.path.join(eval_res_dir, seq, seq + "_traj.png")
        # path_img_path = os.path.join(eval_res_dir, seq, seq + "_{}_traj.png".format(frame_id))
        plot_exist = os.path.exists(path_img_path)
        if plot_exist and not args.overwrite:
            print("{} plots already generated!!!".format(seq))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue

        # src 1: trajectory data
        traj_dir = os.path.join(cb_data.ped_data_dir, "traj")
        if not os.path.exists(traj_dir):
            sys.exit("Please use det2traj.py to extract pedestrian trajectories first!")
        # traj_pkl_path = os.path.join(traj_dir, seq + '.pkl')
        traj_json_path = os.path.join(traj_dir, seq + '.json')

        # src 2: qolo data
        tf_qolo_dir = os.path.join(cb_data.source_data_dir, "tf_qolo")
        pose_stamp_path = os.path.join(tf_qolo_dir, seq + "_tfqolo_sampled.npy")
        pose_stamped = np.load(pose_stamp_path, allow_pickle=True).item()

        trans_array = pose_stamped["position"]
        qolo_pose = {
            'x': trans_array[:, 0],
            'y': trans_array[:, 1],
            'init_ori': pose_stamped["orientation"],
        }

        # ped_traj_dict = load_pkl2dict(traj_pkl_path)
        ped_traj_dict = load_json2dict(traj_json_path)
        viz_qolo_ped_traj_full(
            path_img_path,
            qolo_pose,
            ped_traj_dict,
            ped_num=5,
            color_list=color_list,
        )


if __name__ == "__main__":
    main()
