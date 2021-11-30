#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   gen_viz_img_old.py
@Date created  :   2021/10/20
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow based on mayavi to render resulted image with
extracted pointcloud, robot pose, and pedestrian detection & tracking results.

(Deprecated due to inconvenient environment configuration of mayavi!!!)
"""
# =============================================================================

import os
import argparse
import numpy as np
from mayavi import mlab
import cv2
from crowdbot_data import CrowdBotDatabase
from viz_util_old import plot_frame

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

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
        default="nocam_rosbags",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    for seq_idx in range(cb_data.nr_seqs()):

        seq = cb_data.seqs[seq_idx]
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, cb_data.nr_seqs(), seq, cb_data.nr_frames(seq_idx)
            )
        )

        # seq dest: data/xxxx_processed/viz_imgs/seq
        img_seq_dir = os.path.join(cb_data.img3d_dir, seq)

        if not os.path.exists(img_seq_dir):
            print("Images will be saved in {}".format(img_seq_dir))
            os.makedirs(img_seq_dir, exist_ok=True)

            # generate image every 2 frames
            for fr_idx in range(0, cb_data.nr_frames(seq_idx), 2):
                lidar, dets, dets_conf, trks = cb_data[seq_idx, fr_idx]
                figpath = os.path.join(img_seq_dir, "{0:04d}.png".format(fr_idx))
                fig = plot_frame(lidar, trks, figpath)
                mlab.close(fig)
        else:
            print("{} images already generated!!!".format(cb_data.seqs[seq_idx]))
            continue
