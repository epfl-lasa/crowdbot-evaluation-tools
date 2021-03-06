#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   gen_viz_img.py
@Date created  :   2021/10/26
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow based on open3d to render resulted image with
extracted pointcloud, robot pose, and pedestrian detection & tracking results.
"""
# =============================================================================
"""
TODO:
1. test support with pcd!!!
"""
# =============================================================================


import os
import argparse
import numpy as np
from qolo.core.crowdbot_data import CrowdBotDatabase
from qolo.utils.o3d_util import plot_robot_frame_o3d, plot_world_frame_o3d

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="generate visualization image rendered with Open3D"
    )

    parser.add_argument(
        "-f",
        "--folder",
        default="1203_test",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "-s",
        "--step",
        default=4,
        type=int,
        help="visualization step",
    )
    parser.add_argument(
        "--consider_pose",
        dest="consider_pose",
        action="store_true",
        help="Consider pose transformation when plotting",
    )
    parser.set_defaults(consider_pose=True)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    step_viz = args.step
    consider_pose = args.consider_pose

    for seq_idx in range(cb_data.nr_seqs()):

        seq = cb_data.seqs[seq_idx]
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, cb_data.nr_seqs(), seq, cb_data.nr_frames(seq_idx)
            )
        )

        # seq dest: data/xxxx_processed/media/img_o3d/seq
        img3d_dir = os.path.join(cb_data.media_dir, "img_o3d")
        img_seq_dir = os.path.join(img3d_dir, seq)

        if not os.path.exists(img_seq_dir) or args.overwrite:
            print("Images will be saved in {}".format(img_seq_dir))
            os.makedirs(img_seq_dir, exist_ok=True)

            # pose_stamped based visualization
            if consider_pose:
                tf_qolo_dir = os.path.join(cb_data.source_data_dir, "tf_qolo")
                lidar_stamped_path = os.path.join(
                    tf_qolo_dir, seq + "_tfqolo_sampled.npy"
                )
                lidar_pose_stamped = np.load(
                    lidar_stamped_path, allow_pickle=True
                ).item()

            # generate image every step_viz frames
            for fr_idx in range(0, cb_data.nr_frames(seq_idx), step_viz):
                # input a series of poses
                if consider_pose:
                    trans = lidar_pose_stamped["position"][: fr_idx + 1]
                    rot_quat = lidar_pose_stamped["orientation"][: fr_idx + 1]

                lidar, dets, dets_conf, trks = cb_data[seq_idx, fr_idx]
                figpath = os.path.join(img_seq_dir, "{0:04d}.png".format(fr_idx))
                print(
                    "({}/{}): {:1f}% remaining".format(
                        fr_idx // step_viz + 1,
                        cb_data.nr_frames(seq_idx) // step_viz + 1,
                        100
                        - (50 * fr_idx + 100) / (cb_data.nr_frames(seq_idx) // 2 + 1),
                    )
                )

                if consider_pose:
                    plot_world_frame_o3d(lidar.T, (trans, rot_quat), trks, figpath)
                else:
                    plot_robot_frame_o3d(lidar.T, trks, figpath)
        else:
            print("{} images already generated!!!".format(cb_data.seqs[seq_idx]))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue
