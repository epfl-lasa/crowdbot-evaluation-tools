# -*-coding:utf-8 -*-
"""
@File    :   gen_viz_img.py
@Time    :   2021/10/20
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import os
import argparse
import numpy as np
from mayavi import mlab
import cv2
from crowdbot_data import AllFrames
from viz_util import plot_frame

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

    allf = AllFrames(args)

    for seq_idx in range(allf.nr_seqs()):

        seq = allf.seqs[seq_idx]
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)
            )
        )

        # seq dest: data/xxxx_processed/viz_imgs/seq
        img_seq_dir = os.path.join(allf.imgs_dir, seq)

        if not os.path.exists(img_seq_dir):
            print("Images will be saved in {}".format(img_seq_dir))
            os.makedirs(img_seq_dir, exist_ok=True)

            # generate image every 2 frames
            for fr_idx in range(0, allf.nr_frames(seq_idx), 2):
                lidar, dets, dets_conf, trks = allf[seq_idx, fr_idx]
                figpath = os.path.join(img_seq_dir, "{0:04d}.png".format(fr_idx))
                fig = plot_frame(lidar, trks, figpath)
                mlab.close(fig)
        else:
            print("{} images already generated!!!".format(allf.seqs[seq_idx]))
            continue
