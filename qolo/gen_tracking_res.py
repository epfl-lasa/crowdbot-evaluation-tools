#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   gen_tracking_res.py
@Date created  :   2021/10/19
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides pedestrian tracking results from detection results based on
AB3DMOT framework
"""
# =============================================================================


import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "AB3DMOT"))
import argparse
import numpy as np

from qolo.core.crowdbot_data import CrowdBotDatabase
from AB3DMOT.AB3DMOT_libs.model import AB3DMOT


def reorder(boxes):
    # from x, y, z,  l,  w, h, theta (lidar frame: x-forward, y-left, z-up)
    # to   h, w, l, -y, -z, x, theta (cam frame: z-forward, x-right, y-down)
    inds = [5, 4, 3, 1, 2, 0, 6]
    boxes = boxes[:, inds]
    boxes[:, 3] *= -1
    boxes[:, 4] *= -1
    return boxes


def reorder_back(boxes):
    # from h, w, l, -y, -z, x, theta, ID
    # to   x, y, z,  l,  w, h, theta, ID
    inds = [5, 3, 4, 2, 1, 0, 6, 7]
    boxes = boxes[:, inds]
    boxes[:, 1] *= -1
    boxes[:, 2] *= -1
    return boxes


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="1203_test",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing output (default: false)",
    )
    parser.set_defaults(overwrite=False)
    parser.add_argument(
        "--save_raw",
        dest="save_raw",
        action="store_true",
        help="Whether to save raw data of detection results (default: false)",
    )
    parser.set_defaults(save_raw=False)
    args = parser.parse_args()

    min_conf = 0.5

    cb_data = CrowdBotDatabase(args.folder)

    counter = 0
    seq_num = cb_data.nr_seqs()

    for seq_idx in range(seq_num):
        tracker = AB3DMOT(max_age=2, min_hits=3)

        seq = cb_data.seqs[seq_idx]

        counter += 1
        print("({}/{}): {} frames".format(counter, seq_num, cb_data.nr_frames(seq_idx)))

        # seq dest: data/xxxx_processed/alg_res/tracks/seq
        if args.save_raw:
            trk_seq_dir = os.path.join(cb_data.trks_dir, seq)
        tnpy_all_path = os.path.join(cb_data.trks_dir, seq + '.npy')

        if not os.path.exists(tnpy_all_path) or args.overwrite:
            out_trk_all = dict()

            for fr_idx in range(cb_data.nr_frames(seq_idx)):
                _, dets, dets_conf, _ = cb_data[seq_idx, fr_idx]

                dets = dets[dets_conf > min_conf]
                dets = reorder(dets)
                trk_input = {"dets": dets, "info": np.zeros_like(dets)}
                trks = tracker.update(trk_input)
                trks = reorder_back(trks)

                if args.save_raw:
                    f_path = os.path.join(
                        trk_seq_dir,
                        cb_data.frames[seq_idx][fr_idx].replace("nby", "txt"),
                    )
                    os.makedirs(trk_seq_dir, exist_ok=True)
                    np.savetxt(f_path, trks, delimiter=",")
                out_trk_all.update({fr_idx: trks})
            np.save(
                tnpy_all_path,
                out_trk_all,
            )
        else:
            print("{} tracking results already generated!!!".format(seq))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue
