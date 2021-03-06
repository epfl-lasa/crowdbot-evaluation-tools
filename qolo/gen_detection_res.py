#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   gen_detection_res.py
@Date created  :   2021/10/20
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides pedestrian detection results based on Person_MinkUNet framework
"""
# =============================================================================

import os
import argparse
import numpy as np

from qolo.core.crowdbot_data import CrowdBotDatabase
from lidar_det.detector import DetectorWithClock


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
    # ckpt_e40_train_val.pth | ckpt_e40_train.pth
    parser.add_argument(
        "--model", default="ckpt_e40_train.pth", type=str, help="checkpoints filename"
    )
    parser.add_argument(
        "--save_num",
        type=int,
        default=100,
        help="numbers of detected bbox to save",
    )
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    save_bbox_num = args.save_num

    # model checkpoints
    ckpt_path = os.path.join(
        os.path.dirname(__file__), "Person_MinkUNet", "models", args.model
    )
    detector = DetectorWithClock(ckpt_path)

    # source: lidar data in data/xxxx_processed/lidars
    lidar_file_dir = cb_data.lidar_dir

    seq_num = cb_data.nr_seqs()
    print("Starting detection from {} lidar sequences!".format(seq_num))

    counter = 0
    for seq_idx in range(seq_num):

        seq = cb_data.seqs[seq_idx]

        counter += 1
        print("({}/{}): {}".format(counter, seq_num, seq))
        frames = os.listdir(os.path.join(lidar_file_dir, seq))
        frames.sort()

        # seq dest: data/xxxx_processed/detections/seq
        if args.save_raw:
            det_seq_dir = os.path.join(cb_data.dets_dir, seq)
        dnpy_all_path = os.path.join(cb_data.dets_dir, seq + '.npy')

        # os.makedirs(det_seq_dir, exist_ok=True)
        if os.path.exists(dnpy_all_path) and not args.overwrite:
            print("{} detection results already generated!!!".format(seq))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue
        else:
            if args.save_raw and (not os.path.exists(det_seq_dir)):
                os.makedirs(det_seq_dir)

            out_det_all = dict()

            for fr_idx, frame in enumerate(frames):
                with open(os.path.join(lidar_file_dir, seq, frame), "rb") as f:
                    pc = np.load(f)

                boxes, scores = detector(pc.T)

                # save top 100 bbox with score
                if len(scores) >= save_bbox_num:
                    boxes_, scores_ = boxes[:100, :], scores[:100]
                else:
                    boxes_, scores_ = boxes, scores

                out_det = np.concatenate((boxes_, scores_[:, np.newaxis]), axis=1)
                if args.save_raw:
                    np.savetxt(
                        os.path.join(det_seq_dir, frame.replace("nby", "txt")),
                        out_det,
                        delimiter=",",
                    )
                out_det_all.update({fr_idx: out_det})
            np.save(
                dnpy_all_path,
                out_det_all,
            )
            print(detector)
            detector.reset()
