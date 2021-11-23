# -*-coding:utf-8 -*-
"""
@File    :   gen_detection_res.py
@Time    :   2021/10/20
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""


import os
import argparse
import numpy as np

from crowdbot_data import CrowdBotDatabase
from lidar_det.detector import DetectorWithClock


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    # parser.add_argument(
    #     "-b",
    #     "--base",
    #     default="/home/crowdbot/Documents/yujie/crowdbot_tools",
    #     type=str,
    #     help="base folder, i.e., the path of the current workspace",
    # )
    # parser.add_argument(
    #     "-d",
    #     "--data",
    #     default="data",
    #     type=str,
    #     help="data folder, i.e., the name of folder that stored extracted raw data and processed data",
    # )
    parser.add_argument(
        "-f",
        "--folder",
        default="shared_test",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    # ckpt_e40_train_val.pth | ckpt_e40_train.pth
    parser.add_argument(
        "--model", default="ckpt_e40_train.pth", type=str, help="checkpoints filename"
    )
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args)

    # model checkpoints
    ckpt_path = os.path.join("qolo", "Person_MinkUNet", "models", args.model)
    detector = DetectorWithClock(ckpt_path)

    # source: lidar data in data/xxxx_processed/lidars
    lidar_file_dir = cb_data.lidar_dir

    seq_num = cb_data.nr_seqs()
    print("Starting detection from {} lidar sequences!".format(seq_num))

    counter = 0
    for seq_idx in range(seq_num):

        seq = cb_data.seqs[seq_idx]

        # print(seq)
        counter += 1
        print("({}/{}): {}".format(counter, seq_num, seq))
        frames = os.listdir(os.path.join(lidar_file_dir, seq))
        frames.sort()

        # seq dest: data/xxxx_processed/detections/seq
        det_seq_dir = os.path.join(cb_data.dets_dir, seq)

        # os.makedirs(det_seq_dir, exist_ok=True)
        if not os.path.exists(det_seq_dir):
            os.makedirs(det_seq_dir)

            for frame in frames:
                with open(os.path.join(lidar_file_dir, seq, frame), "rb") as f:
                    pc = np.load(f)

                boxes, scores = detector(pc.T)

                out = np.concatenate((boxes, scores[:, np.newaxis]), axis=1)
                np.savetxt(
                    os.path.join(det_seq_dir, frame.replace("nby", "txt")),
                    out,
                    delimiter=",",
                )

            print(detector)
            detector.reset()
        else:
            print("{} detection results already generated!!!".format(seq))
            continue
