# -*-coding:utf-8 -*-
'''
@File    :   gen_tracking_res.py
@Time    :   2021/10/19
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''


import os
import sys
# export PYTHONPATH="${PYTHONPATH}:${PWD}/qolo/AB3DMOT"
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "AB3DMOT"))
import argparse
import numpy as np

from crowdbot_data import AllFrames
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
    parser = argparse.ArgumentParser(description='convert data from rosbag')
        
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    args = parser.parse_args()

    min_conf = 0.5

    allf = AllFrames(args)

    counter = 0
    seq_num = allf.nr_seqs()

    for seq_idx in range(seq_num):
        tracker = AB3DMOT(max_age=2, min_hits=3)

        counter += 1
        print("({}/{}): {} frames".format(counter, seq_num, allf.nr_frames(seq_idx)))

        # seq dest: data/xxxx_processed/alg_res/tracks/seq
        trk_seq_dir = os.path.join(allf.trks_dir, allf.seqs[seq_idx])
        
        if not os.path.exists(trk_seq_dir):
            for fr_idx in range(allf.nr_frames(seq_idx)):
                lidar, dets, dets_conf, _ = allf[seq_idx, fr_idx]

                dets = dets[dets_conf > min_conf]
                dets = reorder(dets)
                trk_input = {"dets": dets, "info": np.zeros_like(dets)}
                trks = tracker.update(trk_input)
                trks = reorder_back(trks)

                f_path = os.path.join(trk_seq_dir, 
                                     allf.frames[seq_idx][fr_idx]
                                     .replace("nby", "txt")
                                     )
                os.makedirs(trk_seq_dir, exist_ok=True)
                np.savetxt(f_path, trks, delimiter=",")
        else:
            print("{} tracking results already generated!!!".format(allf.seqs[seq_idx]))
            continue
