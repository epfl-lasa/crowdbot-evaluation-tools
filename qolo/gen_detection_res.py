# -*-coding:utf-8 -*-
'''
@File    :   gen_detection_res.py
@Time    :   2021/10/20
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''


import os
import argparse
import numpy as np

from lidar_det.detector import DetectorWithClock


if __name__ == "__main__":
    # ckpt_path = "/globalwork/jia/share/JRDB_cvpr21_workshop/logs/unet_bl_voxel_jrdb_0.05_0.1_20210519_232859/ckpt/ckpt_e40.pth"
    # base_dir = "/globalwork/datasets/crowdbot/qolo_market_data"
    # lidar_dir = os.path.join(base_dir, "lidars")

    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', required=True, type=str,
                        help='different subfolder in rosbag/ dir')
    # ckpt_e40_train_val.pth | ckpt_e40_train.pth
    parser.add_argument('--model', default='ckpt_e40_train.pth', 
                        type=str, help='checkpoints filename')
    args = parser.parse_args()

    # model checkpoints
    ckpt_path = os.path.join(args.base, "qolo", "Person_MinkUNet", "models", args.model)
    detector = DetectorWithClock(ckpt_path)

    # source: lidar data in data/xxxx_processed/lidars
    data_processed = args.folder + "_processed"
    data_processed_folder = os.path.join(args.base, args.data, data_processed)
    lidar_file_folder = os.path.join(data_processed_folder, "lidars")
    seqs = os.listdir(lidar_file_folder)

    # destination: generated detection data in data/xxxx_processed/detections
    detection_file_folder = os.path.join(data_processed_folder, "detections")

    print("Starting detection from {} lidar sequences!".format(len(seqs)))

    counter = 0
    for seq in seqs:
        #print(seq)
        counter += 1
        print("({}/{}): {}".format(counter, len(seqs), seq))
        frames = os.listdir(os.path.join(lidar_file_folder, seq))
        frames.sort()

        # seq dest: data/xxxx_processed/detections/seq
        det_seq_dir = os.path.join(detection_file_folder, seq)

        # os.makedirs(det_seq_dir, exist_ok=True)
        if not os.path.exists(det_seq_dir):
            os.makedirs(det_seq_dir)

            for frame in frames:
                with open(os.path.join(lidar_file_folder, seq, frame), "rb") as f:
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
