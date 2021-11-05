# -*-coding:utf-8 -*-
'''
@File    :   crowdbot_data.py
@Time    :   2021/10/26
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

import os
import numpy as np

class AllFrames(object):
    # "./data/qolo_market_data"
    #def __init__(self, data_dir="/home/crowdbot/Documents/yujie/crowdbot_tools/data"):
    def __init__(self, args):
        data_processed = args.folder + "_processed"
        data_processed_dir = os.path.join(args.base, args.data, data_processed)
        self.lidar_dir = os.path.join(data_processed_dir, "lidars")
        self.dets_dir = os.path.join(data_processed_dir, "detections")
        self.trks_dir = os.path.join(data_processed_dir, "tracks")
        # image and video folder for visualization
        self.imgs_dir = os.path.join(data_processed_dir, "viz_imgs")
        self.video_dir = os.path.join(data_processed_dir, "videos")
        self.pose_dir = os.path.join(data_processed_dir, "pose_stamped")
        self.metrics_dir = os.path.join(data_processed_dir, "metrics")

        # store _stamped.npy in the same folder
        self.seqs = [f for f in os.listdir(self.lidar_dir) if f.split('.')[-1] != 'npy']
        self.seqs.sort()
        self.frames = []
        for seq in self.seqs:
            frs = os.listdir(os.path.join(self.lidar_dir, seq))
            frs.sort()
            self.frames.append(frs)

    def nr_seqs(self):
        return len(self.seqs)

    def nr_frames(self, sq_idx):
        return len(self.frames[sq_idx])

    def __getitem__(self, sq_fr_idx):
        sq_idx, fr_idx = sq_fr_idx

        assert sq_idx < self.nr_seqs(), (
            "Sequence index out of range. "
            f"Requested {sq_idx}, maximum {self.nr_seqs()}."
        )
        assert fr_idx < self.nr_frames(sq_idx), (
            "Frame index out of range. "
            f"Requested {fr_idx}, maximum {self.nr_frames(sq_idx)}."
        )

        seq = self.seqs[sq_idx]
        fr = self.frames[sq_idx][fr_idx]

        l_path = os.path.join(self.lidar_dir, seq, fr)
        lidar = np.load(l_path) if os.path.isfile(l_path) else None
        lidar = lidar.T

        dets, dets_conf = None, None
        d_path = os.path.join(self.dets_dir, seq, fr.replace("nby", "txt"))
        if os.path.isfile(d_path):
            dets = np.loadtxt(d_path, dtype=np.float32, delimiter=",")
            dets_conf = dets[:, -1]  # sorted in descending order
            dets = dets[:, :-1]

        t_path = os.path.join(self.trks_dir, seq, fr.replace("nby", "txt"))
        trks = (
            np.loadtxt(t_path, dtype=np.float32, delimiter=",")
            if os.path.isfile(t_path)
            else None
        )

        return lidar, dets, dets_conf, trks

# yujie: filter the files with specific extensions
# https://newbedev.com/list-files-only-in-the-current-directory
# https://stackoverflow.com/questions/2225564/get-a-filtered-list-of-files-in-a-directory/2225582
# https://www.kite.com/python/answers/how-to-filter-file-types-in-a-directory-in-python
# TODO: consider using fnmatch.filter(os.listdir(path), "*.ext")
def bag_file_filter(f):
    if f[-4:] in ['.bag']:
        return True
    else:
        return False
