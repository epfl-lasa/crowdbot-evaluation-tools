# -*-coding:utf-8 -*-
"""
@File    :   crowdbot_data.py
@Time    :   2021/10/26
@Author  :   Yujie He
@Version :   1.1
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import os
import yaml
import numpy as np
from pathlib import Path


class CrowdBotData(object):
    DEFAULT_CONFIG_PATH = os.path.join(Path(__file__).parents[1], 'data/data.yaml')

    def __init__(self, config=DEFAULT_CONFIG_PATH):
        self.config = config
        data_config = self.read_yaml()
        self.bagbase_dir = data_config['bagbase_dir']
        self.outbase_dir = data_config['outbase_dir']

    def read_yaml(self):
        with open(self.config, encoding='utf-8') as f:
            data = yaml.load(f.read(), Loader=yaml.FullLoader)
        return data

    def write_yaml(self, data):
        """
        :param yaml_path:
        :param data:
        :param encoding:
        """
        with open(self.config, 'w', encoding='utf-8') as f:
            yaml.dump(data, stream=f, allow_unicode=True)


class CrowdBotDatabase(CrowdBotData):
    def __init__(self, args, config=None):

        if config is None:
            super(CrowdBotDatabase, self).__init__()
        else:
            super(CrowdBotDatabase, self).__init__(config)
        data_config = self.read_yaml()
        self.bagbase_dir = data_config['bagbase_dir']
        self.outbase_dir = data_config['outbase_dir']

        data_processed = args.folder + "_processed"
        base_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
        # data_processed_dir = os.path.join(base_folder, "data", data_processed)
        data_processed_dir = os.path.join(self.outbase_dir, data_processed)

        # lidars/
        self.lidar_dir = os.path.join(data_processed_dir, "lidars")

        # alg_res/
        self.dets_dir = os.path.join(data_processed_dir, "alg_res", "detections")
        self.trks_dir = os.path.join(data_processed_dir, "alg_res", "tracks")

        # source_data/[tf_qolo/pose/twist/acc]
        self.source_data_dir = os.path.join(data_processed_dir, "source_data")

        self.metrics_dir = os.path.join(data_processed_dir, "metrics")

        # media/[viz_imgs/videos]
        self.imgs_dir = os.path.join(data_processed_dir, "media", "viz_imgs")
        self.video_dir = os.path.join(data_processed_dir, "media", "videos")

        # filter sequence dir from self.lidar_dir (*_stamped.npy in the same folder)
        self.seqs = [
            f
            for f in os.listdir(self.lidar_dir)
            if os.path.isdir(os.path.join(self.lidar_dir, f))
        ]
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


# filter the files with specific extensions
# https://newbedev.com/list-files-only-in-the-current-directory
# https://stackoverflow.com/questions/2225564/get-a-filtered-list-of-files-in-a-directory/2225582
# https://www.kite.com/python/answers/how-to-filter-file-types-in-a-directory-in-python
# ENHANCEMENT: consider using fnmatch.filter(os.listdir(path), "*.ext")
def bag_file_filter(f):
    if f[-4:] in [".bag"]:
        return True
    else:
        return False
