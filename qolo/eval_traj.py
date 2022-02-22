#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   eval_traj.py
@Date created  :   2022/02/22
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides pipeline to evaluate pededtrian performance
"""
# =============================================================================
"""
TODO:
1. consider angular when tracking: https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking/blob/master/main.py
"""
# =============================================================================

import os
import sys
import argparse
import numpy as np
import pandas as pd
import tqdm

from scipy.spatial.transform import Rotation as R
import quaternion as Q
import quaternion.quaternion_time_series as qseries

from qolo.core.crowdbot_data import CrowdBotDatabase
from qolo.utils.file_io_util import (
    save_dict2pkl,
    save_dict2json,
    load_json2dict,
    load_pkl2dict,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="generate visualization image rendered with Open3D"
    )

    parser.add_argument(
        "-f",
        "--folder",
        default="0410_rds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--seq",
        default="2021-04-10-10-38-36",  # 2021-04-10-10-38-36  2021-04-10-10-41-17
        type=str,
        help="specific sequence in the subfolder",
    )
    parser.add_argument(
        "--all",
        dest="process_all",
        action="store_true",
        help="Process all sequences and disable single sequences",
    )
    parser.set_defaults(process_all=False)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing output (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    if args.seq is None or args.process_all:
        seqs = [cb_data.seqs[seq_idx] for seq_idx in range(cb_data.nr_seqs())]
    else:
        seqs = [args.seq]

    for seq_idx, seq in enumerate(seqs):

        sq_idx = cb_data.seqs.index(seq)
        seq_len = cb_data.nr_frames(sq_idx)

        print("({}/{}): {} with {} frames".format(seq_idx + 1, len(seqs), seq, seq_len))

        # trajectory data
        traj_dir = os.path.join(cb_data.ped_data_dir, "traj")
        if not os.path.exists(traj_dir):
            sys.exit("Please use det2traj.py to extract pedestrian trajectories first!")
        traj_pkl_path = os.path.join(traj_dir, seq + '.pkl')
        traj_json_path = os.path.join(traj_dir, seq + '.json')

        traj_dict = load_pkl2dict(traj_pkl_path)
        # key: id
        # dict -> key: start_idx, rel_pose_list, abs_pose_list, length, end_idx
        traj_data = pd.DataFrame.from_dict(
            traj_dict, orient='index', columns=['start_idx', 'end_idx', 'length']
        )

        traj_data_200frs = traj_data[traj_data.length >= 200]

        print("Largest tracking length:", max(traj_data.length))
        print("Pedestrians tracked >=200 frames:", len(traj_data_200frs))

        # lidar timestamp
        stamp_file_path = os.path.join(
            cb_data.source_data_dir, "timestamp", seq + "_stamped.npy"
        )
        frame_ts = (
            np.load(
                stamp_file_path,
                allow_pickle=True,
            )
            .item()
            .get("timestamp")
        )

        # destination
        vel_dir = os.path.join(cb_data.ped_data_dir, "vel")
        if not os.path.exists(vel_dir):
            os.makedirs(vel_dir)
        vel_pkl_path = os.path.join(vel_dir, seq + '.pkl')
        vel_json_path = os.path.join(vel_dir, seq + '.json')

        vel_files_exist = os.path.exists(vel_pkl_path) and os.path.exists(vel_json_path)
        if vel_files_exist and not args.overwrite:
            print("{} velocities already generated!!!".format(seq))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue

        # generate velocity from pose and time interval
        ids = traj_data_200frs.index.values
        peds_vel_dict = dict()
        for idx, id in enumerate(ids):
            # dval_dt = np.gradient(motion_stamped_dict.get(val), frame_ts, axis=0)
            start_idx = traj_dict[id]['start_idx']
            end_idx = traj_dict[id]['end_idx']

            # linear
            xyz = np.array(traj_dict[id]['abs_pose_list'])
            # the first 2 column in 'lin_vel' are more important
            lin_vel = np.gradient(xyz, frame_ts[start_idx : end_idx + 1], axis=0)

            # angular (has large errors!)
            quat_xyzw = np.array(traj_dict[id]['abs_quat_list'])
            quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]
            quat_wxyz_ = Q.as_quat_array(quat_wxyz)
            ang_vel = qseries.angular_velocity(
                quat_wxyz_, frame_ts[start_idx : end_idx + 1]
            )
            ped_vel_dict = {
                "start_idx": start_idx,
                "end_idx": start_idx,
                "length": traj_dict[id]['length'],
                "lin_vel": lin_vel.tolist(),
                "ang_vel": ang_vel.tolist(),
            }
            peds_vel_dict.update({int(id): ped_vel_dict})  # int64 -> int

        save_dict2pkl(peds_vel_dict, vel_pkl_path)
        save_dict2json(peds_vel_dict, vel_json_path)
