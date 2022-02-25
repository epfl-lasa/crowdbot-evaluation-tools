#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   trk2traj.py
@Date created  :   2021/12/03
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides script to generate pedestrian trajectories from detection
results and save as pickle (or json) files
"""
# =============================================================================
"""
TODO:
1. update using multiprocessing to accelerate the extraction
"""
# =============================================================================

import os
import tqdm
import argparse
import numpy as np

from qolo.core.crowdbot_data import CrowdBotDatabase
from qolo.utils.geo_util import get_pc_tranform, yaw2quat
from qolo.utils.file_io_util import save_dict2pkl, save_dict2json


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="convert tracking result to pedestrian trajtories data"
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
        default="2021-04-10-10-41-17",
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
        help="Whether to overwrite existing output (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    consider_pose = args.consider_pose

    if args.seq is None or args.process_all:
        seqs = [cb_data.seqs[seq_idx] for seq_idx in range(cb_data.nr_seqs())]
    else:
        seqs = [args.seq]

    for seq_idx, seq in enumerate(seqs):

        sq_idx = cb_data.seqs.index(seq)
        seq_len = cb_data.nr_frames(sq_idx)

        print("({}/{}): {} with {} frames".format(seq_idx + 1, len(seqs), seq, seq_len))

        traj_dir = os.path.join(cb_data.ped_data_dir, "traj")
        if not os.path.exists(traj_dir):
            os.makedirs(traj_dir)
        traj_pkl_path = os.path.join(traj_dir, seq + '.pkl')
        traj_json_path = os.path.join(traj_dir, seq + '.json')

        traj_files_exist = os.path.exists(traj_pkl_path) and os.path.exists(
            traj_json_path
        )
        if traj_files_exist and not args.overwrite:
            print("{} trajectories already generated!!!".format(seq))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue

        if consider_pose:
            tf_qolo_dir = os.path.join(cb_data.source_data_dir, "tf_qolo")
            pose_stamp_path = os.path.join(tf_qolo_dir, seq + "_tfqolo_sampled.npy")
            pose_stamped = np.load(pose_stamp_path, allow_pickle=True).item()

        trans_array = pose_stamped["position"]
        quat_array = pose_stamped["orientation"]

        peds_dict = dict()

        with tqdm.tqdm(total=seq_len) as t:
            for fr_idx in range(seq_len):
                # for fr_idx in range(50):

                # print("# Frame {}".format(fr_idx))
                _, _, _, trks = cb_data[seq_idx, fr_idx]

                # bbox: [x, y, z, dx, dy, dz, heading, id]
                # https://stackoverflow.com/a/23596637/7961693
                # bbox = filter_detection_tracking_res(trks, dist=filter_dist, verbose=verbose)
                bbox = trks
                ids = bbox[:, -1]

                # original ids in descending order
                sort_idx = np.argsort(ids)
                ids = ids[sort_idx]
                bbox = bbox[sort_idx]

                if consider_pose:
                    bbox_trans = get_pc_tranform(
                        bbox[:, :3],
                        pos=trans_array[fr_idx, :],
                        quat=quat_array[fr_idx, :],
                    )

                for idx, id in enumerate(ids):
                    # print("{}/{}: pedestrian {}".format(idx + 1, len(ids), int(id)))
                    if id not in peds_dict.keys():
                        # print("New pedestrian {} detected".format(id))
                        id = int(id)
                        ped_dict = {
                            'start_idx': fr_idx,
                            'rel_pose_list': [(bbox[idx, :3]).tolist()],
                        }
                        if consider_pose:
                            abs_quat = yaw2quat(
                                bbox[idx, 6], base_quat=quat_array[fr_idx, :]
                            )
                            ped_dict.update({'abs_quat_list': [abs_quat.tolist()[0]]})
                            ped_dict.update(
                                {'abs_pose_list': [(bbox_trans[idx, :3]).tolist()]}
                            )
                        ped_dict.update({'length': len(ped_dict['rel_pose_list'])})
                        ped_dict.update(
                            {'end_idx': ped_dict['start_idx'] + ped_dict['length'] - 1}
                        )
                    elif id in peds_dict.keys():
                        # print("Update pedestrian {} detected".format(id))
                        ped_dict = peds_dict[id]
                        # print(ped_dict['rel_pose_list'])
                        ped_dict['rel_pose_list'].append((bbox[idx, :3]).tolist())
                        if consider_pose:
                            abs_quat = yaw2quat(
                                bbox[idx, 6], base_quat=quat_array[fr_idx, :]
                            )
                            ped_dict['abs_quat_list'].append(abs_quat.tolist()[0])
                            ped_dict['abs_pose_list'].append(
                                (bbox_trans[idx, :3]).tolist()
                            )
                        ped_dict['length'] += 1
                        ped_dict['end_idx'] += 1
                    # print(ped_dict['length'], ped_dict['end_idx'])

                    peds_dict.update({id: ped_dict})
                t.update()

        save_dict2pkl(peds_dict, traj_pkl_path)
        save_dict2json(peds_dict, traj_json_path)
