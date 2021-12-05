#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   traj2pkl.py
@Date created  :   2021/12/03
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides ...
"""
# =============================================================================
"""
TODO:
1.
"""
# =============================================================================

import os
import argparse
import numpy as np
import tqdm
import pickle
import json

from scipy.spatial.transform import Rotation as R

from crowdbot_data import CrowdBotDatabase


def get_pc_tranform(pc, pos, quat):
    # pshape(pc)
    scipy_rot = R.from_quat(quat)
    rot_mat = scipy_rot.as_matrix()  # 3x3
    # pshape(rot_mat)
    rot_pc = np.matmul(rot_mat, pc.T)  #  (3x3) (3xn)
    # pshape(rot_pc)
    return rot_pc.T + pos


# save & load pickle
def save_pkl(peds_dict, pkl_path):
    # filehandler = open("./archive/test_traj.pkl","wb")
    filehandler = open(pkl_path, "wb")
    pickle.dump(peds_dict, filehandler)
    filehandler.close()


def load_pkl(pkl_path):
    # file = open("./archive/test_traj.pkl", 'rb')
    filehandler = open(pkl_path, 'rb')
    peds_dict = pickle.load(filehandler)
    filehandler.close()
    return peds_dict


# save & load json
def save_json(peds_dict, json_path):
    with open(json_path, "w") as f:
        # with open("./archive/test_traj.json", "w") as json_file:
        json.dump(peds_dict, f, indent=2)


def load_json(json_path):
    # f = open("./archive/test_traj.json")
    f = open(json_path)
    # convert string key into int
    peds_dict = json.load(
        f,
        object_hook=lambda d: {
            int(k) if k.lstrip('-').isdigit() else k: v for k, v in d.items()
        },
    )
    return peds_dict


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="generate visualization image rendered with Open3D"
    )

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
        default="shared_test",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--consider_pose",
        dest="consider_pose",
        action="store_true",
        help="Consider pose transformation when plotting",
    )
    parser.set_defaults(consider_pose=True)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)
    # seq_idx, fr_idx = 0, 2300

    consider_pose = args.consider_pose

    for seq_idx in range(cb_data.nr_seqs()):

        seq = cb_data.seqs[seq_idx]

        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, cb_data.nr_seqs(), seq, cb_data.nr_frames(seq_idx)
            )
        )

        if consider_pose:
            tf_qolo_dir = os.path.join(cb_data.source_data_dir, "tf_qolo")
            pose_stampe_path = os.path.join(tf_qolo_dir, seq + "_tfqolo_sampled.npy")
            lidar_pose_stamped = np.load(pose_stampe_path, allow_pickle=True).item()

        trans_array = lidar_pose_stamped["position"]
        quat_array = lidar_pose_stamped["orientation"]

        peds_dict = dict()
        seq_len = cb_data.nr_frames(seq_idx)

        with tqdm.tqdm(total=seq_len) as t:
            for fr_idx in range(seq_len):
                # for fr_idx in range(50):

                # print("# Frame {}".format(fr_idx))
                _, _, _, trks = cb_data[seq_idx, fr_idx]

                # bbox: [x, y, z, dx, dy, dz, heading]
                # https://stackoverflow.com/a/23596637/7961693
                # bbox = filter_detection_tracking_res(trks, dist=filter_dist, verbose=verbose)
                bbox = trks
                ids = bbox[:, -1]

                # origin id in descending order
                sort_idx = np.argsort(ids)
                ids = ids[sort_idx]
                bbox = bbox[sort_idx]

                if consider_pose:
                    bbox_trans = get_pc_tranform(
                        bbox[:, :3],
                        pos=trans_array[fr_idx, :],
                        quat=quat_array[fr_idx, :],
                    )
                # sing a library such as Pickle or do it yourself using Json strings.
                for idx, id in enumerate(ids):
                    if id not in peds_dict.keys():
                        # print("New pedestrian {} detected".format(id))
                        id = int(id)
                        ped_dict = {
                            'start_idx': fr_idx,
                            'rel_pose_list': [(bbox[idx, :3]).tolist()],
                        }
                        if consider_pose:
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
                            ped_dict['abs_pose_list'].append(
                                (bbox_trans[idx, :3]).tolist()
                            )
                        ped_dict['length'] += 1
                        ped_dict['end_idx'] += 1
                    # print(ped_dict['length'], ped_dict['end_idx'])

                    peds_dict.update({id: ped_dict})
                t.update()
