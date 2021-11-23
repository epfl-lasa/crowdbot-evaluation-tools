# -*-coding:utf-8 -*-
"""
@File    :   gen_source_data.py
@Time    :   2021/11/15
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import os
import sys
import argparse

import numpy as np

import tf
import rosbag

from crowdbot_data import CrowdBotDatabase, bag_file_filter
from process_util import interp_translation, compute_motion_derivative, ts_to_sec


#%% Utility function for single file
from pose2d2npy import extract_pose2d_from_rosbag
from twist2npy import extract_twist_from_rosbag, interp_twist
from tfqolo2npy import (
    extract_pose_from_rosbag,
    deduplicate_tf,
    interp_pose,
    find_round_angle,
    to_euler_zyx,
    compute_ang_vel,
)

"""
pose2d2npy:
    extract_pose2d_from_rosbag(bag_file_path, args)
twist2npy:
    extract_twist_from_rosbag(bag_file_path, args)
    interp_twist(twist_stamped_dict, target_dict)
tfqolo2npy:
    extract_pose_from_rosbag(bag_file_path)
    deduplicate_tf(qolo_tf)
    interp_pose(source_dict, interp_ts)
    find_round_angle(angle, degrees=False)
    to_euler_zyx(rot_quat, degrees=False)
    compute_ang_vel(rpy_list, hz=200.0)
"""

#%% main file
if __name__ == "__main__":
    base_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

    parser = argparse.ArgumentParser(description="convert all source data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="nocam_rosbags",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--twist_topic", default="/qolo/twist", type=str, help="topic for qolo twist"
    )
    parser.add_argument(
        "--gen_acc",
        dest="gen_acc",
        action="store_true",
        help="Whether to generate the acceleration (default: true)",
    )
    parser.set_defaults(gen_acc=True)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: twist data in data/xxxx_processed/source_data/twist
    twist_dir = os.path.join(cb_data.source_data_dir, "twist")
    if not os.path.exists(twist_dir):
        os.makedirs(twist_dir)
    tf_qolo_dir = os.path.join(cb_data.source_data_dir, "tf_qolo")
    if not os.path.exists(tf_qolo_dir):
        os.makedirs(tf_qolo_dir)
    pose2d_dir = os.path.join(cb_data.source_data_dir, "pose2d")
    if not os.path.exists(pose2d_dir):
        os.makedirs(pose2d_dir)

    print("Starting extracting twist_stamped from {} rosbags!".format(len(bag_files)))

    for idx, bf in enumerate(bag_files):
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        print("({}/{}): {}".format(idx + 1, len(bag_files), bag_path))

        # TODO:
        # 1. twist
        # 2. tfqolo
        # 3. pose2d
