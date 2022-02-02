#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   pose2d2npy.py
@Date created  :   2021/11/06
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow to extract Pose2D msg from rosbag.
"""
# =============================================================================
"""
TODO:
1. should specify which part of pose/odom data is saved instead of saving two many zeros
"""
# =============================================================================

import os
import sys
import argparse

import numpy as np

import tf
import rosbag

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter
from qolo.utils.process_util import ts_to_sec

#%% Utility function for extraction pose2d from rosbag and apply interpolation
def extract_pose2d_from_rosbag(bag_file_path, args):
    """extract pose2d from rosbag without rosbag play"""
    pose2d_msg_sum = 0
    num_msgs_between_logs = 1000
    x_list, y_list, theta_list, t_list = [], [], [], []

    with rosbag.Bag(bag_file_path, "r") as bag:
        # Look for the topics that are available and save the total number of messages for each topic (useful for the progress bar)
        total_num_odom_msgs = bag.get_message_count(topic_filters=args.odom_topic)
        print(
            "Found odom topic: {} with {} messages".format(
                args.odom_topic, total_num_odom_msgs
            )
        )

        # Extract pose2d msg to file
        # !!!for topic, msg, t in bag.read_messages(topics=['/topics name'])
        for topic, msg, t in bag.read_messages():
            if topic == args.odom_topic:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                quat = msg.pose.pose.orientation
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [quat.x, quat.y, quat.z, quat.w]
                )
                ts = ts_to_sec(msg.header.stamp)
                x_list.append(x)
                y_list.append(y)
                theta_list.append(yaw)
                t_list.append(ts)

                if (
                    pose2d_msg_sum % num_msgs_between_logs == 0
                    or pose2d_msg_sum >= total_num_odom_msgs - 1
                ):
                    print(
                        "Pose2D messages: {} / {}".format(
                            pose2d_msg_sum + 1, total_num_odom_msgs
                        )
                    )
                pose2d_msg_sum += 1

    x_np = np.asarray(x_list, dtype=np.float32)
    y_np = np.asarray(y_list, dtype=np.float32)
    theta_np = np.asarray(theta_list, dtype=np.float32)
    t_np = np.asarray(t_list, dtype=np.float64)

    pose2d_stamped_dict = {"timestamp": t_np, "x": x_np, "y": y_np, "theta": theta_np}
    print("Current pose2d extracted!")
    return pose2d_stamped_dict


#%% main file
if __name__ == "__main__":
    # base_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="nocam_rosbags",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--odom_topic",
        default="/t265/odom/sample",
        type=str,
        help="topic for qolo odom",
    )
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    # source: rosbag data in data/rosbag/xxxx
    # rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: pose2d data in data/xxxx_processed/source_data/pose2d
    pose2d_dir = os.path.join(cb_data.source_data_dir, "pose2d")
    if not os.path.exists(pose2d_dir):
        os.makedirs(pose2d_dir)

    print("Starting extracting pose2d from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        pose2d_filepath = os.path.join(pose2d_dir, bag_name + "_pose2d.npy")

        if (not os.path.exists(pose2d_filepath)) or (args.overwrite):
            pose2d_stamped_dict = extract_pose2d_from_rosbag(bag_path, args)
            np.save(pose2d_filepath, pose2d_stamped_dict)
        else:
            print("Detecting the generated {} already existed!".format(pose2d_filepath))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue

    print("Finish extracting all pose2d msg!")

"""
ref: https://github.com/uzh-rpg/rpg_e2vid/blob/master/scripts/extract_events_from_rosbag.py
http://wiki.ros.org/rosbag/Code%20API#Python_API
"""
