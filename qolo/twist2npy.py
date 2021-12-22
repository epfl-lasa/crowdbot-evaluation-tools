#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   twist2npy.py
@Date created  :   2021/11/05
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow to extract twist msg from rosbag and compute
acceleration, jerk by taking derivatives.
"""
# =============================================================================

import os
import sys
import argparse

import numpy as np

import rosbag

from crowdbot_data import CrowdBotDatabase, bag_file_filter
from process_util import interp_translation, compute_motion_derivative, ts_to_sec


def check_zero_diff(src_list):

    total_len = src_list.shape[0]
    diff = np.diff(src_list)
    zero_diff = diff[diff == 0.0]
    zero_diff_len = zero_diff.shape[0]

    print("Zero_diff: {}/{}".format(zero_diff_len, total_len))


#%% Utility function for extraction twist_stamped from rosbag and apply interpolation
def extract_twist_from_rosbag(bag_file_path, args):
    """Extract twist_stamped from rosbag without rosbag play"""

    twist_msg_sum = 0
    num_msgs_between_logs = 100
    x_list, zrot_list, t_list = [], [], []

    with rosbag.Bag(bag_file_path, "r") as bag:
        # Look for the topics that are available and save the total number of messages for each topic
        total_num_twist_msgs = bag.get_message_count(topic_filters=args.twist_topic)
        if total_num_twist_msgs > 0:
            print(
                "Found twist topic: {} with {} messages".format(
                    args.twist_topic, total_num_twist_msgs
                )
            )

            for topic, msg, t in bag.read_messages():
                if topic == args.twist_topic:
                    x_vel = msg.twist.linear.x
                    zrot_vel = msg.twist.angular.z
                    ts_vel = ts_to_sec(msg.header.stamp)
                    x_list.append(x_vel)
                    zrot_list.append(zrot_vel)
                    t_list.append(ts_vel)

                    if (
                        twist_msg_sum % num_msgs_between_logs == 0
                        or twist_msg_sum >= total_num_twist_msgs - 1
                    ):
                        print(
                            "twist messages: {} / {}".format(
                                twist_msg_sum + 1, total_num_twist_msgs
                            )
                        )
                    twist_msg_sum += 1
        else:
            print("Warning: No twist detected => use commands (rds_to_gui) instead")
            print("linear vel in twist equals to corrected_linear in rds_to_gui")
            print("angular vel in twist equals to corrected_angular in rds_to_gui")
            total_num_twist_msgs = bag.get_message_count(
                topic_filters=args.to_gui_topic
            )
            if total_num_twist_msgs > 0:
                print(
                    "Found commands topic: {} with {} messages".format(
                        args.to_gui_topic, total_num_twist_msgs
                    )
                )
                for topic, msg, t in bag.read_messages():
                    if topic == args.to_gui_topic:
                        x_vel = msg.corrected_command.linear
                        zrot_vel = msg.corrected_command.angular
                        ts_vel = ts_to_sec(t)
                        x_list.append(x_vel)
                        zrot_list.append(zrot_vel)
                        t_list.append(ts_vel)

                        # print(ts_vel, x_vel, zrot_vel)

                        if (
                            twist_msg_sum % num_msgs_between_logs == 0
                            or twist_msg_sum >= total_num_twist_msgs - 1
                        ):
                            print(
                                "twist messages: {} / {}".format(
                                    twist_msg_sum + 1, total_num_twist_msgs
                                )
                            )
                        twist_msg_sum += 1
            else:
                print("Error: No commands topic found as well!!!")

    t_np = np.asarray(t_list, dtype=np.float64)
    x_np = np.asarray(x_list, dtype=np.float64)
    z_np = np.asarray(zrot_list, dtype=np.float64)

    twist_stamped_dict = {"timestamp": t_np, "x": x_np, "zrot": z_np}
    print("Current twist extracted!")
    return twist_stamped_dict


def interp_twist(twist_stamped_dict, target_dict):
    """Calculate interpolations for all states"""

    source_ts = twist_stamped_dict.get("timestamp")
    source_x = twist_stamped_dict.get("x")
    source_zrot = twist_stamped_dict.get("zrot")
    interp_ts = target_dict.get("timestamp")

    print("lidar timestamp", min(interp_ts), max(interp_ts))
    print("twist timestamp", min(source_ts), max(source_ts))

    if min(interp_ts) > max(source_ts):
        print("Warning: all interp_ts are larger than source_ts")
        interp_ts -= min(interp_ts) - min(source_ts)

    # method1: saturate the timestamp outside the range
    if min(interp_ts) < min(source_ts):
        interp_ts[interp_ts < min(source_ts)] = min(source_ts)
    if max(interp_ts) > max(source_ts):
        interp_ts[interp_ts > max(source_ts)] = max(source_ts)

    # method2: discard timestamps smaller or bigger than source
    # start_idx, end_idx = 0, -1
    # if min(interp_ts) < min(source_ts):
    #     start_idx = np.argmax(interp_ts[interp_ts - source_ts.min() < 0]) + 1
    # if max(interp_ts) > max(source_ts):
    #     end_idx = np.argmax(interp_ts[interp_ts - source_ts.max() <= 0]) + 1
    # interp_ts = interp_ts[start_idx:end_idx]

    interp_dict = {}
    interp_dict["timestamp"] = interp_ts
    interp_dict["x"] = interp_translation(source_ts, interp_ts, source_x)
    interp_dict["zrot"] = interp_translation(source_ts, interp_ts, source_zrot)
    return interp_dict, interp_ts


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="0424_mds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--twist_topic", default="/qolo/twist", type=str, help="topic for qolo twist"
    )
    parser.add_argument(
        "--to_gui_topic",
        default="/rds_to_gui",
        type=str,
        help="topic for qolo rds_to_gui",
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
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: twist data in data/xxxx_processed/source_data/twist
    twist_dir = os.path.join(cb_data.source_data_dir, "twist")
    if not os.path.exists(twist_dir):
        os.makedirs(twist_dir)

    print("Starting extracting twist_stamped from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        twist_raw_filepath = os.path.join(
            twist_dir, bag_name + "_twist_raw.npy"
        )  # _twist.npy
        # sample with lidar frame
        command_sampled_filepath = os.path.join(
            twist_dir, bag_name + "_qolo_command.npy"
        )  # _twist_sampled.npy

        if not os.path.exists(command_sampled_filepath) or (args.overwrite):
            if (not os.path.exists(twist_raw_filepath)) or (args.overwrite):
                twist_stamped_dict = extract_twist_from_rosbag(bag_path, args)
                np.save(twist_raw_filepath, twist_stamped_dict)
            else:
                print(
                    "Detecting the generated {} already existed!".format(
                        twist_raw_filepath
                    )
                )
                print("If you want to overwrite, use flag --overwrite")
                twist_stamped_dict = np.load(
                    twist_raw_filepath, allow_pickle=True
                ).item()

            # twist_sampled
            lidar_stamp_dir = os.path.join(cb_data.source_data_dir, "timestamp")
            stamp_file_path = os.path.join(lidar_stamp_dir, bag_name + "_stamped.npy")
            lidar_stamped = np.load(
                stamp_file_path,
                allow_pickle=True,
            ).item()

            twist_sampled_dict, _ = interp_twist(twist_stamped_dict, lidar_stamped)

            qolo_command_dict = {
                "timestamp": twist_sampled_dict["timestamp"],
                "x_vel": twist_sampled_dict['x'],
                "zrot_vel": twist_sampled_dict['zrot'],
            }

            print("Data from twist_sampled_dict")
            print(
                "NaN index in x_vel: {}\nNaN index in zrot_vel: {}".format(
                    np.squeeze(np.argwhere(np.isnan(twist_sampled_dict["x"]))),
                    np.squeeze(np.argwhere(np.isnan(twist_sampled_dict["zrot"]))),
                ),
            )

            # acc_sampled
            acc_sampled_dict = compute_motion_derivative(twist_sampled_dict)
            # existing NaN due to different duration of timestamp!
            # print(
            #     "NaN index in x_acc: {}\nNaN index in zrot_acc: {}".format(
            #         np.squeeze(np.argwhere(np.isnan(acc_sampled_dict["x"]))),
            #         np.squeeze(np.argwhere(np.isnan(acc_sampled_dict["zrot"]))),
            #     ),
            # )
            acc_sampled_dict["x"][np.isnan(acc_sampled_dict["x"])] = 0.0
            acc_sampled_dict["zrot"][np.isnan(acc_sampled_dict["zrot"])] = 0.0
            qolo_command_dict.update({"x_acc": acc_sampled_dict["x"]})
            qolo_command_dict.update({"zrot_acc": acc_sampled_dict["zrot"]})

            # jerk_sampled
            jerk_sampled_dict = compute_motion_derivative(acc_sampled_dict)
            # existing NaN due to different duration of timestamp!
            # print(
            #     "NaN index in x_jerk: {}\nNaN index in zrot_jerk: {}".format(
            #         np.squeeze(np.argwhere(np.isnan(jerk_sampled_dict["x"]))),
            #         np.squeeze(np.argwhere(np.isnan(jerk_sampled_dict["zrot"]))),
            #     ),
            # )
            jerk_sampled_dict["x"][np.isnan(jerk_sampled_dict["x"])] = 0.0
            jerk_sampled_dict["zrot"][np.isnan(jerk_sampled_dict["zrot"])] = 0.0
            qolo_command_dict.update({"x_jerk": jerk_sampled_dict["x"]})
            qolo_command_dict.update({"zrot_jerk": jerk_sampled_dict["zrot"]})
            qolo_command_dict.update({"avg_x_jerk": np.average(jerk_sampled_dict["x"])})
            qolo_command_dict.update(
                {"avg_zrot_jerk": np.average(jerk_sampled_dict["zrot"])}
            )

            np.save(command_sampled_filepath, qolo_command_dict)

    print("Finish extracting all twist & computing derivate of twist (qolo_command)!")
