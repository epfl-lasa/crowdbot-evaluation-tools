#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   commands2npy.py
@Date created  :   2021/11/27
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow to extract commands from rosbag.
"""
# =============================================================================


import os
import sys
import argparse

import numpy as np

import rosbag

from twist2npy import interp_twist
from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter
from qolo.utils.process_util import interp_translation, ts_to_sec

#%% Utility function for extraction `ToGui` from rosbag and apply interpolation
def extract_cmd_from_rosbag(bag_file_path, args):
    """extract commands from `rds_network_ros/ToGui` in rosbag without rosbag play"""
    command_msg_sum = 0
    num_msgs_between_logs = 100
    ts_list, commands_list = [], []

    with rosbag.Bag(bag_file_path, "r") as bag:
        # Look for the topics that are available and save the total number of messages for each topic (useful for the progress bar)
        total_num_command_msgs = bag.get_message_count(topic_filters=args.to_gui_topic)
        print(
            "Found to_gui topic: {} with {} messages".format(
                args.to_gui_topic, total_num_command_msgs
            )
        )

        for topic, msg, t in bag.read_messages():
            if topic == args.to_gui_topic:
                # these messages do not have time stamps
                timestamp = ts_to_sec(t)  # or t.to_sec()
                command = [
                    msg.nominal_command.linear,
                    msg.nominal_command.angular,
                    msg.corrected_command.linear,
                    msg.corrected_command.angular,
                ]

                ts_list.append(timestamp)
                commands_list.append(command)

                if (
                    command_msg_sum % num_msgs_between_logs == 0
                    or command_msg_sum >= total_num_command_msgs - 1
                ):
                    print(
                        "/rds_to_gui messages: {} / {}".format(
                            command_msg_sum + 1, total_num_command_msgs
                        )
                    )

                command_msg_sum += 1

    ts_np = np.array(ts_list)
    commands = np.array(commands_list)
    nominal_linear = commands[:, 0]
    nominal_angular = commands[:, 1]
    corrected_linear = commands[:, 2]
    corrected_angular = commands[:, 3]

    # cmd_raw_dict = {"timestamp": t_np, "x": x_np, "zrot": z_np}
    cmd_raw_dict = {
        "timestamp": ts_np,
        'nominal_linear': nominal_linear,
        'nominal_angular': nominal_angular,
        'corrected_linear': corrected_linear,
        'corrected_angular': corrected_angular,
    }
    print("Current commands extracted!")
    return cmd_raw_dict


def interp_linear_dict(linear_state_dict, target_dict, subset=None):
    """Calculate interpolations for all states"""

    source_ts = linear_state_dict.get("timestamp")
    interp_ts = target_dict.get("timestamp")
    interp_dict = {"timestamp": interp_ts}

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

    if subset == None:
        subset = linear_state_dict.keys()
    for val in subset:
        if val == "timestamp":
            pass
        else:
            interp = interp_translation(source_ts, interp_ts, linear_state_dict[val])
            interp_dict.update({val: interp})

    return interp_dict


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="1203_test",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--to_gui_topic",
        default="/rds_to_gui",
        type=str,
        help="topic for qolo rds_to_gui",
    )
    parser.add_argument(
        "--twist_topic", default="/qolo/twist", type=str, help="topic for qolo twist"
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

    # destination: twist data in data/xxxx_processed/source_data/commands
    cmd_dir = os.path.join(cb_data.source_data_dir, "commands")
    if not os.path.exists(cmd_dir):
        os.makedirs(cmd_dir)

    print("Starting extracting command msg from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        cmd_raw_filepath = os.path.join(cmd_dir, bag_name + "_commands_raw.npy")
        # sample with lidar frame
        cmd_sampled_filepath = os.path.join(cmd_dir, bag_name + "_commands_sampled.npy")

        if not os.path.exists(cmd_sampled_filepath) or (args.overwrite):
            if (not os.path.exists(cmd_raw_filepath)) or (args.overwrite):
                cmd_raw_dict = extract_cmd_from_rosbag(bag_path, args)
                np.save(cmd_raw_filepath, cmd_raw_dict)
            else:
                print(
                    "Detecting the generated {} already existed!".format(
                        cmd_raw_filepath
                    )
                )
                print("If you want to overwrite, use flag --overwrite")
                cmd_raw_dict = np.load(cmd_raw_filepath, allow_pickle=True).item()

            # sample with lidar frame
            lidar_stamp_dir = os.path.join(cb_data.source_data_dir, "timestamp")
            stamp_file_path = os.path.join(lidar_stamp_dir, bag_name + "_stamped.npy")
            lidar_stamped = np.load(
                stamp_file_path,
                allow_pickle=True,
            ).item()
            cmd_sampled_dict = interp_linear_dict(cmd_raw_dict, lidar_stamped)
            np.save(cmd_sampled_filepath, cmd_sampled_dict)

            source_len = len(cmd_raw_dict["timestamp"])
            target_len = len(lidar_stamped["timestamp"])
            print(
                "# Sample from {} frames to {} frames.".format(source_len, target_len)
            )
        else:
            print(
                "Detecting the generated {} already existed!".format(
                    cmd_sampled_filepath
                )
            )
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")

    print("Finish extracting all commands!")
