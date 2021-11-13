# -*-coding:utf-8 -*-
"""
@File    :   twist2npy.py
@Time    :   2021/11/05
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

# TODO: should specify which part of Twist data is saved instead of saving two many zeros
# TODO: consider transforming velocity to global frame (ref: bagToNpy.py)!

import os
import sys
import argparse

import numpy as np

import rosbag

from crowdbot_data import AllFrames, bag_file_filter
from process_util import interp_translation, compute_motion_derivative, ts_to_sec


#%% Utility function for extraction twist_stamped from rosbag and apply interpolation
def extract_twist_from_rosbag(bag_file_path, args):
    """Extract twist_stamped from rosbag without rosbag play"""

    twist_msg_sum = 0
    num_msgs_between_logs = 100
    x_list, zrot_list, t_list = [], [], []

    with rosbag.Bag(bag_file_path, "r") as bag:
        # Look for the topics that are available and save the total number of messages for each topic
        total_num_twist_msgs = bag.get_message_count(topic_filters=args.twist_topic)
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
    interp_ts = np.asarray(interp_ts, dtype=np.float64)
    # don't resize, just discard the timestamps smaller than source
    if min(interp_ts) < min(source_ts):
        interp_ts[interp_ts < min(source_ts)] = min(source_ts)
    elif max(interp_ts) > max(source_ts):
        interp_ts[interp_ts > max(source_ts)] = max(source_ts)

    interp_dict = {}
    interp_dict["timestamp"] = interp_ts
    interp_dict["x"] = interp_translation(source_ts, interp_ts, source_x)
    interp_dict["zrot"] = interp_translation(source_ts, interp_ts, source_zrot)
    return interp_dict


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

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

    allf = AllFrames(args)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: twist data in data/xxxx_processed/source_data/twist
    twist_dir = os.path.join(allf.source_data_dir, "twist")
    if not os.path.exists(twist_dir):
        os.makedirs(twist_dir)

    print("Starting extracting twist_stamped from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        twist_stamped_filepath = os.path.join(twist_dir, bag_name + "_twist.npy")
        # sample with lidar frame
        twist_sampled_filepath = os.path.join(
            twist_dir, bag_name + "_twist_sampled.npy"
        )

        if not os.path.exists(twist_sampled_filepath) or (args.overwrite):
            if (not os.path.exists(twist_stamped_filepath)) or (args.overwrite):
                twist_stamped_dict = extract_twist_from_rosbag(bag_path, args)
                np.save(twist_stamped_filepath, twist_stamped_dict)
            else:
                print(
                    "Detecting the generated {} already existed!".format(
                        twist_stamped_filepath
                    )
                )
                print("If you want to overwrite, use flag --overwrite")
                twist_stamped_dict = np.load(
                    twist_stamped_filepath, allow_pickle=True
                ).item()

            lidar_stamped = np.load(
                os.path.join(allf.lidar_dir, bag_name + "_stamped.npy"),
                allow_pickle=True,
            ).item()
            twist_sampled_dict = interp_twist(twist_stamped_dict, lidar_stamped)
            np.save(twist_sampled_filepath, twist_sampled_dict)

            source_len = len(twist_stamped_dict["timestamp"])
            target_len = len(lidar_stamped["timestamp"])
            print(
                "# Sample from {} frames to {} frames.".format(source_len, target_len)
            )

        if args.gen_acc:
            acc_dir = os.path.join(allf.source_data_dir, "acc")
            if not os.path.exists(acc_dir):
                os.makedirs(acc_dir)

            # sample with lidar frame
            acc_sampled_filepath = os.path.join(acc_dir, bag_name + "_acc_sampled.npy")

            if (not os.path.exists(acc_sampled_filepath)) or (args.overwrite):
                if not ("lidar_stamped" in locals().keys()):
                    lidar_stamped = np.load(
                        os.path.join(allf.lidar_dir, bag_name + "_stamped.npy"),
                        allow_pickle=True,
                    ).item()
                if not ("twist_sampled_dict" in locals().keys()):
                    twist_sampled_dict = np.load(
                        twist_sampled_filepath, allow_pickle=True
                    ).item()

                # no need to generate acc_stamped_dict
                # acc_stamped_filepath = os.path.join(acc_dir, bag_name+'_acc.npy')
                # acc_stamped_dict = compute_motion_derivative(twist_stamped_dict)
                # np.save(acc_stamped_filepath, acc_stamped_dict)
                # print('acc_stamped_dict', len(acc_stamped_dict['x']))
                # acc_sampled_dict = interp_twist(acc_stamped_dict, lidar_stamped)

                acc_sampled_dict = compute_motion_derivative(twist_sampled_dict)
                np.save(acc_sampled_filepath, acc_sampled_dict)

    finish_output = "Finish extracting all twist!"
    if args.gen_acc:
        finish_output = (
            finish_output.replace("!", " ") + "& computing acc (derivative of twist)!"
        )
    print(finish_output)

"""
ref: https://github.com/uzh-rpg/rpg_e2vid/blob/master/scripts/extract_events_from_rosbag.py
http://wiki.ros.org/rosbag/Code%20API#Python_API
"""
