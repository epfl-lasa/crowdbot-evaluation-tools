# -*-coding:utf-8 -*-
'''
@File    :   gen_twist_ts.py
@Time    :   2021/11/05
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

# TODO: should specify which part of Twist data is saved instead of saving two many zeros

import os
import sys
import argparse
import numpy as np
import rosbag
from crowdbot_data import AllFrames, bag_file_filter

#%% extract twist_stamped from rosbag without rosbag play
def timestamp_str(ts):
    t = ts.secs + ts.nsecs / float(1e9)
    return '{:.12f}'.format(t)

def ts_to_sec(ts):
    return ts.secs + ts.nsecs / float(1e9)

def extract_twist_from_rosbag(bag_file_path, args):
    # load rosbag and BagTfTransformer
    # bag = rosbag.Bag(bag_file_path)
    twist_msg_sum = 0
    num_msgs_between_logs = 50
    x_list, zrot_list, t_list = [], [], []

    with rosbag.Bag(bag_file_path, 'r') as bag:
        # Look for the topics that are available and save the total number of messages for each topic (useful for the progress bar)
        total_num_twist_msgs = 0
        # TODO: need fixing total_num_twist_msgs
        total_num_twist_msgs = bag.get_message_count(topic_filters=args.twist_topic)
        print('Found twist topic: {} with {} messages'.format(args.twist_topic, total_num_twist_msgs))

        # Extract twist msg to file
        # !!!for topic, msg, t in bag.read_messages(topics=['/topics name'])
        for topic, msg, t in bag.read_messages():
            if topic == args.twist_topic:
                x_vel    = msg.twist.linear.x
                zrot_vel = msg.twist.angular.z
                ts_vel   = ts_to_sec(msg.header.stamp)
                x_list.append(x_vel)
                zrot_list.append(zrot_vel)
                t_list.append(ts_vel)

                # TODO: need fixing total_num_twist_msgs
                if twist_msg_sum % num_msgs_between_logs == 0 or twist_msg_sum >= total_num_twist_msgs - 1:
                    print('Event messages: {} / {}'.format(twist_msg_sum + 1, total_num_twist_msgs))
                twist_msg_sum += 1

    t_np = np.asarray(t_list, dtype=np.float64)
    x_np = np.asarray(x_list, dtype=np.float32)
    z_np = np.asarray(zrot_list, dtype=np.float32)

    twist_stamped_dict = {'timestamp': t_np, 
                          'x': x_np, 
                          'zrot': z_np}
    print('Current twist extracted!')
    return twist_stamped_dict


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    parser.add_argument('--twist_topic', default='/qolo/twist', type=str,
                        help='topic for qolo twist')
    args = parser.parse_args()

    allf = AllFrames(args)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir))) 
    
    # destination: twist data in data/xxxx_processed/twist
    if not os.path.exists(allf.twist_dir):
        os.makedirs(allf.twist_dir)

    print("Starting extracting twist_stamped from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        twist_stamped_filepath = os.path.join(allf.twist_dir, bag_name+'_twist_stamped.npy')

        if not os.path.exists(twist_stamped_filepath):
            twist_stamped_dict = extract_twist_from_rosbag(bag_path, args)
            np.save(twist_stamped_filepath, twist_stamped_dict)
        else:
            print("{} already existed!".format(twist_stamped_filepath))
            continue
    
    print("Finish extracting all twist_stamped msg!")

"""
ref: https://github.com/uzh-rpg/rpg_e2vid/blob/master/scripts/extract_events_from_rosbag.py
http://wiki.ros.org/rosbag/Code%20API#Python_API
"""
