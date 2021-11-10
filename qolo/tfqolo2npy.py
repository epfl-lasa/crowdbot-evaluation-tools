# -*-coding:utf-8 -*-
'''
@File    :   tfqolo2npy.py
@Time    :   2021/10/26
@Author  :   Yujie He
@Version :   1.2
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

import os
import sys
import argparse
import numpy as np

import rosbag

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "external"))
from tf_bag import BagTfTransformer

from crowdbot_data import AllFrames, bag_file_filter
from interp_util import interp_rotation, interp_translation

#%% extract pose_stamped from rosbag without rosbag play
def extract_pose_from_rosbag(bag_file_path):
    # load rosbag and BagTfTransformer
    bag = rosbag.Bag(bag_file_path)
    bag_transformer = BagTfTransformer(bag)
    #print(bag_transformer.getTransformGraphInfo())

    # "t265_pose_frame" -> "tf_qolo"
    # "t265_odom_frame" -> "t265_pose_frame"
    # "t265_odom_frame" -> "tf_qolo_world"
    trans_iter = bag_transformer.lookupTransformWhenTransformUpdates("tf_qolo_world", "tf_qolo", 
                                            trigger_orig_frame="t265_odom_frame", 
                                            trigger_dest_frame="t265_pose_frame")
    t_list, p_list, o_list = [], [], []
    for timestamp, transformation in trans_iter:
        (position, orientation) = transformation
        # timestamp in genpy.Time type
        t_list.append(timestamp.to_sec())
        p_list.append(position)
        o_list.append(orientation)

    t_np = np.asarray(t_list, dtype=np.float64)
    p_np = np.asarray(p_list, dtype=np.float64)
    o_np = np.asarray(o_list, dtype=np.float64)

    pose_stamped_dict = {'timestamp': t_np, 
                        'position': p_np, 
                        'orientation': o_np}
    return pose_stamped_dict

# interpolate pose_stamped with scipy
def interp_pose(source_dict, target_dict):
    """Calculate interpolations for all states
    """
    source_ts = source_dict.get('timestamp')
    source_pos = source_dict.get('position')
    source_ori = source_dict.get('orientation')
    interp_ts = target_dict.get('timestamp')
    interp_ts = np.asarray(interp_ts, dtype=np.float64)
    # print(min(interp_ts), max(interp_ts))
    # print(min(source_ts), max(source_ts))
    # don't resize, just discard the timestamps smaller than source
    if min(interp_ts) < min(source_ts):
        interp_ts[interp_ts<min(source_ts)] = min(source_ts)
    elif max(interp_ts) > max(source_ts):
        interp_ts[interp_ts>max(source_ts)] = max(source_ts)
    
    interp_dict = {}
    interp_dict['timestamp'] = interp_ts
    interp_dict['orientation'] = interp_rotation(source_ts, interp_ts, source_ori)
    interp_dict['position'] = interp_translation(source_ts, interp_ts, source_pos)
    return interp_dict

#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    parser.add_argument('--overwrite', dest='overwrite', action='store_true',
                        help="Whether to overwrite existing rosbags (default: false)")
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    allf = AllFrames(args)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))
    if not os.path.exists(allf.lidar_dir):
        print("ERROR: please use `gen_lidar_from_rosbags.py` to extract lidar files first!")
    
    # destination: pose data in data/xxxx_processed/source_data/qolo_tf
    qolo_tf_dir = os.path.join(allf.source_data_dir, 'qolo_tf')
    if not os.path.exists(qolo_tf_dir):
        os.makedirs(qolo_tf_dir)

    print("Starting extracting pose_stamped files from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        all_stamped_filepath = os.path.join(qolo_tf_dir, bag_name+'_tfqolo.npy')
        # sample with lidar frame
        lidar_stamped_filepath = os.path.join(qolo_tf_dir, bag_name+'_tfqolo_sampled.npy')

        if not os.path.exists(lidar_stamped_filepath):
            if (not os.path.exists(all_stamped_filepath)) or (args.overwrite):
                pose_stamped_dict = extract_pose_from_rosbag(bag_path)
                np.save(all_stamped_filepath, pose_stamped_dict)
            else:
                print("Detecting the generated {} already existed!".format(all_stamped_filepath))
                print('If you want to overwrite, use flag --overwrite')
                pose_stamped_dict = np.load(all_stamped_filepath, allow_pickle=True).item()

            lidar_stamped = np.load(os.path.join(allf.lidar_dir, bag_name+"_stamped.npy"), allow_pickle=True)
            lidar_pose_dict = interp_pose(pose_stamped_dict, lidar_stamped.item())
            np.save(lidar_stamped_filepath, lidar_pose_dict)
    print("Finish extracting all pose_stamped and interpolate lidar pose_stamped for viz")
