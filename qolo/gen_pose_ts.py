# -*-coding:utf-8 -*-
'''
@File    :   gen_pose_ts.py
@Time    :   2021/10/26
@Author  :   Yujie He
@Version :   1.1
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

    pose_stamped_dict = {'timestamp': t_list, 
                        'position': p_list, 
                        'orientation': o_list}
    return pose_stamped_dict

#%% interpolate pose_stamped with scipy
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy import interpolate

def interp_pose(source_dict, target_dict):
    """Calculate interpolations for all states
    """
    source_ts = source_dict.get('timestamp')
    source_pos = source_dict.get('position')
    source_ori = source_dict.get('orientation')
    interp_ts = target_dict.get('timestamp')

    source_ts = np.asarray(source_ts, dtype=np.float64)
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

def interp_rotation(source_ts, interp_ts, source_ori):
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html

    slerp = Slerp(source_ts, R.from_quat(source_ori))
    interp_ori = slerp(interp_ts)
    return interp_ori.as_quat()

def interp_translation(source_ts, interp_ts, source_pos):
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html#scipy.interpolate.interp1d

    f = interpolate.interp1d(source_ts, np.transpose(source_pos))
    interp_pos = f(interp_ts)
    return np.transpose(interp_pos)


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    args = parser.parse_args()

    allf = AllFrames(args)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir))) 
    lidar_file_dir = allf.lidar_dir
    
    # destination: pose data in data/xxxx_processed/pose_stamped
    data_processed = args.folder + "_processed"
    data_processed_dir = os.path.join(args.base, args.data, data_processed)
    if not os.path.exists(allf.lidar_dir):
        print("ERROR: please use `gen_lidar_from_rosbags.py` to extract lidar files first!")
    if not os.path.exists(allf.pose_dir):
        os.makedirs(allf.pose_dir)

    print("Starting extracting pose_stamped files from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        all_stamped_filepath = os.path.join(allf.pose_dir, bag_name+'_all_pose_stamped.npy')
        lidar_stamped_filepath = os.path.join(allf.pose_dir, bag_name+'_lidar_pose_stamped.npy')

        if os.path.exists(lidar_stamped_filepath):
            continue
        else:
            if os.path.exists(all_stamped_filepath):
                pose_stamped_dict_np = np.load(all_stamped_filepath, allow_pickle=True)
                pose_stamped_dict = pose_stamped_dict_np.item()
            else:
                pose_stamped_dict = extract_pose_from_rosbag(bag_path)
                np.save(all_stamped_filepath, pose_stamped_dict)
        
            lidar_stamped = np.load(os.path.join(lidar_file_dir, bag_name+"_stamped.npy"), allow_pickle=True)
            lidar_pose_dict = interp_pose(pose_stamped_dict, lidar_stamped.item())
            np.save(lidar_stamped_filepath, lidar_pose_dict)
    print("Finish extracting all pose_stamped and interpolate lidar pose_stamped for viz")
