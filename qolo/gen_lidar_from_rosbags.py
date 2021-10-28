# -*-coding:utf-8 -*-
'''
@File    :   gen_lidar_from_rosbags.py
@Time    :   2021/10/19
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''


import os
import numpy as np

import rospy
import rosbag

import tf2_py as tf2
import ros_numpy

import argparse

# sudo apt-get install ros-$ROS_DISTRO-tf2-sensor-msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def get_tf_tree(bag):
    tf_buffer = tf2.BufferCore(rospy.Duration(1e9))
    for topic, msg, _ in bag.read_messages(topics=["/tf", "/tf_static"]):
        for msg_tf in msg.transforms:
            if topic == "/tf_static":
                tf_buffer.set_transform_static(msg_tf, "default_authority")
            else:
                tf_buffer.set_transform(msg_tf, "default_authority")

    return tf_buffer


def load_lidar(bag, topic, tf_buffer, target_frame="tf_qolo"):
    msgs, ts = [], []
    src_frame = None
    failed_counter = 0
    for _, msg, t in bag.read_messages(topics=[topic]):
        src_frame = msg.header.frame_id

        try:
            trans = tf_buffer.lookup_transform_core(
                target_frame, src_frame, msg.header.stamp
            )
            msg = do_transform_cloud(msg, trans)
        except tf2.ExtrapolationException as e:  # noqa
            # print(e)
            failed_counter += 1
            continue

        pc_xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        msgs.append(pc_xyz)
        ts.append(msg.header.stamp.to_sec())
        # ts.append(t.to_sec())

    ts = np.array(ts, dtype=np.float64)  # warning: float32 is not enough

    s = (
        "Summary\n"
        "topic: {}\n"
        "count: {}\n"
        "min timestamp: {}\n"
        "max timestamp: {}\n"
        "average time between frames: {}\n"
        "source frame: {}\n"
        "target frame: {}\n"
        "tf failed count: {}\n"
    ).format(
        topic,
        len(ts),
        ts[0],
        ts[-1],
        (ts[-1] - ts[0]) / len(ts),
        src_frame,
        target_frame,
        failed_counter,
    )

    print(s)

    return msgs, ts


def extract_lidar_from_rosbag(bag_path, out_dir, args):
    """Extract and save combined laser scan from rosbag. Existing files will be overwritten.
    """
    #print("rosbag: {}".format(bag_path))

    # load lidar in a unified coordinate frame
    with rosbag.Bag(bag_path) as bag:
        tf_buffer = get_tf_tree(bag)
        front_msgs, front_ts = load_lidar(
            bag, args.front_topic, tf_buffer # args.front_topic = "/front_lidar/velodyne_points"
        )
        rear_msgs, rear_ts = load_lidar(bag, args.rear_topic, tf_buffer) # args.rear_topic = "/rear_lidar/velodyne_points"

    # sync lidar
    offset = min(front_ts.min(), rear_ts.min())
    front_ts -= offset  # only for easy viewing
    rear_ts -= offset

    front_t0, front_t1 = front_ts.min(), front_ts.max()
    front_dt = (front_t1 - front_t0) / float(len(front_ts))

    rear_t0, rear_t1 = rear_ts.min(), rear_ts.max()
    rear_dt = (rear_t1 - rear_t0) / float(len(rear_ts))

    sync_dt = max(front_dt, rear_dt)  # sync to the slower one
    sync_t0 = max(front_t0, rear_t0)
    sync_t1 = min(front_t1, rear_t1)
    sync_ts = np.arange(start=sync_t0, step=sync_dt, stop=sync_t1, dtype=np.float64)

    # print(sync_ts[:30])
    # print(front_ts[:30])
    # print(rear_ts[:30])

    def get_sync_inds(ts, sync_ts):
        d = np.abs(sync_ts.reshape(-1, 1) - ts.reshape(1, -1))
        return np.argmin(d, axis=1)

    front_inds = get_sync_inds(front_ts, sync_ts)
    rear_inds = get_sync_inds(rear_ts, sync_ts)

    # print(front_inds[:30])
    # print(rear_inds[:30])

    # write frame to file
    for frame_id, (idx_front, idx_rear) in enumerate(zip(front_inds, rear_inds)):
        pc = np.concatenate((front_msgs[idx_front], rear_msgs[idx_rear]), axis=0)
        file_path = os.path.join(out_dir, "{0:05d}.nby".format(frame_id))
        with open(file_path, "wb") as f:
            np.save(f, pc)

    # yujie: save sync frame id and timestamp
    id_list, ts_list = [], []
    for frame_id, ts in enumerate(sync_ts):
        id_list.append(frame_id)
        ts_list.append(ts+offset) # adding offset
    lidar_stamped_dict = {'timestamp': ts_list, 'id': frame_id}
    bagname = out_dir.split('/')[-1]
    stamp_file_path = os.path.join(out_dir, '..', bagname+'_stamped.npy')
    np.save(stamp_file_path, lidar_stamped_dict)

    s = (
        "Summary\n"
        "topic: synced frames\n"
        "count: {}\n"
        "average time between frames: {}\n"
    ).format(len(sync_ts), sync_dt,)
    print(s)

# yujie: filter the files with specific extensions
def bag_file_filter(f):
    if f[-4:] in ['.bag']:
        return True
    else:
        return False


if __name__ == "__main__":
    # base_dir = "/globalwork/datasets/crowdbot/qolo_market_data"
    # bag_files = os.listdir(os.path.join(base_dir, "rosbag"))

    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    parser.add_argument('--front_topic', default='/front_lidar/velodyne_points', type=str,
                        help='topic for the front lidar')
    parser.add_argument('--rear_topic', default='/rear_lidar/velodyne_points', type=str,
                        help='topic for the rear lidar')                   
    args = parser.parse_args()

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir))) 

    # destination: lidar data in data/xxxx_processed/lidars
    data_processed = args.folder + "_processed"
    data_processed_dir = os.path.join(args.base, args.data, data_processed)
    if not os.path.exists(data_processed_dir):
        os.makedirs(data_processed_dir)
    lidar_file_dir = os.path.join(data_processed_dir, "lidars")

    print("Starting extracting lidar files from {} rosbags!".format(len(bag_files)))

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        out_dir = os.path.join(lidar_file_dir, bf.split(".")[0])

        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
            extract_lidar_from_rosbag(bag_path, out_dir, args)
        else:
            print("{} already extracted!!!".format(bf))
            continue