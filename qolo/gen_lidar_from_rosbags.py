#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   gen_lidar_from_rosbags.py
@Date created  :   2021/10/19
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow to extract pointcloud and corresponding timestamps
from rosbag.
"""
# =============================================================================
"""
TODO:
1. test support with pcd
"""
# =============================================================================


import os
import fnmatch
import argparse
import numpy as np

import rospy
import rosbag
import tf2_py as tf2
import ros_numpy

from qolo.core.crowdbot_data import bag_file_filter, CrowdBotData

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


def save_lidar(filename, pc, library="numpy", write_ascii=False, compressed=True):
    # save with numpy
    if library == "numpy":
        with open(filename + ".nby", "wb") as f:
            np.save(f, pc)

    # save with open3d
    elif library == "open3d":
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        # open3d.io.write_point_cloud(filename, pointcloud, write_ascii=False, compressed=False, print_progress=False)
        o3d.io.write_point_cloud(
            filename + ".pcd", pcd, write_ascii=write_ascii, compressed=compressed
        )


def extract_lidar_from_rosbag(bag_path, out_dir, args):
    """Extract and save combined laser scan from rosbag. Existing files will be overwritten"""

    # print("rosbag: {}".format(bag_path))

    # load lidar in a unified coordinate frame
    with rosbag.Bag(bag_path) as bag:
        tf_buffer = get_tf_tree(bag)
        front_msgs, front_ts = load_lidar(
            bag,
            args.front_topic,
            tf_buffer,  # args.front_topic = "/front_lidar/velodyne_points"
        )
        rear_msgs, rear_ts = load_lidar(
            bag, args.rear_topic, tf_buffer
        )  # args.rear_topic = "/rear_lidar/velodyne_points"

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

    def get_sync_inds(ts, sync_ts):
        d = np.abs(sync_ts.reshape(-1, 1) - ts.reshape(1, -1))
        return np.argmin(d, axis=1)

    front_inds = get_sync_inds(front_ts, sync_ts)
    rear_inds = get_sync_inds(rear_ts, sync_ts)

    # write frame to file
    for frame_id, (idx_front, idx_rear) in enumerate(zip(front_inds, rear_inds)):
        pc = np.concatenate((front_msgs[idx_front], rear_msgs[idx_rear]), axis=0)
        file_path = os.path.join(out_dir, "{0:05d}".format(frame_id))
        if args.compressed:
            save_lidar(file_path, pc, library="open3d")
        else:
            save_lidar(file_path, pc)
        """
        # save with numpy
        with open(file_path+".nby", "wb") as f:
            np.save(f, pc)

        # save with open3d
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        o3d.io.write_point_cloud(file_path+".pcd", pcd, write_ascii=False, compressed=True)
        """

    # yujie: save sync frame id and timestamp
    id_list, ts_list = [], []
    for frame_id, ts in enumerate(sync_ts):
        id_list.append(frame_id)
        ts_list.append(ts + offset)  # adding offset

    lidar_stamped_dict = {
        "timestamp": np.asarray(ts_list, dtype=np.float64),
        "id": np.array(frame_id),
    }
    # bag_name = out_dir.split("/")[-1]
    # stamp_file_path = os.path.join(out_dir, "..", bag_name + "_stamped.npy")
    # np.save(stamp_file_path, lidar_stamped_dict)

    s = (
        "Summary\n"
        "topic: synced frames\n"
        "count: {}\n"
        "average time between frames: {}\n"
    ).format(
        len(sync_ts),
        sync_dt,
    )
    print(s)

    return lidar_stamped_dict


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
        "--front_topic",
        default="/front_lidar/velodyne_points",
        type=str,
        help="topic for the front lidar",
    )
    parser.add_argument(
        "--rear_topic",
        default="/rear_lidar/velodyne_points",
        type=str,
        help="topic for the rear lidar",
    )
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Overwrite existing rosbags (default: False)",
    )
    parser.set_defaults(overwrite=False)
    parser.add_argument(
        "--compressed",
        dest="compressed",
        action="store_true",
        help="Save pointcloud with compressed format (default: True)",
    )
    parser.set_defaults(compressed=False)
    args = parser.parse_args()

    qolo_dataset = CrowdBotData()

    base_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

    # source: rosbag data in data/rosbag/xxxx
    # rosbag_dir = os.path.join(base_folder, "data", "rosbag", args.folder)
    rosbag_dir = os.path.join(qolo_dataset.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: lidar data in xxxx_processed/lidars
    data_processed = args.folder + "_processed"
    data_processed_dir = os.path.join(qolo_dataset.outbase_dir, data_processed)
    if not os.path.exists(data_processed_dir):
        os.makedirs(data_processed_dir)
    lidar_file_dir = os.path.join(data_processed_dir, "lidars")
    lidar_stamp_dir = os.path.join(data_processed_dir, "source_data", "timestamp")
    if not os.path.exists(lidar_stamp_dir):
        os.makedirs(lidar_stamp_dir)

    print("Starting extracting lidar files from {} rosbags!".format(len(bag_files)))

    for idx, bf in enumerate(bag_files):
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        out_dir = os.path.join(lidar_file_dir, bag_name)
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        out_lidar_np = fnmatch.filter(os.listdir(out_dir), "*.nby")

        print("({}/{}): {}".format(idx + 1, len(bag_files), bag_path))

        if args.overwrite:
            lidar_stamped_dict = extract_lidar_from_rosbag(bag_path, out_dir, args)
            print(
                "lidar_stamped_dict with {} frames".format(
                    len(lidar_stamped_dict['timestamp'])
                )
            )
            stamp_file_path = os.path.join(lidar_stamp_dir, bag_name + "_stamped.npy")
            np.save(stamp_file_path, lidar_stamped_dict)
        else:
            print("{} already extracted!!!".format(bf))
            continue
