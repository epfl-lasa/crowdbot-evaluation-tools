#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   bag_filter_image.py
@Date created  :   2022/01/31
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides ...
python bag_filter_image.py -f 1203_manual
"""
# =============================================================================
"""
TODO:
1. to extract the rgbd data maybe as a video (data files)
Export each video (with bouding boxes) as mp4 --> then perform a deface --> then save it again as a rosbag
    - bag to rgb and depth: https://github.com/nihalsoans91/Bag_to_Depth
    - deface link: https://github.com/ORB-HD/deface
2. to filter the camera topic from the rosbag so we can export rosbags without camera data
3. write back defaced rgb image back to rosbag
    - create rosbag back into rosbag
        1. https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_bagcreater
        2. https://github.com/uzh-rpg/rpg_e2vid/blob/master/scripts/image_folder_to_rosbag.py
"""
# =============================================================================


import os
import sys
from tabnanny import verbose
import yaml
import argparse

import numpy as np

import rosbag

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# sys.path.insert(0, '..')
curr_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(curr_dir_path, '../'))
from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter
from qolo.utils.process_util import ts_to_sec, ts_to_sec_str

#%% Utility function for extraction twist_stamped from rosbag and apply interpolation
def filter_image_from_rosbag(inbag_name, inbag_path, outbag_base_path, args):
    """Filter out twist_stamped from rosbag without rosbag play"""

    # twist_msg_sum = 0
    # num_msgs_between_logs = 100
    # x_list, zrot_list, t_list = [], [], []

    image_type = ['Image']
    other_type = []
    if 'Image' in args.filter_type:
        other_type = args.filter_type
        other_type.remove('Image')
        other_type.append('CameraInfo')
    image_topics = []
    other_topics = []
    # ['/camera_left/aligned_depth_to_color/camera_info', '/camera_left/aligned_depth_to_color/image_raw', '/camera_left/color/camera_info', '/camera_left/color/image_raw', '/camera_left/depth/camera_info', '/camera_left/depth/image_rect_raw', '/darknet_ros/detection_image', '/image_with_bounding_boxes']

    inbag = rosbag.Bag(inbag_path, "r")
    baginfo_dict = yaml.load(inbag._get_yaml_info(), Loader=yaml.FullLoader)

    for topic_msg_dict in baginfo_dict['topics']:
        if any(ft in topic_msg_dict['type'] for ft in image_type):
            image_topics.append(topic_msg_dict['topic'])
        if any(ft in topic_msg_dict['type'] for ft in other_type):
            other_topics.append(topic_msg_dict['topic'])

    if len(image_topics) + len(other_topics) > 0:
        print(
            "{} image topics to be filtered out:\n{}".format(
                len(image_topics), image_topics
            )
        )
        print(
            "{} other topics to be filtered out:\n{}".format(
                len(other_topics), other_topics
            )
        )

        outbag_path = os.path.abspath(
            os.path.join(outbag_base_path, '..', 'nocam_' + inbag_name + '.bag')
        )

        # TODO: switch back with overwrite
        if not os.path.exists(outbag_path) or args.overwrite:
            # if not os.path.exists(outbag_path):

            with rosbag.Bag(outbag_path, 'w') as outbag:
                bridge = CvBridge()
                for topic, msg, t in inbag.read_messages():
                    if topic in image_topics:
                        # create folder
                        image_dirpath = os.path.join(outbag_base_path, topic[1:])
                        if not os.path.exists(image_dirpath):
                            os.makedirs(image_dirpath)
                        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
                        # TODO: save image timestamp from original rosbag
                        # TODO: save other metadata & camera info
                        encoding = msg.encoding  # 'rgb8', 'bgr8', '16UC1'
                        # image_with_bounding_boxes/ topic doesn't assign real timestamp!
                        if topic == '/image_with_bounding_boxes':
                            timestamp = ts_to_sec_str(t)
                        else:
                            timestamp = ts_to_sec_str(msg.header.stamp)
                        height = msg.height
                        width = msg.width
                        image_savepath = os.path.join(
                            image_dirpath, str(timestamp) + ".png"
                        )

                        # TODO: add `or args.overwrite`
                        # if not os.path.exists(image_savepath):
                        if not os.path.exists(image_savepath) or args.overwrite:
                            if args.verbose:
                                print(
                                    "Saving {} (encoding: {})".format(
                                        topic[1:] + '/' + str(timestamp) + ".png",
                                        encoding,
                                    )
                                )

                            # add if condition for depth and rgb image separately
                            if encoding == 'rgb8':
                                cv_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                            else:
                                cv_img = bridge.imgmsg_to_cv2(msg, encoding)
                            # write image topics into separate folders for privacy processing
                            cv2.imwrite(image_savepath, cv_img)

                    elif topic in other_topics:
                        # print("Skip msg from", topic)
                        pass
                    else:
                        # write other topics into new rosbag
                        outbag.write(topic, msg, t)
            print("Images are filtered out!")
        else:
            print("Nocam rosbag has already saved in", outbag_path)
    else:
        print("No image topics are stored in current rosbag")


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="1203_manual",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--filter_topic", type=str, nargs='+', help="msg topic(s) to be filtered out"
    )
    # e.g., sensor_msgs/Image, sensor_msgs/CameraInfo
    parser.add_argument(
        "--filter_type",
        default=["Image"],
        type=str,
        nargs='+',
        help="msg type(s) to be filtered out",
    )
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=True)
    parser.add_argument(
        "--verbose",
        dest="verbose",
        action="store_true",
        help="Whether to print debug logs verbosely (default: false)",
    )
    parser.set_defaults(verbose=True)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: twist data in data/xxxx_processed/source_data/twist
    filtered_data_dirpath = os.path.join(rosbag_dir + "_filtered")
    if not os.path.exists(filtered_data_dirpath):
        os.makedirs(filtered_data_dirpath)

    print("Starting filtering images from {} rosbags!".format(len(bag_files)))
    print("Type to be filtered out: {}".format(args.filter_type))
    print("Topic to be filtered out: {}".format(args.filter_topic))

    counter = 0
    for bf in bag_files:
        source_bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), source_bag_path))

        filtered_bag_dirpath = os.path.join(filtered_data_dirpath, bag_name)

        if not os.path.exists(filtered_bag_dirpath) or (args.overwrite):
            if not os.path.exists(filtered_bag_dirpath):
                os.makedirs(filtered_bag_dirpath)
            filter_image_from_rosbag(
                bag_name, source_bag_path, filtered_bag_dirpath, args
            )
        else:
            print(
                "Detected generated data already existed in {}!".format(
                    filtered_bag_dirpath
                )
            )
            print("If you want to overwrite, use flag --overwrite")

    print("Finish filtering out all images from raw rosbag!")
