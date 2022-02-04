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
The module provides pipeline to filter out image and rgb data in pointcloud from
rosbags to generate nocam-version rosbags and images with different image topics
Example: python bag_filter_image.py -f 1203_manual
"""
# =============================================================================


import os
import sys
import yaml
import json
import fnmatch
import argparse

import cv2
import numpy as np

import rosbag
import ros_numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter
from qolo.utils.process_util import (
    ts_to_sec_str,
    delete_rgb_field,
    ts_to_sec,
    get_xyzrgb_points,
)


def save_camera_info(outbag_base_path, topic, msg):
    """save CameraInfo as json file"""

    info_base = topic[1:].rsplit('/', 1)[0]
    image_dirpath = os.path.join(outbag_base_path, info_base)
    if not os.path.exists(image_dirpath):
        os.makedirs(image_dirpath)

    camera_info_savepath = os.path.join(image_dirpath, "camera_info.json")
    # camera_info_savepath = os.path.join(image_dirpath, "camera_info.yaml")

    frame_id = msg.header.frame_id
    height = msg.height
    width = msg.width
    distortion_model = msg.distortion_model
    # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    distortion_coeff = list(msg.D)
    intrinsic_mat = list(msg.K)
    cam_info_dict = {
        'frame_id': frame_id,
        'height': height,
        'width': width,
        'K': intrinsic_mat,
        'distortion_model': distortion_model,
        'D': distortion_coeff,
    }

    # json
    with open(camera_info_savepath, 'w') as cam_info_file:
        json.dump(
            cam_info_dict,
            cam_info_file,
            indent=4,
            sort_keys=False,
        )

    # yaml
    # with open(camera_info_savepath, 'w', encoding='utf-8') as f:
    #     yaml.dump(cam_info_dict, default_flow_style=None)

    print("Save CameraInfo from {} topic!".format(topic))


#%% Utility function for filter images and pointcloud from rosbag and apply interpolation
def filter_image_from_rosbag(inbag_name, inbag_path, outbag_base_path, args):
    """Filter out images and pointcloud from rosbag without rosbag play"""

    image_type = ['Image']
    other_type = []

    if 'Image' in args.filter_type:
        other_type = args.filter_type
        other_type.remove('Image')
        # other_type.append('CameraInfo')

    image_topic_info = dict()
    other_topics_info = dict()

    inbag = rosbag.Bag(inbag_path, "r")
    baginfo_dict = yaml.load(inbag._get_yaml_info(), Loader=yaml.FullLoader)

    for topic_msg_dict in baginfo_dict['topics']:
        if any(ft in topic_msg_dict['type'] for ft in image_type):
            image_topic_info.update(
                {topic_msg_dict['topic']: topic_msg_dict['messages']}
            )
        if any(ft in topic_msg_dict['type'] for ft in other_type):
            other_topics_info.update(
                {topic_msg_dict['topic']: topic_msg_dict['messages']}
            )

    image_topics = list(image_topic_info.keys())
    other_topics = list(other_topics_info.keys())

    if len(image_topics) + len(other_topics) > 0:
        print(
            "{} image topic(s) to be filtered out:{}".format(
                len(image_topics), image_topics
            )
        )
        print(
            "{} other topic(s) to be filtered out:{}".format(
                len(other_topics), other_topics
            )
        )

        outbag_path = os.path.abspath(
            os.path.join(outbag_base_path, '..', 'nocam_' + inbag_name + '.bag')
        )

        # check existing by counting the number of images and msg
        if not args.overwrite:
            print("Checking files existing or not ...")
            exists_cnt = 0
            for topic, msg_num in image_topic_info.items():
                image_dirpath = os.path.join(outbag_base_path, topic[1:])
                if not os.path.exists(image_dirpath):
                    os.makedirs(image_dirpath)
                img_num = len(fnmatch.filter(os.listdir(image_dirpath), "*.png"))
                if msg_num == img_num:
                    exists_cnt += 1

            if exists_cnt == len(image_topics) and os.path.exists(outbag_path):
                all_files_exist = True
            else:
                all_files_exist = False

            print("All files exist?", all_files_exist)
        else:
            all_files_exist = False

        if not all_files_exist or args.overwrite:

            with rosbag.Bag(outbag_path, 'w') as outbag:
                bridge = CvBridge()
                topic_save_counter = dict()
                for topic, msg, t in inbag.read_messages():
                    # dicard msg from image topics
                    if topic in image_topics:
                        # Example: ['/camera_left/aligned_depth_to_color/image_raw', '/camera_left/color/image_raw', '/camera_left/depth/image_rect_raw', '/darknet_ros/detection_image', '/image_with_bounding_boxes']

                        # create folder to save the images
                        image_dirpath = os.path.join(outbag_base_path, topic[1:])
                        if not os.path.exists(image_dirpath):
                            os.makedirs(image_dirpath)
                        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
                        encoding = msg.encoding  # 'rgb8', 'bgr8', '16UC1'
                        # image_with_bounding_boxes/ topic doesn't assign real timestamp!
                        if topic == '/image_with_bounding_boxes':
                            timestamp = ts_to_sec_str(t)
                        else:
                            timestamp = ts_to_sec_str(msg.header.stamp)
                        # save image timestamp from original rosbag as name of exported images
                        image_savepath = os.path.join(
                            image_dirpath, str(timestamp) + ".png"
                        )

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

                    # dicard msg from other topics (e.g., CameraInfo)
                    elif topic in other_topics:
                        # Example: ['/camera_left/aligned_depth_to_color/camera_info', '/camera_left/color/camera_info', '/camera_left/depth/camera_info']
                        pass

                    # dicard rgb data in points generated by realsense camera
                    elif topic == '/camera_left/depth/color/points':

                        pc_arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                        new_pc_arr = delete_rgb_field(pc_arr)
                        filtered_msg = ros_numpy.point_cloud2.array_to_pointcloud2(
                            new_pc_arr,
                            stamp=msg.header.stamp,
                            frame_id=msg.header.frame_id,
                        )
                        outbag.write(topic, filtered_msg, t)

                    # maintain remaining msg from exclusive topics
                    else:
                        # save camera_info
                        # /camera_left/color/camera_info <->
                        # /camera_left/color/image_raw
                        if 'camera_info' in topic:
                            if topic in topic_save_counter.keys():
                                topic_save_counter[topic] += 1
                            else:
                                topic_save_counter.update({topic: 1})
                            if topic_save_counter[topic] == 1:
                                save_camera_info(outbag_base_path, topic, msg)

                        outbag.write(topic, msg, t)

            print("Images are filtered out!")
        else:
            # print("Nocam rosbag has already saved in", outbag_path)
            print(
                "Detected generated data already existed in {}!".format(
                    filtered_bag_dirpath
                )
            )
            print("If you want to overwrite, use flag --overwrite")
    else:
        print("No image topics are stored in current rosbag")


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="create rosbag without image data")

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
    parser.set_defaults(overwrite=False)
    parser.add_argument(
        "--verbose",
        dest="verbose",
        action="store_true",
        help="Whether to print debug logs verbosely (default: false)",
    )
    parser.set_defaults(verbose=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    # source: rosbag data
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))

    # destination: new rosbag data and exported images and camera parameters
    filtered_data_dirpath = os.path.join(rosbag_dir + "_filtered")
    if not os.path.exists(filtered_data_dirpath):
        os.makedirs(filtered_data_dirpath)

    print("# Starting filtering images from {} rosbags!".format(len(bag_files)))
    print("# Type(s) to be filtered out: {}".format(args.filter_type))
    print("# Topic(s) to be filtered out: {}".format(args.filter_topic))

    for counter, bf in enumerate(bag_files):
        bag_name = bf.split(".")[0]
        source_bag_path = os.path.join(rosbag_dir, bf)
        print("({}/{}): {}".format(counter + 1, len(bag_files), source_bag_path))

        filtered_bag_dirpath = os.path.join(filtered_data_dirpath, bag_name)

        if not os.path.exists(filtered_bag_dirpath):
            os.makedirs(filtered_bag_dirpath)
        filter_image_from_rosbag(bag_name, source_bag_path, filtered_bag_dirpath, args)

    print("Finish filtering out all images from raw rosbag!")
