#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   bag_append_image.py
@Date created  :   2022/02/03
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides pipeline to append new image topic(s) into a existing rosbag from image folder(s)
ref:
    - https://github.com/uzh-rpg/rpg_e2vid/blob/master/scripts/image_folder_to_rosbag.py
    - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
python bag_append_image.py -f 0424_shared_control
"""
# =============================================================================

import os, sys
import argparse
import fnmatch

import cv2
import numpy as np

import rospy, rosbag, ros_numpy
from cv_bridge import CvBridge, CvBridgeError

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter
from qolo.utils.process_util import (
    sec_str_to_ts,
    ts_to_sec_str,
    delete_rgb_field,
    ts_to_sec,
    get_xyzrgb_points,
)

# main recorded rgb image topics
image_topics = [
    '/camera_left/color/image_raw',
    '/darknet_ros/detection_image',
    '/image_with_bounding_boxes',
]

# std_msgs/Header header
# uint32 height
# uint32 width
# string encoding
# uint8 is_bigendian
# uint32 step
# uint8[] data


def create_deface_rosbag(
    inbag_dir, inbag_filename, deface_data_dir, outbag_dir, image_topic_info, args
):
    """Replace rgb images in <image_topics> of rosbags with its defaced counterpart without rosbag play"""

    ## input
    bag_name = inbag_filename.split(".")[0]
    # xxx_filtered/bag_name/
    deface_img_dir = os.path.join(deface_data_dir, bag_name)

    outbag_path = os.path.abspath(
        os.path.join(outbag_dir, 'defaced_' + inbag_filename)
    )

    # check existing or not
    defaced_bag_exists = os.path.exists(outbag_path)

    print("Defaced rosbag exist?", defaced_bag_exists)
    print("Overwrite?", args.overwrite)

    if not defaced_bag_exists or args.overwrite:
        inbag_path = os.path.join(inbag_dir, inbag_filename)
        inbag = rosbag.Bag(inbag_path, "r")

        bridge = CvBridge()
        with rosbag.Bag(outbag_path, 'w') as outbag:

            for topic, msg, t in inbag.read_messages():
                # dicard msg from image topics
                if not topic in image_topic_info.keys():
                    # maintain all other ros msg
                    outbag.write(topic, msg, t)

                else:
                    # defaced image path
                    defaced_img_dirpath = os.path.join(deface_img_dir, topic[1:]+'_deface')
                    if not os.path.exists(defaced_img_dirpath):
                        sys.exit('Please defaced images first')

                    encoding = msg.encoding  # 'rgb8', 'bgr8', '16UC1'
                    # image_with_bounding_boxes/ topic doesn't assign real timestamp!
                    if topic == '/image_with_bounding_boxes':
                        # timestamp => str
                        stamp_sec = ts_to_sec_str(t)
                        # timestamp => rospy.rostime.Time
                        stamp_ros = t
                    else:
                        stamp_sec = ts_to_sec_str(msg.header.stamp)
                        stamp_ros = rospy.Time(float(stamp_sec))

                    # cv2.imread(os.path.join(defaced_img_dirpath, stamp_sec+'.png'), 0)

                    deface_img_path = os.path.join(defaced_img_dirpath, stamp_sec+'.png')
                    if os.path.exists(deface_img_path):
                        defaced_img = cv2.imread(deface_img_path)
                    else:
                        sys.exit('Please check whether {} exists !!!'.format(deface_img_path))

                    try:
                        img_msg = bridge.cv2_to_imgmsg(defaced_img, encoding=encoding)
                        # print(defaced_img.shape, stamp_ros)
                        img_msg.header.stamp = stamp_ros
                        img_msg.header.seq = msg.header.seq
                        outbag.write(topic, img_msg, img_msg.header.stamp)

                    except CvBridgeError as e:
                        print(e)

            outbag.close()
        print('Start reindexing ...')
        os.system('rosbag reindex {}'.format(outbag_path))
        orig_outbag_path = os.path.abspath(
            os.path.join(outbag_dir, 'defaced_{}.orig.bag'.format(bag_name))
        )
        print('Delete original bag ...')
        os.system('rm -rf {}'.format(orig_outbag_path))
        print("Defaced rosbag are created!")
    else:
        # print("Defaced rosbag has already saved in", outbag_path)
        print("Detected generated data already existed in {}!".format(outbag_path))
        print("If you want to overwrite, use flag --overwrite")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f",
        "--folder",
        default="0424_shared_control",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--outdir",
        default="/hdd/data_qolo/defaced_bags",
        type=str,
        help="different subfolder in rosbag/ dir",
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

    print('Folder to process: {}'.format(args.folder))
    cb_data = CrowdBotDatabase(args.folder)

    # source
    inbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    deface_data_dir = os.path.join(inbag_dir + "_filtered")
    if not os.path.exists(deface_data_dir):
        sys.exit("Please use bag_filter_image.py to extract image data first!")
    bag_files = list(filter(bag_file_filter, os.listdir(inbag_dir)))

    # destination: new rosbag data and exported images and camera parameters
    if args.outdir is not None:
        outbag_dir = os.path.join(args.outdir, args.folder)
    else:
        outbag_dir = inbag_dir
    os.makedirs(outbag_dir, exist_ok=True)

    print("# Starting appending images to {} rosbags!".format(len(bag_files)))

    image_topic_info = {
        '/camera_left/color/image_raw': 'rgb8',
        '/darknet_ros/detection_image': 'bgr8',
        '/image_with_bounding_boxes': 'bgr8',
        # '/camera_left/aligned_depth_to_color/image_raw': '16UC1', depth image
        }

    for counter, inbag_filename in enumerate(bag_files):
        bag_name = inbag_filename.split(".")[0]

        print("({}/{}): {}".format(counter + 1, len(bag_files), bag_name))

        create_deface_rosbag(inbag_dir, inbag_filename, deface_data_dir, outbag_dir, image_topic_info, args)
