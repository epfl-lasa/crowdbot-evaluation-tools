#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   deface_images.py
@Date created  :   2022/02/04
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides pipeline to deface images extracted from multiple topics
within rosbags to ensure anonymization
Example: python deface_images.py -f 1203_manual
"""
# =============================================================================
"""
TODO:
1. tune suitable parameters for each dataset!
"""
# =============================================================================


import os
import sys
import time
import fnmatch
import argparse

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter


def deface_img(in_img, out_img, thresh=0.2, backend='opencv'):

    # ValueError: This ORT build has ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'] enabled. Since ORT 1.9, you are required to explicitly set the providers parameter when instantiating InferenceSession. For example, onnxruntime.InferenceSession(..., providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'], ...)

    deface_cmd = "deface {} -o {} -t {} --backend {}".format(
        in_img, out_img, thresh, backend
    )

    os.system(deface_cmd)


def naive_example():
    in_img = 'examples/city.jpg'
    out_img = 'examples/city_anonymized.jpg'
    thresh = 0.2

    # more options can be found: https://github.com/ORB-HD/deface#cli-usage-and-options-summary
    deface_img(in_img, out_img, thresh)


def deface_img_dir(deface_bag_dir, args, print_info=''):
    """deface images in the example"""
    topic_dirs = [
        '/camera_left/color/image_raw',
        '/darknet_ros/detection_image',
        '/image_with_bounding_boxes',
    ]
    for topic_id, topic in enumerate(topic_dirs):
        topic_dir = os.path.join(deface_bag_dir, topic[1:])
        deface_topic_dir = os.path.join(deface_bag_dir, topic[1:] + '_deface')
        if not os.path.exists(deface_topic_dir):
            os.makedirs(deface_topic_dir)
        topic_images = fnmatch.filter(os.listdir(topic_dir), "*.png")

        # print(
        #     "({}/3) Start defacing {} images from {}".format(
        #         topic_id + 1, len(topic_images), topic
        #     )
        # )

        start = time.time()
        for img_id, img in enumerate(topic_images):
            in_img = os.path.join(topic_dir, img)
            out_img = os.path.join(deface_topic_dir, img)
            print(
                "({}, Topic: {}/3, Image: {}/{})".format(
                    print_info, topic_id + 1, img_id + 1, len(topic_images)
                )
            )
            if not os.path.exists(out_img):
                deface_img(in_img, out_img, thresh=args.thresh, backend='opencv')
            else:
                print("{} exists!".format(out_img))
        print("--- %s seconds ---" % (time.time() - start))


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="deface images extracted from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="1203_shared_control",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "-t",
        "--thresh",
        default=0.2,
        type=float,
        help="detection threshold (tune this to trade off between false positive and false negative rate).",
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

    # destination: exported images and camera parameters
    deface_data_dir = os.path.join(rosbag_dir + "_filtered")
    if not os.path.exists(deface_data_dir):
        # os.makedirs(deface_data_dir)
        sys.exit("Please use bag_filter_image.py to extract image data first!")

    print("# Starting defacing images from {} rosbags!".format(len(bag_files)))

    for counter, bf in enumerate(bag_files):
        bag_name = bf.split(".")[0]
        deface_bag_dir = os.path.join(deface_data_dir, bag_name)
        if not os.path.exists(deface_bag_dir):
            sys.exit("Please use bag_filter_image.py to extract image data first!")

        # print("({}/{}): {}".format(counter + 1, len(bag_files), deface_bag_dir))
        print_info = "Seq de: {}/{}".format(counter + 1, len(bag_files))

        filtered_bag_dirpath = os.path.join(deface_data_dir, bag_name)

        if not os.path.exists(filtered_bag_dirpath):
            os.makedirs(filtered_bag_dirpath)
        deface_img_dir(deface_bag_dir, args, print_info)

    print("Finish defacing out all images from raw rosbag!")
