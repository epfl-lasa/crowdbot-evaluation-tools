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
within rosbags to ensure anonymization. `multiprocessing` package has been used
to accelerate the speed.
Example: python deface_images.py -f 0424_shared_control
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
import glob
import argparse

import multiprocessing as mp
from multiprocessing import Pool
from itertools import product

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter


def deface_img_io(in_img, out_img, thresh=0.2, backend='opencv'):

    deface_cmd = "deface {} -o {} -t {} --backend {}".format(
        in_img, out_img, thresh, backend
    )

    os.system(deface_cmd)


def deface_img(in_img, args):

    img_dir = in_img.rsplit('/', 1)[0]
    img_name = in_img.rsplit('/', 1)[-1]
    if not os.path.exists(img_dir):
        os.makedirs(img_dir)
    out_img = os.path.join(img_dir + '_deface', img_name)

    deface_cmd = "deface {} -o {} -t {} --backend {}".format(
        in_img, out_img, args.thresh, args.backend
    )
    os.system(deface_cmd)


def deface_img_wrapper(args):
    return deface_img(*args)


def naive_example():
    in_img = 'examples/city.jpg'
    out_img = 'examples/city_anonymized.jpg'
    thresh = 0.2

    # more options can be found: https://github.com/ORB-HD/deface#cli-usage-and-options-summary
    deface_img_io(in_img, out_img, thresh)


def deface_img_dir(deface_bag_dir, args, print_info=''):
    """deface images in the example"""
    image_topics = [
        '/camera_left/color/image_raw',
        '/darknet_ros/detection_image',
        '/image_with_bounding_boxes',
    ]
    for topic_id, topic in enumerate(image_topics):
        topic_dir = os.path.join(deface_bag_dir, topic[1:])
        deface_topic_dir = os.path.join(deface_bag_dir, topic[1:] + '_deface')
        if not os.path.exists(deface_topic_dir):
            os.makedirs(deface_topic_dir)

        topic_images = sorted(glob.glob("{}/*.png".format(topic_dir)))
        defaced_images = sorted(glob.glob("{}/*.png".format(deface_topic_dir)))
        print(len(topic_images), len(defaced_images))
        already_defaced = (len(topic_images) == len(defaced_images))
        if already_defaced and not args.overwrite:
            print("If you want to overwrite {}, use flag --overwrite".format(topic))
            continue

        print("({}, Topic: {}/3)".format(print_info, topic_id + 1))
        print("{} images: {}".format(len(topic_images), topic_dir))

        if args.n_proc >= 2:
            # construct mp.Pool
            input_args = list(product(topic_images, [args]))
            with Pool(processes=args.n_proc) as p:
                p.map(deface_img_wrapper, input_args)
        else:

            for img_id, in_img in enumerate(topic_images):
                # out_img = os.path.join(deface_topic_dir, img)
                print(
                    "({}, Topic: {}/3, Image: {}/{})".format(
                        print_info, topic_id + 1, img_id + 1, len(topic_images)
                    )
                )

                # in_img = os.path.join(topic_dir, img)
                deface_img(in_img, args)


#%% main pipeline
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="deface images extracted from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="0325_rds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    # more false positives > 0.2
    # more robust blurring < 0.2
    parser.add_argument(
        "-t",
        "--thresh",
        default=0.1,
        type=float,
        help="detection threshold (tune this to trade off between false positive and false negative rate).",
    )
    parser.add_argument(
        "--backend",
        default="opencv",
        choices=["opencv", "auto", "onnxrt"],
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--n_proc",
        type=int,
        default=6,
        help="number of processes to deface images",
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

    if args.n_proc >= 2:
        # setup multiprocessing
        mp.set_start_method('spawn')

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

        print("({}/{}): {}".format(counter + 1, len(bag_files), deface_bag_dir))
        print_info = "Seq: {}/{}".format(counter + 1, len(bag_files))

        filtered_bag_dirpath = os.path.join(deface_data_dir, bag_name)

        start = time.time()

        if not os.path.exists(filtered_bag_dirpath):
            os.makedirs(filtered_bag_dirpath)
        deface_img_dir(deface_bag_dir, args, print_info)

        print("--- {} seconds ---".format(time.time() - start))

    print("Finish defacing out all images from raw rosbag!")
