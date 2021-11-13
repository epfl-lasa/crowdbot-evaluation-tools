# -*-coding:utf-8 -*-
"""
@File    :   gen_video.py
@Time    :   2021/10/28
@Author  :   Yujie He
@Version :   1.1
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""


import os
import argparse
from crowdbot_data import AllFrames

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-b",
        "--base",
        default="/home/crowdbot/Documents/yujie/crowdbot_tools",
        type=str,
        help="base folder, i.e., the path of the current workspace",
    )
    parser.add_argument(
        "-d",
        "--data",
        default="data",
        type=str,
        help="data folder, i.e., the name of folder that stored extracted raw data and processed data",
    )
    parser.add_argument(
        "-f",
        "--folder",
        default="nocam_rosbags",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    args = parser.parse_args()

    allf = AllFrames(args)

    for seq_idx in range(allf.nr_seqs()):

        seq = allf.seqs[seq_idx]
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)
            )
        )

        # seq source: data/xxxx_processed/viz_imgs/seq
        img_seq_dir = os.path.join(allf.imgs_dir, seq)
        # seq dest: data/xxxx_processed/videos/seq.mp4
        os.makedirs(allf.video_dir, exist_ok=True)
        video_seq_filepath = os.path.join(allf.video_dir, seq + ".mp4")

        if not os.path.exists(video_seq_filepath):
            print("Images will be converted into {}".format(video_seq_filepath))

            # method1: generate video using ffmpeg via os.system()
            # TODO: consider using ffmpeg-py
            # os.system("ffmpeg -y -r 15 -pattern_type glob -i 'tmp/*.png' -c:v libx264 -vf fps=30 -pix_fmt yuv420p 'tmp/frames.mp4'")
            header_cmd = "ffmpeg -y -r 15 -pattern_type glob -i "
            in_images = "'{}/*.png'".format(img_seq_dir)
            middle_cmd = " -c:v libx264 -vf fps=30 -pix_fmt yuv420p "
            out_video = "'{}'".format(video_seq_filepath)
            cmd = header_cmd + in_images + middle_cmd + out_video
            os.system(cmd)

            # images to gifs: ffmpeg -i lapse.mp4 -r 12 -s 640x360 output.gif
            header_cmd = "ffmpeg -i "
            in_images = "'{}'".format(video_seq_filepath)
            middle_cmd = " -r 12 -s 640x480 "
            out_gif = "{}".format(video_seq_filepath.replace("mp4", "gif"))
            cmd = header_cmd + in_images + middle_cmd + out_gif
            os.system(cmd)

        else:
            print("{} already generated!!!".format(video_seq_filepath))
            continue
