#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   gen_animation.py
@Date created  :   2021/11/28
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides the workflow to generate videos and gifs from image folder
with ffmpeg.
"""
# =============================================================================


import os
import fnmatch
import argparse

import moviepy.editor as mpy

from crowdbot_data import CrowdBotDatabase

# need ffmpeg
def img_dir2video_ffmpeg(img_dir, video_path):
    """Generate video using ffmpeg via os.system()"""

    # cmd: ffmpeg -y -r 15 -pattern_type glob -i 'tmp/*.png' -c:v libx264 -vf fps=30 -pix_fmt yuv420p 'tmp/frames.mp4'
    header_cmd = "ffmpeg -y -r 15 -pattern_type glob -i "
    in_images = "'{}/*.png'".format(img_dir)
    middle_cmd = " -c:v libx264 -vf fps=30 -pix_fmt yuv420p "
    out_video = "'{}'".format(video_path)
    cmd = header_cmd + in_images + middle_cmd + out_video
    os.system(cmd)


# need ffmpeg
def video2gif_ffmpeg(video_path, gif_path, w=640, h=480):
    """Generate gif from video using ffmpeg via os.system()"""

    # cmd: ffmpeg -i lapse.mp4 -r 12 -s 640x360 output.gif
    header_cmd = "ffmpeg -i "
    in_images = "'{}'".format(video_seq_filepath)
    middle_cmd = " -r 12 -s {}x{} ".format(w, h)
    out_gif = "{}".format(gif_path)
    cmd = header_cmd + in_images + middle_cmd + out_gif
    os.system(cmd)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="nocam_rosbags",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    for seq_idx in range(cb_data.nr_seqs()):

        seq = cb_data.seqs[seq_idx]
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, cb_data.nr_seqs(), seq, cb_data.nr_frames(seq_idx)
            )
        )

        # seq source: data/xxxx_processed/viz_imgs/seq
        img_seq_dir = os.path.join(cb_data.img3d_dir, seq)
        if not os.path.exists(img_seq_dir):
            print("please generate images with gen_viz_img_o3d.py")

        # seq dest: data/xxxx_processed/videos/seq.mp4
        os.makedirs(cb_data.video_dir, exist_ok=True)
        video_seq_filepath = os.path.join(cb_data.video_dir, seq + ".mp4")

        if not os.path.exists(video_seq_filepath):
            print("Images will be converted into {}".format(video_seq_filepath))

            # img_dir2video_ffmpeg(img_seq_dir, video_seq_filepath)

            # video2gif_ffmpeg(
            #     video_seq_filepath, video_seq_filepath.replace("mp4", "gif")
            # )

            img_files = sorted(fnmatch.filter(os.listdir(img_seq_dir), "*.png"))
            print("{} frames detected".format(len(img_files)))
            img_seq = [os.path.join(img_seq_dir, img) for img in img_files]

            clip = mpy.ImageSequenceClip(img_seq, fps=30)
            # .mp4 video
            clip.write_videofile(video_seq_filepath, fps=15)
            # .gif
            clip.resize((720, 480)).write_gif(
                video_seq_filepath.replace("mp4", "gif"), fps=15
            )

        else:
            print("{} already generated!!!".format(video_seq_filepath))
            continue
