# -*-coding:utf-8 -*-
'''
@File    :   gen_video.py
@Time    :   2021/10/28
@Author  :   Yujie He
@Version :   1.1
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''


import os
import argparse
from crowdbot_data import AllFrames

# pip install ffmpeg-python
import ffmpeg

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

    for seq_idx in range(allf.nr_seqs()):

        seq = allf.seqs[seq_idx]
        print("({}/{}): {} with {} frames".format(seq_idx+1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)))

        # seq source: data/xxxx_processed/viz_imgs/seq
        img_seq_dir = os.path.join(allf.imgs_dir, seq)
        # seq dest: data/xxxx_processed/videos/seq.mp4
        os.makedirs(allf.video_dir, exist_ok=True)
        video_seq_filepath = os.path.join(allf.video_dir, seq+'.mp4')

        if not os.path.exists(video_seq_filepath):
            print("Images will be converted into {}".format(video_seq_filepath))

            # use ffmpeg-py
            (
                ffmpeg
                .input('{}/*.png'.format(img_seq_dir), pattern_type='glob', framerate=25)
                .output(video_seq_filepath)
                .run()
            )

            # old verion (deprecated)
            # cat *.png | ffmpeg -f image2pipe -i - output.mp4
            # ffmpeg -framerate 10 -pattern_type glob -i '*.png' -c:v libx264 -pix_fmt yuv420p out.mp4
            # TODO: fix bugs by integrating `%04d`
            # """
            # generate video using ffmpeg via os.system()
            # os.system("ffmpeg -y -r 15 -pattern_type glob -i 'tmp/*.png' -c:v libx264 -vf fps=30 -pix_fmt yuv420p 'tmp/frames.mp4'")
            header_cmd = "ffmpeg -y -r 15 -pattern_type glob -i "
            in_images = "'{}/*.png'".format(img_seq_dir) # "'tmp/*.png'"
            middle_cmd = " -c:v libx264 -vf fps=30 -pix_fmt yuv420p "
            out_video = "'{}'".format(video_seq_filepath) # "'tmp/frames.mp4'"
            cmd = header_cmd+in_images+middle_cmd+out_video
            os.system(cmd)
            # """
        else:
            print("{} already generated!!!".format(video_seq_filepath))
            continue