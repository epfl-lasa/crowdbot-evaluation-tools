import os, sys
import fnmatch
import argparse
import moviepy.editor as mpy

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter

parser = argparse.ArgumentParser(description="convert data from rosbag")

# parser.add_argument(
#     "-i",
#     "--input",
#     required=True,
#     type=str,
#     help="input image folder",
# )
# parser.add_argument(
#     "-o",
#     "--output",
#     default='videos',
#     type=str,
#     help="output video folder",
# )
parser.add_argument(
    "-f",
    "--folder",
    default="1203_manual",
    type=str,
    help="different subfolder in rosbag/ dir",
)
parser.add_argument(
    "--seq",
    default="2021-12-03-19-27-16",
    type=str,
    help="specific sequence in the subfolder",
)
parser.add_argument(
    "--all",
    dest="process_all",
    action="store_true",
    help="Process all sequences and disable single sequences",
)
parser.add_argument(
    "--overwrite",
    dest="overwrite",
    action="store_true",
    help="Whether to overwrite existing rosbags (default: false)",
)
parser.add_argument(
    "--deface_topic",
    default="/camera_left/color/image_raw_deface",
    type=str,
    help="specific topic in the subfolder",
)
parser.set_defaults(overwrite=False)
args = parser.parse_args()

cb_data = CrowdBotDatabase(args.folder)
if args.seq is None or args.process_all:
    seqs = [cb_data.seqs[seq_idx] for seq_idx in range(cb_data.nr_seqs())]
else:
    seqs = [args.seq]


def main():
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    filtered_data_dirpath = os.path.join(rosbag_dir + "_filtered")
    if not os.path.exists(filtered_data_dirpath):
        sys.exit("Please use deface_images.py to extract and deface first!")
    # bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))
    for counter, seq in enumerate(seqs):
        # seq = bf.split(".")[0]
        deface_img_dir = os.path.join(filtered_data_dirpath, seq, args.deface_topic[1:])

        video_strs = args.deface_topic.rsplit("/", 1)
        video_seq_filepath = os.path.join(filtered_data_dirpath, seq, video_strs[0][1:], video_strs[1]+'.mp4')


        print("Images will be converted into {}".format(video_seq_filepath))


        img_files = sorted(fnmatch.filter(os.listdir(deface_img_dir), "*.png"))
        if img_files:
            print("{} frames detected".format(len(img_files)))
            img_seq = [os.path.join(deface_img_dir, img) for img in img_files]

            clip = mpy.ImageSequenceClip(img_seq, fps=30)
            # .mp4 video
            clip.write_videofile(video_seq_filepath, fps=15)
            # .gif
            # clip.resize((720, 480)).write_gif(
            #     video_seq_filepath.replace("mp4", "gif"), fps=15
            # )


if __name__ == "__main__":
    main()
