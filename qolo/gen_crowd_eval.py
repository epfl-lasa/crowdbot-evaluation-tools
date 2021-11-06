# -*-coding:utf-8 -*-
'''
@File    :   gen_crowd_eval.py
@Time    :   2021/11/02
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

"""
evaluate the min. dist. from qolo and crowd density within 10m of qolo and save corresponding images
TODO: compare with detected pedestrain from the rosbag!
TODO: separate the plotting code into a independent part in eval/
TODO: read /rds_to_gui in `rds_network_ros/ToGui` to get start timestamp and stoptimestamp
"""

import os
import argparse

import numpy as np
import pandas as pd
# use TeX-like rendering: https://matplotlib.org/3.1.1/tutorials/text/mathtext.html
import matplotlib.pyplot as plt

from crowdbot_data import AllFrames

#%% utility functions
def compute_metrics(trks):

    # 1. all_det
    all_det = np.shape(trks)[0]

    # 2. within_det
    # r_square_within = (trks[:,0]**2 + trks[:,1]**2) < args.dist**2
    # within_det = np.shape(trks[r_square_within,:])[0]
    all_dist = np.linalg.norm(trks[:,[0,1]], axis=1)
    within_det5 = np.sum(np.less(all_dist, 5.0))
    within_det10 = np.sum(np.less(all_dist, 10.0))

    # 3. min_dist
    # b = np.random.rand(5,3)
    # b01_norm = np.linalg.norm(b[:,[0,1]], axis=1)
    min_dist = min(all_dist)

    # 4. crowd_density
    area_local5 = np.pi*5.0**2
    crowd_density5 = within_det5/area_local5
    area_local10 = np.pi*10.0**2
    crowd_density10 = within_det10/area_local10
    return (all_det, 
            within_det5, within_det10, 
            crowd_density5, crowd_density10, 
            min_dist)

# save crowd_density plotting
def save_cd_img(eval_dict, base_dir, seq_name):
    # unpack md data from eval_dict
    ts = eval_dict.get('timestamp')
    cd5 = eval_dict.get('crowd_density5')
    cd10 = eval_dict.get('crowd_density10')
    start_ts = eval_dict.get('start_command_ts')

    duration = np.max(ts)-np.min(ts)

    fig, ax = plt.subplots(figsize=(8, 4))

    # crowd_density chart
    l1, = ax.plot(ts-np.min(ts), cd5, linewidth=1, color='coral', label='x = 5')
    l2, = ax.plot(ts-np.min(ts), cd10, linewidth=1, color='navy', label='x = 10')

    # start_ts vertical line
    start_ts_offset = np.max([start_ts-np.min(ts), 0.0])
    ax.axvline(x=start_ts_offset, linestyle='--', linewidth=2, color='red')
    plt.text(x=start_ts_offset+1, y=0.45, 
        s='$t_s$={0:.1f}s'.format(start_ts_offset), 
        horizontalalignment='left',
        fontsize=10)

    ax.legend(handles=[l1, l2])
    ax.set_title("Crowd Density within x [m] of qolo ({0:.1f}s)".format(duration), fontsize=15)
    _ = ax.set_xlabel("t [s]")
    _ = ax.set_ylabel("Density [1/m^2]")

    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0, top=0.5)

    fig.tight_layout()
    cd_img_path = os.path.join(base_dir, seq_name+'_crowd_density.png')
    plt.savefig(cd_img_path, dpi=300) # png, pdf

# save min. dist. plotting
def save_md_img(eval_dict, base_dir, seq_name):
    # unpack md data from eval_dict
    ts = eval_dict.get('timestamp')
    md = eval_dict.get('min_dist')
    start_ts = eval_dict.get('start_command_ts')

    duration = np.max(ts)-np.min(ts)

    fig, ax = plt.subplots(figsize=(8, 4))

    # min_dist chart
    ax.plot(ts-np.min(ts), md, linewidth=1, color='coral')

    # start_ts vertical line
    start_ts_offset = np.max([start_ts-np.min(ts), 0.0])
    ax.axvline(x=start_ts_offset, linestyle='--', linewidth=2, color='red')
    plt.text(x=start_ts_offset+1, y=4.5, 
        s='$t_s$={0:.1f}s'.format(start_ts_offset), 
        horizontalalignment='left',
        fontsize=10)

    # y=0.3 horizontal line
    # 'xmin' in Axes.axhline denotes the maximum of the axis
    # ax.axhline(y=0.3, xmin=0.0, xmax=duration, linestyle='--', color='navy')
    ax.plot((0.0, duration), (0.3, 0.3), linestyle='--', color='navy')
    plt.text(x=duration/2, y=0.4, s=r'$\mathrm{dist}_{\min}=0.3$', 
         horizontalalignment='center', verticalalignment="baseline",
         fontsize=10)

    ax.set_title("Min. Distance of Pedestrain from qolo ({0:.1f}s)".format(duration), fontsize=15)
    _ = ax.set_xlabel("t [s]")
    _ = ax.set_ylabel("Distance [m]")
    
    ax.set_xlim(left=0.0)
    ax.set_ylim(bottom=0.0, top=5.0)

    fig.tight_layout()
    md_img_path = os.path.join(base_dir, seq_name+'_min_dist.png')
    plt.savefig(md_img_path, dpi=300) # png, pdf

#%% main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    # parser.add_argument('--dist', default=10., type=float, nargs='+', 
    #                     help='considered distance(s) in density evaluation')
    # directly set limitation as [5, 10]
    parser.add_argument('--save_img', dest='save_img', action='store_true',
                        help='plot and save crowd density image')
    parser.set_defaults(save_img=True)
    parser.add_argument('--overwrite', dest='overwrite', action='store_true',
                        help="Whether to overwrite existing rosbags (default: false)")
    parser.set_defaults(overwrite=False)
    parser.add_argument('--replot', dest='replot', action='store_true',
                        help="Whether to re-plot existing images (default: false)")
    parser.set_defaults(replot=False)
    args = parser.parse_args()

    allf = AllFrames(args)

    print("Starting extracting crowd_density files from {} rosbags!".format(allf.nr_seqs()))

    eval_res_dir = os.path.join(allf.metrics_dir)
    twist_dir = os.path.join(allf.source_data_dir, 'twist')

    if not os.path.exists(eval_res_dir):
        print("Crowd density images and npy will be saved in {}".format(eval_res_dir))
        os.makedirs(eval_res_dir, exist_ok=True)

    if not os.path.exists(twist_dir):
        print("ERROR: Please extract twist_stamped by using gen_twist_ts")

    for seq_idx in range(allf.nr_seqs()):

        seq = allf.seqs[seq_idx]
        print("({}/{}): {} with {} frames".format(seq_idx+1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)))

        # source
        source_twist_path = os.path.join(twist_dir, seq+'_twist_stamped.npy')
        source_twist = np.load(source_twist_path, allow_pickle=True).item()
        # print(source_twist.get("x")[1:200:5])
        # print(source_twist.get("zrot")[1:200:5])
        # starting: larger than zero
        start_idx = np.min([np.min(np.nonzero(source_twist.get("x"))), 
                           np.min(np.nonzero(source_twist.get("zrot")))])
        start_ts = source_twist.get("timestamp")[start_idx]
        print("starting timestamp: {}".format(start_ts))
        # TODO: extract end which is closet to the goal within a distance
        # TODO: consider adding a threshold


        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        crowd_eval_npy = os.path.join(eval_res_dir, seq+'_crowd_eval.npy')

        # only for plotting function update!
        if args.replot:
            crowd_eval_dict = np.load(crowd_eval_npy, allow_pickle=True).item()

            # figure1: crowd density
            save_cd_img(crowd_eval_dict, eval_res_dir, seq)

            # figure2: min. dist.
            save_md_img(crowd_eval_dict, eval_res_dir, seq)

            print("Replotted images!")
        else:
            if (not os.path.exists(crowd_eval_npy)) or (args.overwrite):

                # timestamp can be read from lidars/ folder
                stamp_file_path = os.path.join(allf.lidar_dir, seq+'_stamped.npy')
                lidar_stamped_dict = np.load(stamp_file_path, allow_pickle=True)
                ts = lidar_stamped_dict.item().get('timestamp')

                # targeted metrics and correspoding dtype
                attrs = ('all_det', 
                        'within_det5', 'within_det10', 
                        'crowd_density5', 'crowd_density10',
                        'min_dist')
                dtypes = (np.uint8, 
                            np.uint8, np.uint8, 
                            np.float32, np.float32,
                            np.float32)

                crowd_eval_list_dict = {k:[] for k in attrs}

                num_msgs_between_logs = 200
                nr_frames = allf.nr_frames(seq_idx)

                for fr_idx in range(nr_frames):

                    _, _, _, trks = allf[seq_idx, fr_idx]

                    metrics = compute_metrics(trks)

                    if fr_idx % num_msgs_between_logs == 0 or fr_idx >= nr_frames - 1:
                        print("Seq {}/{} - Frame {}/{}: filtered/overall boxes within 10m: {}/{}"
                        .format(seq_idx+1, allf.nr_seqs(),
                                fr_idx+1, nr_frames, 
                                metrics[2], metrics[0]))

                    # update value for each attr
                    for idx, val in enumerate(metrics):
                        crowd_eval_list_dict[attrs[idx]].append(val)

                crowd_eval_dict = {name:np.asarray(crowd_eval_list_dict[attrs[idx]], dtype=dtype) for idx, (name, dtype) in enumerate(zip(attrs, dtypes))}

                # other attributes
                crowd_eval_dict.update({'timestamp': ts})

                crowd_eval_dict.update({'start_command_ts': start_ts})

                # avg_min_dict = np.average(crowd_eval_dict['min_dict'])
                for attr in attrs:
                    avg_attr = 'avg_' + attr
                    crowd_eval_dict.update({avg_attr: np.average(crowd_eval_dict[attr])})

                np.save(crowd_eval_npy, crowd_eval_dict)

                if args.save_img:
                    # figure1: crowd density
                    save_cd_img(crowd_eval_dict, eval_res_dir, seq)

                    # figure2: min. dist.
                    save_md_img(crowd_eval_dict, eval_res_dir, seq)
            else:
                print("Detecting the generated {} already existed!".format(crowd_eval_npy))
                print('Will not overwrite. If you want to overwrite, use flag --overwrite')
                continue

    print("Finish crowd evaluation!")
