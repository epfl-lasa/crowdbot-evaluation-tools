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
TODO: consider the ylim of each plot
TODO: separate the plotting code into a independent part in eval/
"""

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from crowdbot_data import AllFrames

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
    parser.add_argument('--dist', default=10., type=float,
                        help='considered distance in density evaluation')
    parser.add_argument('--save_img', dest='save_img', action='store_true',
                        help='plot and save crowd density image')
    parser.set_defaults(save_img=True)
    args = parser.parse_args()

    allf = AllFrames(args)

    print("Starting extracting crowd_density files from {} rosbags!".format(allf.nr_seqs()))

    # seq dest: data/xxxx_processed/viz_imgs/seq
    eval_res_dir = os.path.join(allf.metrics_dir)

    if not os.path.exists(eval_res_dir):
        print("Crowd density images and npy will be saved in {}".format(eval_res_dir))
        os.makedirs(eval_res_dir, exist_ok=True)

    # print(allf.nr_seqs())

    for seq_idx in range(allf.nr_seqs()):

        seq = allf.seqs[seq_idx]
        print("({}/{}): {} with {} frames".format(seq_idx+1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)))

        crowd_eval_npy = os.path.join(eval_res_dir, seq+'_crowd_eval.npy')

        if not os.path.exists(crowd_eval_npy):

            # timestamp can be read from lidars/ folder
            stamp_file_path = os.path.join(allf.lidar_dir, seq+'_stamped.npy')
            lidar_stamped_dict = np.load(stamp_file_path, allow_pickle=True)
            ts_np = lidar_stamped_dict.item().get('timestamp')
            all_det_list = []
            within_det_list = []
            min_dist_list = []
            crowd_density_list = []

            for fr_idx in range(allf.nr_frames(seq_idx)):

                _, _, _, trks = allf[seq_idx, fr_idx]

                # 1. all_det
                all_det = np.shape(trks)[0]

                # 2. within_det
                # r_square_within = (trks[:,0]**2 + trks[:,1]**2) < args.dist**2
                # within_det = np.shape(trks[r_square_within,:])[0]
                all_dist = np.linalg.norm(trks[:,[0,1]], axis=1)
                within_det = np.sum(np.less(all_dist, args.dist))
                print("Seq {}/{} - Frame {}/{}: filtered/overall boxes within {}m: {}/{}"
                      .format(seq_idx+1, allf.nr_seqs(),
                              fr_idx+1, allf.nr_frames(seq_idx), 
                              args.dist, within_det, all_det))

                # 3. min_dist
                # b = np.random.rand(5,3)
                # b01_norm = np.linalg.norm(b[:,[0,1]], axis=1)
                min_dist = min(all_dist)

                # 4. crowd_density
                area_local_eval = np.pi*args.dist**2
                crowd_density = within_det/area_local_eval

                # append into list
                all_det_list.append(all_det)
                within_det_list.append(within_det)
                min_dist_list.append(min_dist)
                crowd_density_list.append(crowd_density)

            ad_np = np.asarray(all_det_list, dtype=np.uint8)
            wd_np = np.asarray(within_det_list, dtype=np.uint8)
            md_np = np.asarray(min_dist_list, dtype=np.float32)
            cd_np = np.asarray(crowd_density_list, dtype=np.float32)
            crowd_eval_dict = {'timestamp': ts_np, 
                            'all_det': ad_np, 
                            'within_det': wd_np, 
                            'min_dist': md_np, 
                            'crowd_density': cd_np}
            np.save(crowd_eval_npy, crowd_eval_dict)

            if args.save_img:
                # figure1: crowd density
                fig, ax = plt.subplots(figsize=(8, 4))

                ax.plot(ts_np - np.min(ts_np), cd_np, linewidth=1)
                ax.set_title("Crowd Density within 10m of qolo", fontsize=15)
                _ = ax.set_xlabel("t [s]")
                _ = ax.set_ylabel("Density [1/m^2]")

                fig.tight_layout()
                cd_img_path = os.path.join(eval_res_dir, seq+'_crowd_density.png')
                plt.savefig(cd_img_path, dpi=300) # png, pdf

                # figure2: min. dist.
                fig2, ax2 = plt.subplots(figsize=(8, 4))

                ax2.plot(ts_np - np.min(ts_np), md_np, linewidth=1)
                ax2.set_title("Min. Distance of Pedestrain from qolo", fontsize=15)
                _ = ax2.set_xlabel("t [s]")
                _ = ax2.set_ylabel("Distance [m]")

                fig2.tight_layout()
                md_img_path = os.path.join(eval_res_dir, seq+'_min_dist.png')
                plt.savefig(md_img_path, dpi=300) # png, pdf
        else:
            print("Crowd density of {} already generated!!!".format(seq))
            continue
