# -*-coding:utf-8 -*-
'''
@File    :   eval_qolo.py
@Time    :   2021/11/09
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

import os
import copy
import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from crowdbot_data import AllFrames

#%% utility functions to evaluate qolo
# TODO: check data source from pose2d (odom) or tf_qolo
def compute_time_path(qolo_twist, qolo_pose2d):
    # 1. calculate starting timestamp based on nonzero twist command
    # starting: larger than zero
    start_idx = np.min([np.min(np.nonzero(qolo_twist.get("x"))), 
                        np.min(np.nonzero(qolo_twist.get("zrot")))])
    start_ts = qolo_twist.get("timestamp")[start_idx]
    print("starting timestamp: {}".format(start_ts))

    # 2. calculate ending timestamp based on closest point to goal
    pose_ts = qolo_pose2d.get("timestamp")
    pose_x = qolo_pose2d.get("x")
    pose_y = qolo_pose2d.get("y")
    theta = qolo_pose2d.get("theta")
    ## borrow from evalMetricsPathAndTimeToGoal.py
    angle_init = np.sum(theta[9:19])/10.0
    goal = np.array([np.cos(angle_init), np.sin(angle_init)])*20.0

    # determine when the closest point to the goal is reached
    min_dist2goal = np.inf
    end_idx = -1
    for idx in range(len(pose_ts)):
        dist2goal = np.sqrt((pose_x[idx] - goal[0])**2 + (pose_y[idx] - goal[1])**2)
        if dist2goal < min_dist2goal:
            min_dist2goal = dist2goal
            end_idx = idx

    path_length2goal = np.sum(np.sqrt(np.diff(pose_x[:end_idx])**2 
                                    + np.diff(pose_y[:end_idx])**2))
    end_ts = pose_ts[end_idx]
    duration2goal = end_ts - start_ts
    print("ending timestamp: {}".format(end_ts))
    print("Duration: {}s".format(duration2goal))
    ## borrow from evalMetricsPathAndTimeToGoal.py

    return (start_ts, end_ts, duration2goal, path_length2goal, end_idx, goal, min_dist2goal)

# https://github.com/epfl-lasa/qolo-evaluation/blob/main/scripts/test_metrics.py
# https://github.com/epfl-lasa/qolo-evaluation/blob/main/notebook/crowd_evaluation.py
def compute_fluency(qolo_twist):
    
    vel = qolo_twist.get("x")
    omega = qolo_twist.get("zrot")
    v_max, w_max = np.max(vel), np.max(omega)

    fluency_v = []
    fluency_w = []
    
    # TODO: what's the difference between starting from 2 or 1 -> matlab?
    for idx in range(1, len(vel)):
        if vel[idx] or omega[idx]:
            fluency_v.append(1 - np.abs(vel[idx] - vel[idx-1])/v_max)
            fluency_w.append(1 - np.abs(omega[idx] - omega[idx-1])/w_max)

    fluencyv = np.mean(np.array(fluency_v))
    fluencyv_sd = np.std(np.array(fluency_v))
    fluencyw = np.mean(np.array(fluency_w))
    fluencyw_sd = np.std(np.array(fluency_w))

    if fluencyv < fluencyw:
        return (fluencyv, fluencyv_sd)
    else:
        return (fluencyw, fluencyw_sd)

def save_path_img(qolo_pose2d, time_path_computed, base_dir, seq_name):
    pose_x = qolo_pose2d.get("x")
    pose_y = qolo_pose2d.get("y")
    duration2goal = time_path_computed[2]
    path_length2goal = time_path_computed[3]
    end_idx = time_path_computed[4]
    goal = time_path_computed[5]
    min_dist2goal = time_path_computed[6]

    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(pose_x[:end_idx], pose_y[:end_idx], "orangered", linewidth=2,
        label="path (l=%.1f m, t=%.1f s)" % (path_length2goal, duration2goal))
    ax.plot(pose_x[end_idx:], pose_y[end_idx:], "skyblue", linewidth=2,
        label="remaining path")
    ax.plot([goal[0]], [goal[1]], "kx", label="goal")
    # ax.plot([0.0], [0.0], "ks", label="start")
    ax.legend(fontsize='x-small')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # adjust plots with equal axis aspect ratios
    ax.axis('equal')
    
    ax.set_title("Path. Closest distance to the goal={0:.1f}m".format(min_dist2goal))
    # plt.show()
    fig.tight_layout()
    path_img_path = os.path.join(base_dir, seq_name+'_path.png')
    plt.savefig(path_img_path, dpi=300) # png, pdf

def save_motion_img(qolo_eval_dict, base_dir, seq_name):
    ts = qolo_eval_dict['timestamp']
    start_ts = qolo_eval_dict.get('start_command_ts')
    duration2goal = qolo_eval_dict.get('duration2goal')
    
    new_start_ts = np.max([start_ts-np.min(ts), 0.0])
    new_end_ts = new_start_ts + duration2goal

    plot_attr = ('x', 'zrot', 'x_acc', 'zrot_acc', 'x_jerk', 'zrot_jerk')
    unit = ('$V$ [$m/s$]', '$V_w$ [$rad/s$]', '$a$ [$m/s^2$]', '$a_w$ [$rad/s^2$]', '$J$ [$m/s^3$]', '$J_w$ [$rad/s^3$]')

    # ref: https://jakevdp.github.io/PythonDataScienceHandbook/04.08-multiple-subplots.html
    fig, ax = plt.subplots(3, 2, sharex='col',figsize=(10, 4))
    fig.subplots_adjust(hspace=0.4, wspace=0.4)
    # ref: https://matplotlib.org/stable/gallery/subplots_axes_and_figures/axes_zoom_effect.html#sphx-glr-gallery-subplots-axes-and-figures-axes-zoom-effect-py
    # ref: https://matplotlib.org/stable/gallery/shapes_and_collections/artist_reference.html#sphx-glr-gallery-shapes-and-collections-artist-reference-py
    import matplotlib.patches as mpatches
    # (left, bottom), width, height
    # rect = mpatches.Rectangle((new_start_ts, 0), duration2goal, 10, facecolor="red", alpha=0.1)
    # ax.add_patch(rect)

    for i in range(3):
        for j in range(2):
            xx = ts-np.min(ts)
            yy = qolo_eval_dict[plot_attr[i*2+j]]
            ax[i, j].plot(xx, yy, linewidth=0.8, color='purple')
            ax[i, j].axvline(x=new_start_ts, linestyle='--', linewidth=1.5, color='red')
            ax[i, j].axvline(x=new_end_ts, linestyle='--', linewidth=1.5, color='red')

            # ref: https://stackoverflow.com/questions/50753721/can-not-reset-the-axes
            ax[i, j].add_patch(mpatches.Rectangle((new_start_ts, 0), duration2goal, 100, facecolor="red", alpha=0.2))

            ax[i, j].set_ylabel(unit[i*2+j])
            if i==2:
                ax[i, j].set_xlabel("t [s]")
            
            ax[i, j].set_ylim(bottom=0.0)
    fig.tight_layout()
    twist_acc_jerk_img_path = os.path.join(base_dir, seq_name+'_twist_acc_jerk.png')
    plt.savefig(twist_acc_jerk_img_path, dpi=300) # png, pdf

#%% main function
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='convert data from rosbag')
    
    parser.add_argument('-b', '--base', default='/home/crowdbot/Documents/yujie/crowdbot_tools', type=str,
                        help='base folder, i.e., the path of the current workspace')
    parser.add_argument('-d', '--data', default='data', type=str,
                        help='data folder, i.e., the name of folder that stored extracted raw data and processed data')
    parser.add_argument('-f', '--folder', default='nocam_rosbags', type=str,
                        help='different subfolder in rosbag/ dir')
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

    print("Starting evaluating qolo from {} sequences!".format(allf.nr_seqs()))

    eval_res_dir = os.path.join(allf.metrics_dir)

    if not os.path.exists(eval_res_dir):
        print("Result images and npy will be saved in {}".format(eval_res_dir))
        os.makedirs(eval_res_dir, exist_ok=True)

    for seq_idx in range(allf.nr_seqs()):

        seq = allf.seqs[seq_idx]
        print("({}/{}): {} with {} frames".format(seq_idx+1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)))

        # load pose2d, twist, acc (can be read in loop)
        pose2d_dir = os.path.join(allf.source_data_dir, 'pose2d')
        qolo_pose2d_path = os.path.join(pose2d_dir, seq+'_pose2d.npy')
        if not os.path.exists(qolo_pose2d_path):
            print("ERROR: Please extract pose2d by using pose2d2npy.py")
        qolo_pose2d = np.load(qolo_pose2d_path, allow_pickle=True).item()

        twist_dir = os.path.join(allf.source_data_dir, 'twist')
        qolo_twist_path = os.path.join(twist_dir, seq+'_twist.npy')
        qolo_twist_sampled_path = os.path.join(twist_dir, seq+'_twist_sampled.npy')
        if not os.path.exists(qolo_twist_path):
            print("ERROR: Please extract twist_stamped by using twist2npy.py")
        qolo_twist = np.load(qolo_twist_path, allow_pickle=True).item()
        qolo_twist_sampled = np.load(qolo_twist_sampled_path, allow_pickle=True).item()

        acc_dir = os.path.join(allf.source_data_dir, 'acc')
        qolo_acc_sampled_path = os.path.join(acc_dir, seq+'_acc_sampled.npy')
        if not os.path.exists(qolo_acc_sampled_path):
            print("ERROR: Please extract acc by using twist2npy.py")
        qolo_acc_sampled = np.load(qolo_acc_sampled_path, allow_pickle=True).item()

        # 00. compute (start_ts, end_idx, end_ts, duration2goal, path_length2goal)
        time_path_computed = compute_time_path(qolo_twist_sampled, qolo_pose2d)

        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        qolo_eval_npy = os.path.join(eval_res_dir, seq+'_qolo_eval.npy')

        # only for plotting function update!
        if args.replot:
            qolo_eval_npy = np.load(qolo_eval_npy, allow_pickle=True).item()

            # figure1: path
            save_path_img(qolo_pose2d, time_path_computed, eval_res_dir, seq)

            # figure2: viz twist, acc, jerk
            save_motion_img(qolo_eval_dict, eval_res_dir, seq)

            print("Replot images!")
        else:
            if (not os.path.exists(qolo_eval_npy)) or (args.overwrite):

                # timestamp can be read from lidars/ folder
                stamp_file_path = os.path.join(allf.lidar_dir, seq+'_stamped.npy')
                lidar_stamped_dict = np.load(stamp_file_path, allow_pickle=True)
                ts = lidar_stamped_dict.item().get('timestamp')

                attrs = ('jerk', 'agreement', 'fluency')

                # 0. qolo_eval_dict (include twist, acc)
                qolo_eval_dict = copy.deepcopy(qolo_twist_sampled)
                qolo_eval_dict.update({'x_acc': qolo_acc_sampled['x']})
                qolo_eval_dict.update({'zrot_acc': qolo_acc_sampled['zrot']})

                # 1. jerk
                from twist2npy import compute_motion_derivative
                qolo_jerk_sampled = compute_motion_derivative(qolo_acc_sampled)
                qolo_eval_dict.update({'x_jerk': qolo_jerk_sampled['x']})
                qolo_eval_dict.update({'zrot_jerk': qolo_jerk_sampled['zrot']})
                qolo_eval_dict.update({'avg_x_jerk': np.average(qolo_jerk_sampled['x'])})
                qolo_eval_dict.update({'avg_zrot_jerk': np.average(qolo_jerk_sampled['zrot'])})
                
                # 2. fluency
                fluency = compute_fluency(qolo_twist_sampled)
                qolo_eval_dict.update({'avg_fluency': fluency[0]})
                qolo_eval_dict.update({'std_fluency': fluency[1]})

                # TODO: 3. agreement with command sampled!
                # need to extract command

                # 4. path related-metrics
                qolo_eval_dict.update({'start_command_ts': time_path_computed[0]})
                qolo_eval_dict.update({'end_command_ts': time_path_computed[1]})
                qolo_eval_dict.update({'duration2goal': time_path_computed[2]})
                qolo_eval_dict.update({'path_lenth2goal': time_path_computed[3]})

                np.save(qolo_eval_npy, qolo_eval_dict)

                if args.save_img:
                    # figure1: path
                    save_path_img(qolo_pose2d, time_path_computed, eval_res_dir, seq)

                    # figure2: viz twist, acc, jerk
                    save_motion_img(qolo_eval_dict, eval_res_dir, seq)
            else:
                print("Detecting the generated {} already existed!".format(qolo_eval_npy))
                print('Will not overwrite. If you want to overwrite, use flag --overwrite')
                continue


    print("Finish qolo evaluation!")
