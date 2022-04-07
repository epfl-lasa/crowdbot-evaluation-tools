#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   eval_traj.py
@Date created  :   2022/02/22
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides pipeline to evaluate pededtrian performance
python qolo/eval_traj.py -f 0410_mds --all --overwrite
"""
# =============================================================================

import os
import sys
import copy
import argparse
import numpy as np
import pandas as pd
from timeit import default_timer as timer

from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
import quaternion as Q
import quaternion.quaternion_time_series as qseries

from qolo.core.crowdbot_data import CrowdBotDatabase
from qolo.utils.file_io_util import (
    save_dict2pkl,
    save_dict2json,
    load_json2dict,
    load_pkl2dict,
)
# TODO: Exception has occurred: ModuleNotFoundError No module named
curr_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(curr_dir_path, 'external/trajectory_smoothing/'))
# advanced smoothing algorithm
from smooth_traj import smooth_traj


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

def traj_filtering(x_raw, y_raw,z_raw, cutoff=2, print_time=False):

    if print_time:
        start = timer()
    order = 6
    fs = 20.0       # sample rate, Hz (pedestrian is about 20 Hz)
	# desired cutoff frequency of the filter, Hz
    # points = []
    x_filtered = butter_lowpass_filter(x_raw, cutoff, fs, order)
    y_filtered = butter_lowpass_filter(y_raw, cutoff, fs, order)
    z_filtered = butter_lowpass_filter(z_raw, cutoff, fs, order)

    if print_time:
        end = timer()
        time_elapsed = end - start
        print("Time Elapsed for Filtering: %.4f seconds." %time_elapsed)

    return x_filtered, y_filtered, z_filtered

def traj_viz_debug(xline, yline, zline, prefix='original'):
    import matplotlib.pyplot as plt
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    # zline = np.linspace(0, 15, 1000)
    # xline = np.sin(zline)
    # yline = np.cos(zline)
    ax.plot3D(xline, yline, zline, 'gray')

    # Data for three-dimensional scattered points
    # zdata = 15 * np.random.random(100)
    # xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
    # ydata = np.cos(zdata) + 0.1 * np.random.randn(100)
    path_color = [np.sqrt(
        (xline[i]-xline[0])**2 + (yline[i]-yline[0])**2
        ) for i in range(len(xline))]
    ax.scatter3D(xline, yline, zline, c=path_color, cmap='Greens');
    # plt.show()
    ax.view_init(80, 35)
    plt.savefig(prefix+"_traj.png")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="evaluate trajectories of pedestrians around Qolo"
    )

    parser.add_argument(
        "-f",
        "--folder",
        default="0410_rds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--seq",
        default="2021-04-10-10-38-36",  # 2021-04-10-10-38-36  2021-04-10-10-41-17
        type=str,
        help="specific sequence in the subfolder",
    )
    parser.add_argument(
        "--all",
        dest="process_all",
        action="store_true",
        help="Process all sequences and disable single sequences",
    )
    parser.set_defaults(process_all=False)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing output (default: false)",
    )
    parser.set_defaults(overwrite=True)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    if args.seq is None or args.process_all:
        seqs = [cb_data.seqs[seq_idx] for seq_idx in range(cb_data.nr_seqs())]
    else:
        seqs = [args.seq]

    for seq_idx, seq in enumerate(seqs):

        sq_idx = cb_data.seqs.index(seq)
        seq_len = cb_data.nr_frames(sq_idx)

        print("({}/{}): {} with {} frames".format(seq_idx + 1, len(seqs), seq, seq_len))

        # trajectory data
        traj_dir = os.path.join(cb_data.ped_data_dir, "traj")
        if not os.path.exists(traj_dir):
            sys.exit("Please use det2traj.py to extract pedestrian trajectories first!")
        traj_pkl_path = os.path.join(traj_dir, seq + '.pkl')
        traj_json_path = os.path.join(traj_dir, seq + '.json')
        proc_traj_pkl_path = os.path.join(traj_dir, seq + '_proc.pkl')

        traj_dict = load_pkl2dict(traj_pkl_path)
        proc_traj_dict = copy.deepcopy(traj_dict)
        # key: id
        # dict -> key: start_idx, rel_pose_list, abs_pose_list, length, end_idx
        traj_data = pd.DataFrame.from_dict(
            traj_dict, orient='index', columns=['start_idx', 'end_idx', 'length']
        )

        traj_data_200frs = traj_data[traj_data.length >= 200]

        print("Largest tracking length:", max(traj_data.length))
        print("Pedestrians tracked >=200 frames:", len(traj_data_200frs))

        # lidar timestamp
        stamp_file_path = os.path.join(
            cb_data.source_data_dir, "timestamp", seq + "_stamped.npy"
        )
        frame_ts = (
            np.load(
                stamp_file_path,
                allow_pickle=True,
            )
            .item()
            .get("timestamp")
        )

        # destination
        vel_dir = os.path.join(cb_data.ped_data_dir, "vel")
        if not os.path.exists(vel_dir):
            os.makedirs(vel_dir)
        vel_pkl_path = os.path.join(vel_dir, seq + '.pkl')
        vel_json_path = os.path.join(vel_dir, seq + '.json')

        vel_files_exist = os.path.exists(vel_pkl_path) and os.path.exists(vel_json_path)
        if vel_files_exist and not args.overwrite:
            print("{} velocities already generated!!!".format(seq))
            print("Will not overwrite. If you want to overwrite, use flag --overwrite")
            continue

        # generate velocity from pose and time interval
        ids = traj_data_200frs.index.values
        peds_vel_dict = dict()
        for idx, id in enumerate(ids):
            # dval_dt = np.gradient(motion_stamped_dict.get(val), frame_ts, axis=0)
            start_idx = traj_dict[id]['start_idx']
            end_idx = traj_dict[id]['end_idx']

            # position (lvie_frames, 3)
            xyz = np.array(traj_dict[id]['abs_pose_list'])
            xx = xyz[:,0]
            yy = xyz[:,1]
            zz = xyz[:,2]
            tt = frame_ts[start_idx : end_idx + 1]
            # traj_viz_debug(xx, yy, zz)

            # 0.0027 seconds for ~450 frames
            fx, fy, fz = traj_filtering(xx,yy,zz,cutoff=2)
            # traj_viz_debug(fx, fy, fz, prefix='filtered')
            filtered_xyz = np.vstack((fx, fy, fz)).T
            proc_traj_dict[id]['abs_pose_list'] = filtered_xyz.tolist()

            # case 0-Bezier curves: tool two long time!
            # case 1-Spline-curves: 31.1879 seconds for ~450 frames
            # case 2-cannot config correctly
            # case 3/4/5-cannot smooth correctly!
            # TODO: need to accelerate Spline smoothing speed!!!
            # sx, sy, sz = smooth_traj(xx,yy,zz,tt,case=3)
            # traj_viz_debug(sx, sy, sz, prefix='smoothed')

            # the first 2 column in 'lin_vel' are more important
            lin_vel = np.gradient(xyz, frame_ts[start_idx : end_idx + 1], axis=0)
            # lin_vel = np.gradient(filtered_xyz, frame_ts[start_idx : end_idx + 1], axis=0)

            # method1: calculate from quat (has large errors!)
            # quat_xyzw = np.array(traj_dict[id]['abs_quat_list'])
            # quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]
            # quat_wxyz_ = Q.as_quat_array(quat_wxyz)
            # ang_vel = qseries.angular_velocity(
            #     quat_wxyz_, frame_ts[start_idx : end_idx + 1]
            # )

            # method2: calculate from dy/dx
            # vel_theta = np.tan(lin_vel[:,1]/lin_vel[:,0])

            ped_vel_dict = {
                "start_idx": start_idx,
                "end_idx": start_idx,
                "length": traj_dict[id]['length'],
                "lin_vel": lin_vel.tolist(),
                # "ang_vel": ang_vel.tolist(),
            }
            peds_vel_dict.update({int(id): ped_vel_dict})  # int64 -> int

        save_dict2pkl(peds_vel_dict, vel_pkl_path)
        save_dict2json(peds_vel_dict, vel_json_path)

        save_dict2pkl(proc_traj_dict, proc_traj_pkl_path)
