#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   tfqolo2npy.py
@Date created  :   2021/10/26
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides workflow to extract robot pose (`tf_qolo` reference to
`tf_qolo_world`) from rosbag, filter data with Savitzky-Golay method, and
compute velocity acceleration, jerk by taking derivatives.
"""
# =============================================================================
"""
TODO:
1. better filter to position, quaternions, velocities, etc.
"""
# =============================================================================

import os
import sys
import argparse
import numpy as np

import quaternion as Q
import quaternion.quaternion_time_series as qseries

import rosbag

from qolo.external.tf_bag import BagTfTransformer

from qolo.core.crowdbot_data import CrowdBotDatabase, bag_file_filter
from qolo.utils.process_util import (
    interp_rotation,
    interp_translation,
    compute_motion_derivative,
    smooth1d,
    smooth,
    strict_increase,
)

#%% Utility function for extraction tf from rosbag and apply interpolation
def extract_pose_from_rosbag(bag_file_path):
    """Esxtract pose_stamped from rosbag without rosbag play"""

    # load rosbag and BagTfTransformer
    bag = rosbag.Bag(bag_file_path)
    bag_transformer = BagTfTransformer(bag)

    # "t265_pose_frame" -> "tf_qolo"
    # "t265_odom_frame" -> "t265_pose_frame"
    # "t265_odom_frame" -> "tf_qolo_world"
    trans_iter = bag_transformer.lookupTransformWhenTransformUpdates(
        "tf_qolo_world",
        "tf_qolo",
        trigger_orig_frame="t265_odom_frame",
        trigger_dest_frame="t265_pose_frame",
    )
    t_list, p_list, o_list = [], [], []
    for timestamp, transformation in trans_iter:
        (position, orientation) = transformation
        # timestamp in genpy.Time type
        t_list.append(timestamp.to_sec())
        p_list.append(position)
        o_list.append(orientation)

    t_np = np.asarray(t_list, dtype=np.float64)
    p_np = np.asarray(p_list, dtype=np.float64)
    o_np = np.asarray(o_list, dtype=np.float64)

    pose_stamped_dict = {"timestamp": t_np, "position": p_np, "orientation": o_np}
    return pose_stamped_dict


# deduplicate tf qolo
def deduplicate_tf(qolo_tf):
    """Delete duplicate tf in recorded rosbag"""
    ts = qolo_tf.get("timestamp")
    tf_qolo_pos = qolo_tf.get("position")
    tf_qolo_ori = qolo_tf.get("orientation")
    print(
        "Raw input {} frames, about {:.1f} Hz".format(
            len(ts), len(ts) / (max(ts) - min(ts))
        )
    )

    tf_qolo_pos_ = np.vstack(([0.0, 0.0, 0.0], tf_qolo_pos))
    tf_qolo_pos_delta = np.diff(tf_qolo_pos_, axis=0)

    qolo_x = np.nonzero(tf_qolo_pos_delta[:, 0])
    qolo_y = np.nonzero(tf_qolo_pos_delta[:, 1])
    qolo_z = np.nonzero(tf_qolo_pos_delta[:, 2])
    tf_qolo_unique_idx = tuple(set(qolo_x[0]) & set(qolo_y[0]) & set(qolo_z[0]))

    new_idx = sorted(tf_qolo_unique_idx)

    ts_new = ts[list(new_idx)]
    tf_qolo_pos_new = tf_qolo_pos[new_idx, :]
    tf_qolo_ori_ori = tf_qolo_ori[new_idx, :]
    print(
        "Reduplicated Output {} frames, about {:.1f} Hz".format(
            len(ts_new), len(ts_new) / (max(ts_new) - min(ts_new))
        )
    )
    return {
        "timestamp": ts_new,
        "position": tf_qolo_pos_new,
        "orientation": tf_qolo_ori_ori,
    }


def interp_pose(source_dict, interp_ts):
    """Calculate interpolations for all states with scipy"""
    source_ts = source_dict.get("timestamp")
    source_pos = source_dict.get("position")
    source_ori = source_dict.get("orientation")

    # method1: saturate the timestamp outside the range
    if np.min(interp_ts) < np.min(source_ts):
        interp_ts[interp_ts < min(source_ts)] = min(source_ts)
    if np.max(interp_ts) > np.max(source_ts):
        interp_ts[interp_ts > max(source_ts)] = max(source_ts)

    # method2: discard timestamps smaller or bigger than source
    # start_idx, end_idx = 0, -1
    # if min(interp_ts) < min(source_ts):
    #     start_idx = np.argmax(interp_ts[interp_ts - source_ts.min() < 0]) + 1
    # if max(interp_ts) > max(source_ts):
    #     end_idx = np.argmax(interp_ts[interp_ts - source_ts.max() <= 0]) + 1
    # interp_ts = interp_ts[start_idx:end_idx]

    # print(interp_ts.min(), interp_ts.max(), source_ts.min(), source_ts.max())

    # Slerp -> interp_rotation -> ValueError: Times must be in strictly increasing order.
    interp_dict = {}
    interp_dict["timestamp"] = interp_ts
    interp_dict["orientation"] = interp_rotation(source_ts, interp_ts, source_ori)
    interp_dict["position"] = interp_translation(source_ts, interp_ts, source_pos)
    return interp_dict, interp_ts


# calculate velocity
def quat_mul(quat0, quat1):
    x0, y0, z0, w0 = quat0
    x1, y1, z1, w1 = quat1
    return np.array(
        [
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        ],
        dtype=np.float64,
    )


def quat_norm(quat_):
    quat_sum = np.linalg.norm(quat_)
    for i, val in enumerate(quat_):
        quat_[i] = val / quat_sum
    return quat_


def quat_conjugate(quat_):
    x0, y0, z0, w0 = quat_
    return np.array(
        [-x0, -y0, -z0, w0],
        dtype=np.float64,
    )


def qv_mult(quat_, vec_):
    # vec -> list; quat -> list
    quat_ = quat_norm(quat_)
    temp = quat_mul(quat_, vec_)
    res_vec = quat_mul(temp, quat_conjugate(quat_))
    return res_vec[:3]


#%% main file
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-f",
        "--folder",
        default="0325_rds",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--hz", default=200.0, type=float, help="desired interpolated high frequency"
    )
    parser.add_argument(
        "--smooth",
        dest="smooth",
        action="store_true",
        help="Filter datapoints with Savitzky-Golay or moving-average filter (default: false)",
    )
    parser.set_defaults(smooth=True)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    cb_data = CrowdBotDatabase(args.folder)

    # source: rosbag data in data/rosbag/xxxx
    # base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
    # rosbag_dir = os.path.join(base_dir, "data/rosbag", args.folder)
    rosbag_dir = os.path.join(cb_data.bagbase_dir, args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))
    if not os.path.exists(cb_data.lidar_dir):
        print(
            "ERROR: please use `gen_lidar_from_rosbags.py` to extract lidar files first!"
        )

    # destination: pose data in data/xxxx_processed/source_data/tfqolo
    tfqolo_dir = os.path.join(cb_data.source_data_dir, "tf_qolo")
    if not os.path.exists(tfqolo_dir):
        os.makedirs(tfqolo_dir)

    print(
        "Starting extracting pose_stamped files from {} rosbags!".format(len(bag_files))
    )

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        seq = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        # sample with lidar frame
        all_stamped_filepath = os.path.join(tfqolo_dir, seq + "_tfqolo_raw.npy")
        lidar_stamped_filepath = os.path.join(tfqolo_dir, seq + "_tfqolo_sampled.npy")

        # sample at high frequency (200Hz)
        state_filepath = os.path.join(tfqolo_dir, seq + "_qolo_state.npy")

        if (
            (not os.path.exists(lidar_stamped_filepath))
            or (not os.path.exists(state_filepath))
            or (args.overwrite)
        ):
            if (not os.path.exists(all_stamped_filepath)) or (args.overwrite):
                pose_stamped_dict = extract_pose_from_rosbag(bag_path)
                pose_stamped_dict_ = deduplicate_tf(pose_stamped_dict)
                np.save(all_stamped_filepath, pose_stamped_dict_)
            else:
                print(
                    "Detecting the generated {} already existed!".format(
                        all_stamped_filepath
                    )
                )
                print("If you want to overwrite, use flag --overwrite")
                pose_stamped_dict_ = np.load(
                    all_stamped_filepath, allow_pickle=True
                ).item()

            # _qolo_state.npy
            init_ts = pose_stamped_dict_.get("timestamp")
            start_ts, end_ts = init_ts.min(), init_ts.max()
            interp_dt = 1 / args.hz
            high_interp_ts = np.arange(
                start=start_ts, step=interp_dt, stop=end_ts, dtype=np.float64
            )
            # position & orientation
            state_dict, high_interp_ts = interp_pose(pose_stamped_dict_, high_interp_ts)
            print(
                "Interpolated output {} frames, about {:.1f} Hz".format(
                    len(high_interp_ts),
                    len(high_interp_ts) / (max(high_interp_ts) - min(high_interp_ts)),
                )
            )

            # tfqolo {xyz, quat, ts} -> {x, y, z, roll, pitch, yaw, ts}
            # vel {x_vel, y_vel, z_vel, xrot_vel, yrot_vel, zrot_vel, ts}
            from scipy.spatial.transform import Rotation as R

            quat_xyzw = state_dict["orientation"]
            state_pose_r = R.from_quat(quat_xyzw)

            # rotate to local frame
            # TODO: check if needing reduce compared to first frame?
            state_r_aligned = state_pose_r.reduce(
                left=R.from_quat(state_dict["orientation"][0, :]).inv()
            )
            # rot_mat_list_aligned (frame, 3, 3) robot -> world
            r2w_rot_mat_aligned_list = state_r_aligned.as_matrix()
            # world -> robot
            w2r_rot_mat_aligned_list = state_r_aligned.inv().as_matrix()

            print("Computing linear velocity!")
            position_g = state_dict["position"]

            # smooth with Savitzky-Golay filter
            print("Using Savitzky-Golay filter to smooth position!")
            if args.smooth:
                smoothed_position_g = smooth(
                    position_g,
                    filter='savgol',
                    window=41,
                    polyorder=3,
                )
                position_g = smoothed_position_g

            state_pose_g = {
                "x": position_g[:, 0],
                "y": position_g[:, 1],
                "z": position_g[:, 2],
                "timestamp": high_interp_ts,
            }
            state_vel_g = compute_motion_derivative(
                state_pose_g, subset=["x", "y", "z"]
            )

            # xyz_vel_g = np.hstack(
            #     (state_vel_g["x"], state_vel_g["y"], state_vel_g["z"])
            # )
            # xyz_vel_g = np.reshape(xyz_vel_g, (-1, 3, 1))
            # xyz_vel = np.matmul(w2r_rot_mat_aligned_list, xyz_vel_g)  # (frames, 3, 1)
            # xyz_vel = np.reshape(xyz_vel, (-1, 3))

            # https://math.stackexchange.com/a/2030281
            # https://answers.ros.org/question/196149/how-to-rotate-vector-by-quaternion-in-python/
            xyz_vel_g = np.vstack(
                (state_vel_g["x"], state_vel_g["y"], state_vel_g["z"])
            ).T
            xyz_vel = np.zeros_like(xyz_vel_g)
            for idx in range(xyz_vel_g.shape[0]):
                vel = xyz_vel_g[idx, :]
                quat = quat_xyzw[idx, :]
                vel_ = np.zeros(4, dtype=np.float64)
                vel_[:3] = vel
                xyz_vel[idx, :] = qv_mult(quat, vel_)
            print("Using new conversion")

            print("Computing angular velocity velocity!")

            # calculate difference
            high_interp_ts_delta = np.diff(high_interp_ts)

            # find the index with zero variation
            result = np.where(high_interp_ts_delta == 0)[0]

            if len(result) > 0:
                print("non-increasing elements number:", len(result))
                nonzero_idx = np.nonzero(high_interp_ts_delta != 0)
                start_zidx = np.min(nonzero_idx)
                end_zidx = np.max(nonzero_idx) + 1
                # print(start_zidx, end_zidx, len(np.arange(start_zidx, end_zidx)))

                # extract the non-increasing point
                new_high_interp_ts = high_interp_ts[start_zidx : end_zidx + 1]
                new_quat_xyzw = quat_xyzw[start_zidx : end_zidx + 1, :]

                # print(new_high_interp_ts.shape, new_quat_xyzw.shape)

                quat_wxyz = new_quat_xyzw[:, [3, 0, 1, 2]]
                quat_wxyz_ = Q.as_quat_array(quat_wxyz)
                # ValueError: `x` must be strictly increasing sequence.
                # dx = np.diff(x)
                #     if np.any(dx <= 0):
                #     raise ValueError("`x` must be strictly increasing sequence.")
                ang_vel = qseries.angular_velocity(quat_wxyz_, new_high_interp_ts)
                # print(ang_vel.shape)

                if start_zidx > 0:
                    before = np.zeros((3, start_zidx), dtype=ang_vel.dtype)
                    ang_vel = np.concatenate((before, ang_vel), axis=0)
                    # print(ang_vel.shape)
                after = len(result) - start_zidx
                if after > 0:
                    ang_vel = np.pad(ang_vel, ((0, after), (0, 0)), 'edge')
                # print(ang_vel.shape)
            else:
                # Fixed ValueError: `x` must be strictly increasing sequence.
                # dx = np.diff(x)
                #     if np.any(dx <= 0):
                #     raise ValueError("`x` must be strictly increasing sequence.")
                quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]
                quat_wxyz_ = Q.as_quat_array(quat_wxyz)
                ang_vel = qseries.angular_velocity(quat_wxyz_, high_interp_ts)

            assert ang_vel.shape[0] == quat_xyzw.shape[0]
            ## ensure the use of CubicSpline inside `angular_velocity``

            state_vel = {
                "timestamp": high_interp_ts,
                "x": xyz_vel[:, 0],
                "zrot": ang_vel[:, 2],
            }
            if args.smooth:
                print("Using Savitzky-Golay filter to smooth vel!")
                # 211116: unfiltered data
                # state_dict.update({"x_vel_uf": xyz_vel[:, 0]})
                # state_dict.update({"zrot_vel_uf": ang_vel[:, 2]})
                # 211116: apply filter to computed vel
                smoothed_x_vel = smooth1d(
                    xyz_vel[:, 0],
                    filter='savgol',
                    window=41,
                    polyorder=5,
                    check_thres=True,
                )
                smoothed_zrot_vel = smooth1d(
                    ang_vel[:, 2],
                    filter='savgol',
                    window=41,
                    polyorder=5,
                    check_thres=True,
                    thres=[-4.124, 4.124],
                )

                # update filtered velocity
                xyz_vel[:, 0] = smoothed_x_vel
                ang_vel[:, 2] = smoothed_zrot_vel

                state_dict.update({"x_vel": smoothed_x_vel})
                state_dict.update({"zrot_vel": smoothed_zrot_vel})
            else:
                state_dict.update({"x_vel": xyz_vel[:, 0]})
                state_dict.update({"zrot_vel": ang_vel[:, 2]})

            # acc
            print("Computing acc!")
            state_acc = compute_motion_derivative(state_vel)
            if args.smooth:
                print("Using Savitzky-Golay filter to smooth acc!")
                # 211116: unfiltered data
                # state_dict.update({"x_acc_uf": state_acc["x"]})
                # state_dict.update({"zrot_acc_uf": state_acc["zrot"]})
                # 211116: apply filter to computed acc
                smoothed_x_acc = smooth1d(
                    state_acc['x'],
                    filter='savgol',
                    window=41,
                    polyorder=5,
                    check_thres=True,
                    thres=[-1.5, 1.5],  # limit value
                )
                smoothed_zrot_acc = smooth1d(
                    state_acc['zrot'],
                    filter='savgol',
                    window=41,
                    polyorder=5,
                    check_thres=True,
                    thres=[-4.5, 4.5],  # limit value
                )

                # update filtered acceleration
                state_acc['x'] = smoothed_x_acc
                state_acc['zrot'] = smoothed_zrot_acc

                state_dict.update({"x_acc": smoothed_x_acc})
                state_dict.update({"zrot_acc": smoothed_zrot_acc})
            else:
                state_dict.update({"x_acc": smoothed_x_acc})
                state_dict.update({"zrot_acc": smoothed_zrot_acc})

            # jerk
            print("Computing jerk!")
            state_jerk = compute_motion_derivative(state_acc)
            if args.smooth:
                print("Using Savitzky-Golay filter to smooth jerk!")
                smoothed_x_jerk = smooth1d(
                    state_jerk['x'],
                    filter='savgol',
                    window=41,
                    polyorder=5,
                    check_thres=True,
                    thres=[-20, 40],  # limit value
                )
                smoothed_zrot_jerk = smooth1d(
                    state_jerk['zrot'],
                    filter='savgol',
                    window=41,
                    polyorder=5,
                    check_thres=True,
                    thres=[-40, 40],  # limit value
                )

                # update filtered acceleration
                state_jerk['x'] = smoothed_x_jerk
                state_jerk['zrot'] = smoothed_zrot_jerk

            state_dict.update({"x_jerk": state_jerk["x"]})
            state_dict.update({"zrot_jerk": state_jerk["zrot"]})
            state_dict.update({"avg_x_jerk": np.average(state_jerk["x"])})
            state_dict.update({"avg_zrot_jerk": np.average(state_jerk["zrot"])})
            np.save(state_filepath, state_dict)

            # _stamped.npy
            lidar_stamp_dir = os.path.join(cb_data.source_data_dir, "timestamp")
            stamp_file_path = os.path.join(lidar_stamp_dir, seq + "_stamped.npy")
            lidar_stamped = np.load(
                stamp_file_path,
                allow_pickle=True,
            ).item()
            lidar_ts = lidar_stamped.get("timestamp")
            lidar_stamped_dict, lidar_ts = interp_pose(pose_stamped_dict_, lidar_ts)

            # interpolate velocity
            if min(lidar_ts) < min(high_interp_ts):
                lidar_ts[lidar_ts < min(high_interp_ts)] = min(high_interp_ts)
            if max(lidar_ts) > max(high_interp_ts):
                lidar_ts[lidar_ts > max(high_interp_ts)] = max(high_interp_ts)
            x_vel_sampled = interp_translation(
                high_interp_ts, lidar_ts, state_dict["x_vel"]
            )
            zrot_vel_sampled = interp_translation(
                high_interp_ts, lidar_ts, state_dict["zrot_vel"]
            )
            lidar_stamped_dict.update({"x_vel": x_vel_sampled})
            lidar_stamped_dict.update({"zrot_vel": zrot_vel_sampled})
            np.save(lidar_stamped_filepath, lidar_stamped_dict)

    print("Finish extracting all twist and compute state msg!")
