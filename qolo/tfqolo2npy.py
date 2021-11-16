# -*-coding:utf-8 -*-
"""
@File    :   tfqolo2npy.py
@Time    :   2021/10/26
@Author  :   Yujie He
@Version :   1.2
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import os
import sys
import argparse
import numpy as np

import rosbag

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "external"))
from tf_bag import BagTfTransformer

from crowdbot_data import AllFrames, bag_file_filter
from process_util import interp_rotation, interp_translation, compute_motion_derivative

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
        "Input {} frames, about {:.1f} Hz".format(
            len(ts), len(ts) / (max(ts) - min(ts))
        )
    )

    tf_qolo_pos_ = np.vstack(([0.0, 0.0, 0.0], tf_qolo_pos))
    tf_qolo_pos_delta = np.diff(tf_qolo_pos_, axis=0)

    qolo_x = np.nonzero(tf_qolo_pos_delta[:, 0])
    qolo_y = np.nonzero(tf_qolo_pos_delta[:, 1])
    qolo_z = np.nonzero(tf_qolo_pos_delta[:, 2])
    tf_qolo_unique_idx = tuple(set(qolo_x[0]) & set(qolo_y[0]) & set(qolo_z[0]))

    ts_new = ts[list(tf_qolo_unique_idx)]
    tf_qolo_pos_new = tf_qolo_pos[tf_qolo_unique_idx, :]
    tf_qolo_ori_ori = tf_qolo_ori[tf_qolo_unique_idx, :]
    print(
        "Output {} frames, about {:.1f} Hz".format(
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

    # don't resize, just discard timestamps smaller or bigger than source
    if min(interp_ts) < min(source_ts):
        interp_ts[interp_ts < min(source_ts)] = min(source_ts)
    elif max(interp_ts) > max(source_ts):
        interp_ts[interp_ts > max(source_ts)] = max(source_ts)

    interp_dict = {}
    interp_dict["timestamp"] = interp_ts
    interp_dict["orientation"] = interp_rotation(source_ts, interp_ts, source_ori)
    interp_dict["position"] = interp_translation(source_ts, interp_ts, source_pos)
    return interp_dict


#%% Utility function for computing angular velocity by differentiating
def find_round_angle(angle, degrees=False):
    """Compute complementary angle that add up to 2pi/360 degrees"""
    # TODO: consider using numpy.unwrap()
    if degrees:
        res = 360 - angle
    else:
        res = 2 * np.pi - angle
    return res


def to_euler_zyx(rot_quat, degrees=False):
    """Convert quaternions to euler angles"""
    scipy_rot = R.from_quat(rot_quat)
    rot_zyx = scipy_rot.as_euler("zyx", degrees)
    return rot_zyx


def compute_ang_vel(rpy_list, hz=200.0):
    """Compute estimated angular velocity from list of orientations"""
    dt = 1 / hz
    rpy_list_ = np.vstack((rpy_list, rpy_list[-1, :]))
    ang_vel = np.zeros_like(rpy_list)

    for i in range(np.shape(rpy_list_)[1]):

        rpy_dt = np.diff(rpy_list_[:, i], axis=0)
        latter = np.concatenate(([0.0], rpy_dt))
        former = np.concatenate((rpy_dt, [0.0]))
        rpy_d2t = (former[:-1] + latter[:-1]) / 2
        ang_vel_poss1 = rpy_d2t / (2 * dt)

        forward_sign = np.sign(rpy_d2t)
        backward_sign = -forward_sign
        round_rpy_d2t = find_round_angle(abs(rpy_d2t))
        ang_vel_poss2 = np.multiply(backward_sign, round_rpy_d2t) / (2 * dt)

        for j in range(len(ang_vel_poss2)):
            if abs(ang_vel_poss1[j]) > abs(ang_vel_poss2[j]):
                ang_vel[j, i] = ang_vel_poss2[j]
            else:
                ang_vel[j, i] = ang_vel_poss1[j]
    return ang_vel


#%% main file
if __name__ == "__main__":
    base_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

    parser = argparse.ArgumentParser(description="convert data from rosbag")

    parser.add_argument(
        "-b",
        "--base",
        default=base_folder,
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
        default="shared_test",
        type=str,
        help="different subfolder in rosbag/ dir",
    )
    parser.add_argument(
        "--hz", default=200.0, type=float, help="desired interpolated high frequency"
    )
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    args = parser.parse_args()

    allf = AllFrames(args)

    # source: rosbag data in data/rosbag/xxxx
    rosbag_dir = os.path.join(args.base, args.data, "rosbag", args.folder)
    bag_files = list(filter(bag_file_filter, os.listdir(rosbag_dir)))
    if not os.path.exists(allf.lidar_dir):
        print(
            "ERROR: please use `gen_lidar_from_rosbags.py` to extract lidar files first!"
        )

    # destination: pose data in data/xxxx_processed/source_data/tfqolo
    tf_qolo_dir = os.path.join(allf.source_data_dir, "tf_qolo")
    if not os.path.exists(tf_qolo_dir):
        os.makedirs(tf_qolo_dir)

    print(
        "Starting extracting pose_stamped files from {} rosbags!".format(len(bag_files))
    )

    counter = 0
    for bf in bag_files:
        bag_path = os.path.join(rosbag_dir, bf)
        bag_name = bf.split(".")[0]
        counter += 1
        print("({}/{}): {}".format(counter, len(bag_files), bag_path))

        all_stamped_filepath = os.path.join(tf_qolo_dir, bag_name + "_tfqolo_raw.npy")
        # sample with lidar frame
        lidar_stamped_filepath = os.path.join(
            tf_qolo_dir, bag_name + "_tfqolo_sampled.npy"
        )
        # sample at high frequency (200Hz)
        state_filepath = os.path.join(tf_qolo_dir, bag_name + "_qolo_state.npy")

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

            # _tfqolo_sampled.npy
            lidar_stamped = np.load(
                os.path.join(allf.lidar_dir, bag_name + "_stamped.npy"),
                allow_pickle=True,
            ).item()
            lidar_interp_ts = lidar_stamped.get("timestamp")
            lidar_pose_dict = interp_pose(pose_stamped_dict_, lidar_interp_ts)
            np.save(lidar_stamped_filepath, lidar_pose_dict)

            # _qolo_state.npy
            init_ts = pose_stamped_dict_.get("timestamp")
            start_ts, end_ts = init_ts.min(), init_ts.max()
            interp_dt = 1 / args.hz
            high_interp_ts = np.arange(
                start=start_ts, step=interp_dt, stop=end_ts, dtype=np.float64
            )
            # position & orientation
            state_dict = interp_pose(pose_stamped_dict_, high_interp_ts)
            print(
                "Interpolated output {} frames, about {:.1f} Hz".format(
                    len(high_interp_ts),
                    len(high_interp_ts) / (max(high_interp_ts) - min(high_interp_ts)),
                )
            )

            # tfqolo {xyz, quat, ts} -> {x, y, z, roll, pitch, yaw, ts}
            # vel {x_vel, y_vel, z_vel, xrot_vel, yrot_vel, zrot_vel, ts}
            from scipy.spatial.transform import Rotation as R

            state_pose_r = R.from_quat(state_dict["orientation"])
            ori_zyx = state_pose_r.as_euler("zyx", degrees=True)
            roll_g = ori_zyx[:, 2]  # X
            pitch_g = ori_zyx[:, 1]  # Y
            yaw_g = ori_zyx[:, 0]  # Z

            state_pose_g = {
                "x": state_dict["position"][:, 0],
                "y": state_dict["position"][:, 1],
                "z": state_dict["position"][:, 2],
                "roll": roll_g,
                "pitch": pitch_g,
                "yaw": yaw_g,
                "timestamp": state_dict["timestamp"],
            }
            # vel
            print("Computing vel!")
            state_vel_g = compute_motion_derivative(
                state_pose_g, subset=["x", "y", "z"]
            )
            # ang_vel_list = compute_ang_vel(np.hstack((roll_g, pitch_g, yaw_g)))
            ang_vel_g = compute_ang_vel(ori_zyx[:, [2, 1, 0]])
            state_vel_g.update({"xrot": ang_vel_g[:, 0]})
            state_vel_g.update({"yrot": ang_vel_g[:, 1]})
            state_vel_g.update({"zrot": ang_vel_g[:, 2]})

            # rotate to local frame
            state_r_aligned = state_pose_r.reduce(
                left=R.from_quat(state_dict["orientation"][0, :]).inv()
            )
            # rot_mat_list_aligned (frame, 3, 3) robot -> world
            r2w_rot_mat_aligned_list = state_r_aligned.as_matrix()
            # world -> robot
            w2r_rot_mat_aligned_list = state_r_aligned.inv().as_matrix()
            # state_vel
            xyz_vel_g = np.hstack(
                (state_vel_g["x"], state_vel_g["y"], state_vel_g["z"])
            )
            xyz_vel_g = np.reshape(xyz_vel_g, (-1, 3, 1))
            ang_vel_g = np.reshape(ang_vel_g, (-1, 3, 1))

            # matrix multiplication in high-dim (19635, 3, 3) * (19635, 3, 1)
            # ref: https://stackoverflow.com/questions/23576973/python-numpy-matrix-multiplication-in-high-dimension
            # ref: https://numpy.org/doc/stable/reference/generated/numpy.matmul.html#numpy.matmul
            xyz_vel = np.matmul(w2r_rot_mat_aligned_list, xyz_vel_g)  # (frames, 3, 1)
            xyz_vel = np.reshape(xyz_vel, (-1, 3))
            ang_vel = np.matmul(w2r_rot_mat_aligned_list, ang_vel_g)  # (frames, 3, 1)
            ang_vel = np.reshape(ang_vel, (-1, 3))

            state_vel = {
                "timestamp": state_dict["timestamp"],
                "x": xyz_vel[:, 0],
                "zrot": ang_vel[:, 2],
            }
            state_dict.update({"x_vel": xyz_vel[:, 0]})
            state_dict.update({"zrot_vel": ang_vel[:, 2]})

            # acc
            print("Computing acc!")
            state_acc = compute_motion_derivative(state_vel)
            state_dict.update({"x_acc": state_acc["x"]})
            state_dict.update({"zrot_acc": state_acc["zrot"]})

            # jerk
            print("Computing jerk!")
            state_jerk = compute_motion_derivative(state_acc)
            state_dict.update({"x_jerk": state_jerk["x"]})
            state_dict.update({"zrot_jerk": state_jerk["zrot"]})
            state_dict.update({"avg_x_jerk": np.average(state_jerk["x"])})
            state_dict.update({"avg_zrot_jerk": np.average(state_jerk["zrot"])})

            np.save(state_filepath, state_dict)

    print("Finish extracting all twist and compute state msg")
