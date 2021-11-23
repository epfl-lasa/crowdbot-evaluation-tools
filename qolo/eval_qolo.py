# -*-coding:utf-8 -*-
"""
@File    :   eval_qolo.py
@Time    :   2021/11/09
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import os
import argparse

import numpy as np

from crowdbot_data import AllFrames
from eval_util import save_motion_img, save_path_img


# TODO: check data source from pose2d (odom) or tf_qolo


#%% utility functions to evaluate qolo data
def compute_time_path(qolo_twist, qolo_pose2d):
    """calculate starting and ending timestamp and path"""

    # 1. calculate starting timestamp based on nonzero twist command
    # starting: larger than zero
    start_idx = np.min(
        [
            np.min(np.nonzero(qolo_twist.get("x"))),
            np.min(np.nonzero(qolo_twist.get("zrot"))),
        ]
    )
    start_ts = qolo_twist.get("timestamp")[start_idx]
    print("starting timestamp: {}".format(start_ts))

    # 2. calculate ending timestamp based on closest point to goal
    pose_ts = qolo_pose2d.get("timestamp")
    pose_x = qolo_pose2d.get("x")
    pose_y = qolo_pose2d.get("y")
    theta = qolo_pose2d.get("theta")
    ## borrow from evalMetricsPathAndTimeToGoal.py
    angle_init = np.sum(theta[9:19]) / 10.0
    goal = np.array([np.cos(angle_init), np.sin(angle_init)]) * 20.0

    # determine when the closest point to the goal is reached
    min_dist2goal = np.inf
    end_idx = -1
    for idx in range(len(pose_ts)):
        dist2goal = np.sqrt((pose_x[idx] - goal[0]) ** 2 + (pose_y[idx] - goal[1]) ** 2)
        if dist2goal < min_dist2goal:
            min_dist2goal = dist2goal
            end_idx = idx

    path_length2goal = np.sum(
        np.sqrt(np.diff(pose_x[:end_idx]) ** 2 + np.diff(pose_y[:end_idx]) ** 2)
    )
    end_ts = pose_ts[end_idx]
    duration2goal = end_ts - start_ts
    print("ending timestamp: {}".format(end_ts))
    print("Duration: {}s".format(duration2goal))
    ## borrow from evalMetricsPathAndTimeToGoal.py

    return (
        start_ts,
        end_ts,
        duration2goal,
        path_length2goal,
        end_idx,
        goal,
        min_dist2goal,
    )


# https://github.com/epfl-lasa/qolo-evaluation/blob/main/notebook/crowd_evaluation.py#L187
def compute_fluency(qolo_command):
    """compute consistency of the velocity command"""

    vel = qolo_command.get("x_vel")
    omega = qolo_command.get("zrot_vel")
    v_max, w_max = np.max(vel), np.max(omega)

    fluency_v = []
    fluency_w = []

    for idx in range(1, len(vel)):
        if vel[idx] or omega[idx]:
            fluency_v.append(1 - np.abs(vel[idx] - vel[idx - 1]) / v_max)
            fluency_w.append(1 - np.abs(omega[idx] - omega[idx - 1]) / w_max)

    fluencyv = np.mean(np.array(fluency_v))
    fluencyv_sd = np.std(np.array(fluency_v))
    fluencyw = np.mean(np.array(fluency_w))
    fluencyw_sd = np.std(np.array(fluency_w))

    if fluencyv < fluencyw:
        return (fluencyv, fluencyv_sd)
    else:
        return (fluencyw, fluencyw_sd)


# https://github.com/epfl-lasa/qolo-evaluation/blob/main/notebook/crowd_evaluation.py#L222
def compute_agreement(qolo_command, qolo_state):
    """compute real qolo velocity in reference to command"""
    vel_c = qolo_command.get("x_vel")
    omega_c = qolo_command.get("zrot_vel")
    vc_max, wc_max = np.max(vel_c), np.max(omega_c)
    # max_v, max_w
    vec_size = vel_c.shape[0]

    vel_r = qolo_state.get("x_vel")
    omega_r = qolo_state.get("zrot_vel")

    angle_U = np.arctan2(vel_c / vc_max, omega_c / wc_max)
    angle_R = np.arctan2(vel_r / vc_max, omega_r / wc_max)
    angle_diff = angle_R - angle_U

    ccount = 0
    omega_diff = []
    vel_diff = []
    agreement_vec = []
    for idx in range(2, vec_size):
        if vel_c[idx] or omega_c[idx]:
            vel_diff.append(np.abs(vel_r[idx] - vel_c[idx]) / vc_max)
            omega_diff.append(np.abs(omega_r[idx] - omega_c[idx]) / wc_max)
            agreement_vec.append(1 - (abs(angle_diff[idx]) / np.pi))
            ccount += 1  # TODO: check unused?
    command_diff = np.column_stack((np.array(vel_diff), np.array(omega_diff)))
    command_vec = np.linalg.norm(command_diff, axis=1)

    vel_diff = np.array(vel_diff)
    omega_diff = np.array(omega_diff)
    agreement_vec = np.array(agreement_vec)
    # TODO: check directional_agreement unused?
    directional_agreement = [np.mean(agreement_vec), np.std(agreement_vec)]

    linear_dis = [np.mean(vel_diff), np.std(vel_diff)]
    heading_dis = [np.mean(omega_diff), np.std(omega_diff)]

    disagreement = [np.mean(np.array(command_vec)), np.std(np.array(command_vec))]

    return (linear_dis, heading_dis, disagreement)  # Contribution


#%% main function
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
    parser.add_argument(
        "--save_img",
        dest="save_img",
        action="store_true",
        help="plot and save crowd density image",
    )
    parser.set_defaults(save_img=True)
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Whether to overwrite existing rosbags (default: false)",
    )
    parser.set_defaults(overwrite=False)
    parser.add_argument(
        "--replot",
        dest="replot",
        action="store_true",
        help="Whether to re-plot existing images (default: false)",
    )
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
        print(
            "({}/{}): {} with {} frames".format(
                seq_idx + 1, allf.nr_seqs(), seq, allf.nr_frames(seq_idx)
            )
        )

        # load pose2d
        pose2d_dir = os.path.join(allf.source_data_dir, "pose2d")
        qolo_pose2d_path = os.path.join(pose2d_dir, seq + "_pose2d.npy")
        if not os.path.exists(qolo_pose2d_path):
            print("ERROR: Please extract pose2d by using pose2d2npy.py")
        qolo_pose2d = np.load(qolo_pose2d_path, allow_pickle=True).item()

        # load twist, qolo_command
        twist_dir = os.path.join(allf.source_data_dir, "twist")
        qolo_twist_path = os.path.join(twist_dir, seq + "_twist_raw.npy")
        command_sampled_filepath = os.path.join(twist_dir, seq + "_qolo_command.npy")
        if (not os.path.exists(qolo_twist_path)) or (
            not os.path.exists(command_sampled_filepath)
        ):
            print("ERROR: Please extract twist_stamped by using twist2npy.py")
        qolo_twist = np.load(qolo_twist_path, allow_pickle=True).item()
        qolo_command_dict = np.load(command_sampled_filepath, allow_pickle=True).item()
        # print("qolo_command_dict.keys()", qolo_command_dict.keys())

        # load qolo_state
        tfqolo_dir = os.path.join(allf.source_data_dir, "tf_qolo")
        qolo_state_filepath = os.path.join(tfqolo_dir, seq + "_qolo_state.npy")
        qolo_lidarstamp_filepath = os.path.join(tfqolo_dir, seq + "_tfqolo_sampled.npy")
        if not os.path.exists(qolo_state_filepath):
            print("ERROR: Please extract twist_stamped by using tfqolo2npy.py")
        qolo_state_dict = np.load(qolo_state_filepath, allow_pickle=True).item()
        qolo_lidarstamp_dict = np.load(
            qolo_lidarstamp_filepath, allow_pickle=True
        ).item()
        # print("qolo_state_dict.keys()", qolo_state_dict.keys())

        # 00. compute (start_ts, end_idx, end_ts, duration2goal, path_length2goal)
        time_path_computed = compute_time_path(qolo_twist, qolo_pose2d)

        # dest: seq+'_crowd_eval.npy' file in eval_res_dir
        qolo_eval_npy = os.path.join(eval_res_dir, seq + "_qolo_eval.npy")

        # only for plotting function update!
        if args.replot:
            qolo_eval_npy = np.load(qolo_eval_npy, allow_pickle=True).item()

            # figure1: path
            save_path_img(qolo_pose2d, time_path_computed, eval_res_dir, seq)

            # figure2: viz twist, acc, jerk from qolo_command and qolo_state
            save_motion_img(
                qolo_command_dict,
                qolo_eval_dict,
                eval_res_dir,
                seq,
                suffix="_qolo_command",
            )
            save_motion_img(
                qolo_state_dict,
                qolo_eval_dict,
                eval_res_dir,
                seq,
                suffix="_qolo_state",
            )

            print("Replot images!")
        else:
            if (not os.path.exists(qolo_eval_npy)) or (args.overwrite):

                # timestamp can be read from lidars/ folder
                stamp_file_path = os.path.join(allf.lidar_dir, seq + "_stamped.npy")
                lidar_stamped_dict = np.load(stamp_file_path, allow_pickle=True)
                ts = lidar_stamped_dict.item().get("timestamp")

                attrs = ("jerk", "agreement", "fluency")

                # 1. jerk
                qolo_eval_dict = dict()
                qolo_eval_dict.update(
                    {"avg_x_jerk": np.average(qolo_command_dict["x_jerk"])}
                )
                qolo_eval_dict.update(
                    {"avg_zrot_jerk": np.average(qolo_command_dict["zrot_jerk"])}
                )

                # 2. fluency
                fluency = compute_fluency(qolo_command_dict)
                qolo_eval_dict.update({"avg_fluency": fluency[0]})
                qolo_eval_dict.update({"std_fluency": fluency[1]})

                # 3. agreement with command sampled (may need to extract command from rds msg)
                agreement = compute_agreement(qolo_command_dict, qolo_lidarstamp_dict)
                print("(linear_dis, heading_dis, disagreement) =", agreement)

                # 4. path related-metrics
                qolo_eval_dict.update({"start_command_ts": time_path_computed[0]})
                qolo_eval_dict.update({"end_command_ts": time_path_computed[1]})
                qolo_eval_dict.update({"duration2goal": time_path_computed[2]})
                qolo_eval_dict.update({"path_lenth2goal": time_path_computed[3]})

                np.save(qolo_eval_npy, qolo_eval_dict)

                if args.save_img:
                    # figure1: path
                    save_path_img(qolo_pose2d, time_path_computed, eval_res_dir, seq)

                    # figure2: viz twist, acc, jerk
                    save_motion_img(
                        qolo_command_dict,
                        qolo_eval_dict,
                        eval_res_dir,
                        seq,
                        suffix="_qolo_command",
                    )
                    save_motion_img(
                        qolo_state_dict,
                        qolo_eval_dict,
                        eval_res_dir,
                        seq,
                        suffix="_qolo_state",
                    )
            else:
                print(
                    "Detecting the generated {} already existed!".format(qolo_eval_npy)
                )
                print(
                    "Will not overwrite. If you want to overwrite, use flag --overwrite"
                )
                continue

    print("Finish qolo evaluation!")
