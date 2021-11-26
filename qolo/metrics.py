# -*-coding:utf-8 -*-
"""
@File    :   metrics.py
@Time    :   2021/11/23 11:13:35
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

# TODO: use qolo_twist and qolo_pose2d sample at the same timestamps

import numpy as np


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
    # print("starting timestamp: {}".format(start_ts))

    # 2. calculate ending timestamp based on closest point to goal
    pose_ts = qolo_pose2d.get("timestamp")
    pose_x = qolo_pose2d.get("x")
    pose_y = qolo_pose2d.get("y")
    theta = qolo_pose2d.get("theta")

    angle_init = np.sum(theta[9:19]) / 10.0
    goal_loc = np.array([np.cos(angle_init), np.sin(angle_init)]) * 20.0

    # 3. determine when the closest point to the goal
    goal_reached = False
    min_dist2goal = np.inf
    end_idx = -1
    for idx in range(len(pose_ts)):
        # np.sqrt((pose_x[idx] - goal_loc[0]) ** 2 + (pose_y[idx] - goal_loc[1]) ** 2)
        dist2goal = np.linalg.norm([pose_x[idx], pose_y[idx]] - goal_loc)
        if dist2goal < min_dist2goal:
            min_dist2goal = dist2goal
            end_idx = idx

    # 4. determine goal_reached
    if min_dist2goal <= 3.0:
        goal_reached = True

    # 5. path_length2goal & duration2goal
    path_length2goal = np.sum(
        np.sqrt(np.diff(pose_x[:end_idx]) ** 2 + np.diff(pose_y[:end_idx]) ** 2)
    )
    end_ts = pose_ts[end_idx]
    duration2goal = end_ts - start_ts
    # print("Ending timestamp: {}".format(end_ts))
    # print("Duration: {}s".format(duration2goal))

    # 6. relative path_length2goal & duration2goal
    """
    Relative time to the goal:
        For our experiments, we took the desired velocity (Vel_command_max) of the robot
        and the linear distance to (L_goal = ||pose_final - pose_init||).
        So that, we can estimate a relative time to the goal:  Trobot=Lgoal/vel
    Relative path length:
        Use the previous L_goal
        Lr=L_goal/L_crowd
        where L_crowd is the distance traveled by the robot
    """
    # TODO: use qolo_twist and qolo_pose2d sample at the same timestamps
    vel_command_all = qolo_twist.get("x")
    twist_ts_all = qolo_twist.get("timestamp")
    start_vel_idx = start_idx
    end_vel_idx = (np.abs(twist_ts_all - pose_ts[end_idx])).argmin()
    # print("Debug:", start_vel_idx, end_vel_idx, len(twist_ts_all))
    # only consider Vel_command_max when travelling towards goal
    vc_max = np.max(vel_command_all[start_vel_idx : end_vel_idx + 1])

    start_pose_idx = (np.abs(pose_ts - start_ts)).argmin()
    end_pose_idx = end_idx
    pose_init = pose_x[start_pose_idx]
    pose_final = pose_x[end_pose_idx]
    l_goal = np.linalg.norm(pose_init - pose_final)

    rel_duration2goal = l_goal / vc_max
    rel_path_length2goal = l_goal / path_length2goal

    return (
        start_ts,
        end_ts,
        duration2goal,
        path_length2goal,
        end_idx,
        goal_reached,
        min_dist2goal,
        goal_loc,
        rel_duration2goal,
        rel_path_length2goal,
    )


def compute_relative_jerk(x_jerk, zrot_jerk, cmd_ts, start_cmd_ts, end_cmd_ts):

    start_idx, end_idx = 0, len(cmd_ts) - 1
    # find start and ending points
    if cmd_ts.min() < start_cmd_ts:
        start_idx = np.argmax(cmd_ts[cmd_ts - start_cmd_ts <= 0])
    if cmd_ts.max() < end_cmd_ts:
        end_idx = np.argmax(cmd_ts[cmd_ts - end_cmd_ts <= 0])
    # print(start_idx, end_idx)

    # compute the entering jerk
    jerk_sum = (x_jerk[start_idx] + zrot_jerk[start_idx]) * (
        cmd_ts[start_idx + 1] - start_cmd_ts
    )
    for idx in range(1, end_idx - start_idx):
        jerk_sum += (x_jerk[start_idx + idx] + zrot_jerk[start_idx + idx]) * (
            cmd_ts[start_idx + idx + 1] - cmd_ts[start_idx + idx]
        )
    # compute the existing jerk
    jerk_sum += (x_jerk[end_idx] + zrot_jerk[end_idx]) * (end_cmd_ts - cmd_ts[end_idx])

    # normalized by time
    duration = end_cmd_ts - start_cmd_ts
    relative_jerk = jerk_sum / duration

    return relative_jerk


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
    vec_size = vel_c.shape[0]

    vel_r = qolo_state.get("x_vel")
    omega_r = qolo_state.get("zrot_vel")

    # TODO: fix lidar timestamp in
    # 2021-04-10-12-13-54 with 3464 frames
    # 2021-04-10-12-17-19 with 4759 frames
    # (12/13): 2021-04-10-12-24-53 with 489 frames
    #   ValueError: operands could not be broadcast together with shapes (479,) (489,)
    # 2021-04-10-12-26-08 with 3167 frames
    #   ValueError: operands could not be broadcast together with shapes (3157,) (3167,)

    # print(type(vel_c), type(omega_c))
    # print(vel_c.shape, omega_c.shape)
    # print(vel_r.shape, omega_r.shape)
    # (3464,) (3464,)
    # (3454,) (3454,)

    # TODO: divide by zero encountered in true_divide
    # 2021-04-10-10-58-23 with 579 frames
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
