#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   metric_qolo_perf.py
@Date created  :   2021/11/23
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides functions to compute metrics about the path efficiency and
shared control performance of qolo.
"""
# =============================================================================
# TODO: check the end_idx calculation

import numpy as np


def compute_time_path(qolo_twist, qolo_pose2d, goal_dist=20.0):
    """Compute starting and ending timestamp and path"""

    print("Distance to travel in the experiment: {} m".format(goal_dist))

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
    pose_theta = qolo_pose2d.get("theta")

    # init_yaw = np.sum(pose_theta[9:19]) / 10.0
    avg_num = 10
    init_yaw = np.sum(pose_theta[start_idx : start_idx + avg_num]) / avg_num
    goal_loc = np.array([np.cos(init_yaw), np.sin(init_yaw)]) * goal_dist

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
    vel_command_all = qolo_twist.get("x")
    twist_ts_all = qolo_twist.get("timestamp")
    start_vel_idx = start_idx
    end_vel_idx = (np.abs(twist_ts_all - pose_ts[end_idx])).argmin()
    # print("Debug:", start_vel_idx, end_vel_idx, len(twist_ts_all))
    # only consider Vel_command_max when travelling towards goal
    vc_max = np.max(np.abs(vel_command_all[start_vel_idx : end_vel_idx + 1]))

    start_pose_idx = (np.abs(pose_ts - start_ts)).argmin()
    end_pose_idx = end_idx
    pose_init = pose_x[start_pose_idx]
    pose_final = pose_x[end_pose_idx]
    l_goal = np.linalg.norm(pose_init - pose_final)

    theory_duration2goal = l_goal / vc_max
    rel_duration2goal = theory_duration2goal / duration2goal
    rel_path_length2goal = path_length2goal / l_goal

    return {
        "start_command_ts": start_ts,
        "end_command_ts": end_ts,
        "start_idx": start_idx,
        "end_idx": end_idx,
        "goal_loc": goal_loc,
        "goal_reached": goal_reached,
        "min_dist2goal": min_dist2goal,
        "duration2goal": duration2goal,
        "path_length2goal": path_length2goal,
        "rel_duration2goal": rel_duration2goal,
        "rel_path_length2goal": rel_path_length2goal,
    }


# deprecated because of using nominal sum
def compute_jerk(x_jerk, zrot_jerk, cmd_ts, start_cmd_ts, end_cmd_ts):
    """Compute relative jerk, which is the time integral of x_jerk and zrot_jerk
    (direct sum) along the operation duration. Can consider normalizing by the duration"""

    start_idx, end_idx = 0, len(cmd_ts) - 1
    # find start and ending points
    if cmd_ts.min() < start_cmd_ts:
        start_idx = np.argmax(cmd_ts[cmd_ts - start_cmd_ts <= 0])
    if cmd_ts.max() > end_cmd_ts:
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
    # compute the exiting jerk
    # jerk_sum += (x_jerk[end_idx] + zrot_jerk[end_idx]) * (end_cmd_ts - cmd_ts[end_idx])

    # normalized by time
    duration = end_cmd_ts - start_cmd_ts
    qolo_jerk = jerk_sum / duration

    return qolo_jerk


def compute_rel_jerk(
    x_jerk,
    zrot_jerk,
    cmd_ts,
    start_cmd_ts,
    end_cmd_ts,
    baseline=13.80,
):
    """Compute relative jerk, which is the time integral of x_jerk and zrot_jerk
    (square sum) along the operation duration. Can consider normalizing by the duration"""

    # load with actual index!!!
    start_idx, end_idx = 0, len(cmd_ts) - 1
    # find start and ending points
    if cmd_ts.min() < start_cmd_ts:
        start_idx = np.argmax(cmd_ts[cmd_ts - start_cmd_ts <= 0])
    if cmd_ts.max() > end_cmd_ts:
        end_idx = np.argmax(cmd_ts[cmd_ts - end_cmd_ts <= 0])
    # print(cmd_ts.min(), start_cmd_ts, cmd_ts[start_idx])
    # print(cmd_ts.max(), end_cmd_ts, cmd_ts[end_idx])

    # print(start_idx, end_idx)
    print("Avg linear jerk:", np.abs(x_jerk[start_idx : end_idx + 1]).mean())
    print("Avg angular jerk:", np.abs(zrot_jerk[start_idx : end_idx + 1]).mean())

    # compute the entering jerk
    # jerk_sum = np.linalg.norm([x_jerk[start_idx], zrot_jerk[start_idx]]) * (
    #     cmd_ts[start_idx + 1] - start_cmd_ts
    # )
    jerk_sum = 0
    for idx in range(1, end_idx - start_idx):
        jerk_sum += np.linalg.norm(
            [x_jerk[start_idx + idx], zrot_jerk[start_idx + idx]]
        ) * (cmd_ts[start_idx + idx + 1] - cmd_ts[start_idx + idx])
    # compute the exiting jerk
    # jerk_sum += np.linalg.norm([x_jerk[end_idx], zrot_jerk[end_idx]]) * (
    #     end_cmd_ts - cmd_ts[end_idx]
    # )
    # print('starting time', cmd_ts[start_idx + 1] - start_cmd_ts)
    # print('final time', end_cmd_ts - cmd_ts[end_idx])

    # normalized by time
    duration = end_cmd_ts - start_cmd_ts
    qolo_jerk = jerk_sum / duration
    rel_jerk = qolo_jerk / baseline
    print(
        "qolo_jerk = jerk_sum / duration: {} = {}/{}".format(
            qolo_jerk, jerk_sum, duration
        )
    )
    print('relative jerk:', rel_jerk)

    return rel_jerk


# https://github.com/epfl-lasa/qolo-evaluation/blob/main/notebook/crowd_evaluation.py#L187
def compute_fluency(qolo_command, start_cmd_ts, end_cmd_ts):
    """Compute consistency of the velocity command"""

    # from qolo_command_dict
    # vel = qolo_command.get("x_vel")
    # omega = qolo_command.get("zrot_vel")
    # from cmd_raw_dict
    vel = qolo_command.get("nominal_linear")
    omega = qolo_command.get("nominal_angular")

    # find start and ending timestamp
    cmd_ts = qolo_command.get("timestamp")
    start_idx, end_idx = 0, len(cmd_ts) - 1
    if cmd_ts.min() < start_cmd_ts:
        start_idx = np.argmax(cmd_ts[cmd_ts - start_cmd_ts <= 0])
    if cmd_ts.max() > end_cmd_ts:
        end_idx = np.argmax(cmd_ts[cmd_ts - end_cmd_ts <= 0])
    # print(start_idx, end_idx)

    # only consider data within the operation duration
    vel = vel[start_idx:end_idx]
    omega = omega[start_idx:end_idx]

    v_max, w_max = np.max(np.abs(vel)), np.max(np.abs(omega))

    # print(v_max, w_max)

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
# qolo_command & qolo_human_desired (from to GUI) but not `qolo_state`
def compute_agree_contri(
    qolo_command,
    start_cmd_ts,
    end_cmd_ts,
    control_type,
    vel_user_max,
    omega_user_max,
):
    """compute agreement and contribution for each type of control method"""

    print(
        "Nominal max velocity in the experiment: ({} m/s, {} rad/s)".format(
            vel_user_max, omega_user_max
        )
    )

    # unpack data
    vel_user = qolo_command.get("nominal_linear")
    omega_user = qolo_command.get("nominal_angular")
    vel_robo = qolo_command.get("corrected_linear")
    omega_robo = qolo_command.get("corrected_linear")

    # find start and ending timestamp
    cmd_ts = qolo_command.get("timestamp")
    start_idx, end_idx = 0, len(cmd_ts) - 1
    if cmd_ts.min() < start_cmd_ts:
        start_idx = np.argmax(cmd_ts[cmd_ts - start_cmd_ts <= 0])
    if cmd_ts.max() > end_cmd_ts:
        end_idx = np.argmax(cmd_ts[cmd_ts - end_cmd_ts <= 0])

    # only consider data within the operation duration
    vel_user = vel_user[start_idx:end_idx]
    omega_user = omega_user[start_idx:end_idx]
    vel_robo = vel_robo[start_idx:end_idx]
    omega_robo = omega_robo[start_idx:end_idx]

    # normalized by max speed -> contribution
    # if 'shared_control' in control_type:
    #     vel_user_max, omega_user_max = 1.2, 4.124
    # elif 'rds' in control_type:
    #     vel_user_max, omega_user_max = 0.9, 4.124 / 4
    # elif 'mds' in control_type:
    #     vel_user_max, omega_user_max = 0.9, 4.124 / 4
    # elif 'test' in control_type:
    #     vel_user_max, omega_user_max = 1.2, 1.0

    norm_vel_u = vel_user / vel_user_max
    norm_omega_u = omega_user / omega_user_max
    norm_vel_r = vel_robo / vel_user_max
    norm_omega_r = omega_robo / omega_user_max

    angle_user = np.arctan2(norm_vel_u, norm_omega_u)
    angle_rds = np.arctan2(norm_vel_r, norm_omega_r)
    # angle_user = np.arctan2(vel_user, omega_user)
    # angle_rds = np.arctan2(vel_rds, omega_rds)
    angle_diff = np.abs(angle_rds - angle_user)

    omega_diff = []
    vel_diff = []
    agreement_vec = []
    for idx in range(vel_user.shape[0]):
        if vel_user[idx] or omega_user[idx]:
            vel_diff.append(np.abs(vel_robo[idx] - vel_user[idx]) / vel_user_max)
            omega_diff.append(np.abs(vel_robo[idx] - omega_user[idx]) / omega_user_max)
            agreement_vec.append(1 - (abs(angle_diff[idx]) / np.pi))

    vel_diff = np.array(vel_diff)
    omega_diff = np.array(omega_diff)

    agreement_vec = np.array(agreement_vec)

    cmd_diff_vw = np.column_stack((vel_diff, omega_diff))
    cmd_diff = np.linalg.norm(cmd_diff_vw, axis=1)

    # contribution
    contribution_vec = []
    u_human = np.vstack((norm_vel_u, norm_omega_u)).T  # human
    u_robot = np.vstack((norm_vel_r, norm_omega_r)).T  # rds output
    if control_type in ['mds']:
        u_diff = u_robot
    else:
        u_diff = u_human - u_robot
    u_human_norm = np.linalg.norm(u_human, axis=1)
    u_diff_norm = np.linalg.norm(u_diff, axis=1)
    for idx in range(vel_user.shape[0]):

        if u_human[idx, 0] or u_human[idx, 1]:
            if control_type in ['mds']:
                if u_human[idx, 0] == 0:
                    contribution_vec.append(u_diff[idx, 1])
                elif u_human[idx, 1] == 0:
                    contribution_vec.append(u_diff[idx, 0])
                else:
                    contribution_vec.append(u_diff_norm[idx] / np.sqrt(2))
            else:

                if (u_human[idx, 0] == 0) and (u_human[idx, 1] == 0):
                    if np.nonzero(u_human[idx, :]):
                        contribution_vec.append(u_diff_norm[idx])
                    else:
                        continue
                elif u_human[idx, 0] == 0:
                    contribution_vec.append(u_diff[idx, 1] / u_human[idx, 1])
                elif u_human[idx, 1] == 0:
                    contribution_vec.append(u_diff[idx, 0] / u_human[idx, 0])
                else:
                    contribution_vec.append(u_diff_norm[idx] / u_human_norm[idx])
    contribution = np.mean(contribution_vec)

    return (
        contribution,
        np.mean(agreement_vec),
        np.std(agreement_vec),
        np.mean(cmd_diff),
        np.std(cmd_diff),
        np.mean(vel_diff),
        np.std(vel_diff),
        np.mean(omega_diff),
        np.std(omega_diff),
    )
