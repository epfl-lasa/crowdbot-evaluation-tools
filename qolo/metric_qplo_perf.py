#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   metric_qplo_perf.py
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
"""
TODO:
1. use qolo_twist and qolo_pose2d sample at the same timestamps
"""
# =============================================================================

import numpy as np


def compute_time_path(qolo_twist, qolo_pose2d):
    """Compute starting and ending timestamp and path"""

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
    vc_max = np.max(np.abs(vel_command_all[start_vel_idx : end_vel_idx + 1]))

    start_pose_idx = (np.abs(pose_ts - start_ts)).argmin()
    end_pose_idx = end_idx
    pose_init = pose_x[start_pose_idx]
    pose_final = pose_x[end_pose_idx]
    l_goal = np.linalg.norm(pose_init - pose_final)

    theory_duration2goal = l_goal / vc_max
    rel_duration2goal = theory_duration2goal / duration2goal
    rel_path_length2goal = path_length2goal / l_goal

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
    """Compute relative jerk, which is the time integral of x_jerk and zrot_jerk
    along the operation duration. Can consider normalizing by the duration"""

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
    if cmd_ts.max() < end_cmd_ts:
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
def compute_agreement(qolo_command, start_cmd_ts, end_cmd_ts):
    """compute real qolo velocity in reference to command"""

    # unpack data
    vel_user = qolo_command.get("nominal_linear")
    omega_user = qolo_command.get("nominal_angular")
    vel_rds = qolo_command.get("corrected_linear")
    omega_rds = qolo_command.get("corrected_linear")

    # find start and ending timestamp
    cmd_ts = qolo_command.get("timestamp")
    start_idx, end_idx = 0, len(cmd_ts) - 1
    if cmd_ts.min() < start_cmd_ts:
        start_idx = np.argmax(cmd_ts[cmd_ts - start_cmd_ts <= 0])
    if cmd_ts.max() < end_cmd_ts:
        end_idx = np.argmax(cmd_ts[cmd_ts - end_cmd_ts <= 0])
    # print(start_idx, end_idx)

    # only consider data within the operation duration
    vel_user = vel_user[start_idx:end_idx]
    omega_user = omega_user[start_idx:end_idx]
    vel_rds = vel_rds[start_idx:end_idx]
    omega_rds = omega_rds[start_idx:end_idx]

    # normalized by max speed -> contribution
    vel_user_max, omega_user_max = np.max(np.abs(vel_user)), np.max(np.abs(omega_user))
    # print(vel_user_max, omega_user_max)
    # print("start_t/end_t:", start_cmd_ts - cmd_ts.min(), end_cmd_ts - cmd_ts.min())
    norm_vel_u = vel_user / vel_user_max
    norm_omega_u = omega_user / omega_user_max
    norm_vel_r = vel_rds / vel_user_max
    norm_omega_r = omega_rds / omega_user_max

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
            vel_diff.append(np.abs(vel_rds[idx] - vel_user[idx]) / vel_user_max)
            omega_diff.append(np.abs(omega_rds[idx] - omega_user[idx]) / omega_user_max)
            agreement_vec.append(1 - (abs(angle_diff[idx]) / np.pi))

    vel_diff = np.array(vel_diff)
    omega_diff = np.array(omega_diff)

    agreement_vec = np.array(agreement_vec)

    cmd_diff_vw = np.column_stack((vel_diff, omega_diff))
    cmd_diff = np.linalg.norm(cmd_diff_vw, axis=1)

    # agreement_data = agreement_vec.mean(), agreement_vec.std()
    # cmd_diff_data = [np.mean(command_vec), np.std(command_vec)]
    # linear_data = [vel_diff.mean(), vel_diff.std()]
    # heading_data = [omega_diff.mean(), omega_diff.std()]

    # contribution
    contribution_vec = []
    u_human = np.vstack((norm_vel_u, norm_omega_u)).T  # human
    u_rds = np.vstack((norm_vel_r, norm_omega_r)).T  # rds output
    u_diff = u_human - u_rds
    u_human_norm = np.linalg.norm(u_human, axis=1)
    u_diff_norm = np.linalg.norm(u_diff, axis=1)
    for idx in range(vel_user.shape[0]):
        if u_human[idx, 0] or u_human[idx, 1]:
            if u_human[idx, 0] == 0:
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


"""
single_oa/2021-11-26-12-50-09.bag

consider using abs -> fixed

line 171: v_max, w_max = np.max(np.abs(vel)), np.max(np.abs(omega))
line 221: vel_user_max, omega_user_max = np.max(np.abs(vel_user)), np.max(np.abs(omega_user))

```
$ python3 ../qolo/eval_qolo.py --overwrite -f single_oa
(12/14): 2021-11-26-12-50-09 with 338 frames
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:181: RuntimeWarning: invalid value encountered in double_scalars
  fluency_w.append(1 - np.abs(omega[idx] - omega[idx - 1]) / w_max)
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:181: RuntimeWarning: divide by zero encountered in double_scalars
  fluency_w.append(1 - np.abs(omega[idx] - omega[idx - 1]) / w_max)
0.7999879717826843 0.0
start_t/end_t: 3.408465623855591 17.172191619873047
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:225: RuntimeWarning: divide by zero encountered in true_divide
  norm_omega_u = omega_user / omega_user_max
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:225: RuntimeWarning: invalid value encountered in true_divide
  norm_omega_u = omega_user / omega_user_max
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:227: RuntimeWarning: divide by zero encountered in true_divide
  norm_omega_r = omega_rds / omega_user_max
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:241: RuntimeWarning: divide by zero encountered in double_scalars
  omega_diff.append(np.abs(omega_rds[idx] - omega_user[idx]) / omega_user_max)
/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/metric_qplo_perf.py:271: RuntimeWarning: invalid value encountered in double_scalars
  contribution_vec.append(u_diff_norm[idx] / u_human_norm[idx])
/home/crowdbot/.local/lib/python3.8/site-packages/numpy/core/_methods.py:230: RuntimeWarning: invalid value encountered in subtract
  x = asanyarray(arr - arrmean)
```
"""
