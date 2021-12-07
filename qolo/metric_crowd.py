#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   metric_crowd.py
@Date created  :   2021/11/27
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides functions to calculate the crowd_density, min_dist, and
proximity around qolo detected pedestrian data.
"""
# =============================================================================

import math
import numpy as np

# borrow from https://github.com/epfl-lasa/qolo-evaluation/blob/main/src/crowd_evaluation/crowd_evaluation.py
class Capsule:
    def __init__(self, y_front, y_back, r):
        self.y_front = y_front
        self.y_back = y_back
        self.r = r

    def distanceLocal(self, x, y):
        if y > self.y_front:
            return math.sqrt(x * x + (y - self.y_front) * (y - self.y_front)) - self.r
        elif y < self.y_back:
            return math.sqrt(x * x + (y - self.y_back) * (y - self.y_back)) - self.r
        else:
            return math.fabs(x) - self.r

    def distanceGlobal(self, x_obs, y_obs, x_rob, y_rob, phi_rob):
        Rinv = [
            [np.cos(phi_rob - np.pi / 2), np.sin(phi_rob - np.pi / 2)],
            [-np.sin(phi_rob - np.pi / 2), np.cos(phi_rob - np.pi / 2)],
        ]

        p_rel_global = np.array([x_obs - x_rob, y_obs - y_rob])
        p_rel_local = np.matmul(np.array(Rinv), p_rel_global)
        return self.distanceLocal(p_rel_local[0], p_rel_local[1])


def compute_crowd_metrics(bbox):
    """compute crowd density and min_dist from qolo"""

    # 0. all_det
    all_det = np.shape(bbox)[0]

    # 1. all_dist: all pedestrain from qolo
    # all_dist = np.linalg.norm(bbox[:, [0, 1]], axis=1)
    bbox_xy = bbox[:, [0, 1]]
    all_dist = np.linalg.norm(bbox_xy, axis=1)
    # consider qolo capsule
    # all_dist_capsuled
    capsule_qolo = Capsule(0.18, -0.5, 0.45)
    all_dist_capsuled = np.ones([all_det]) * np.inf
    for idx in range(all_det):
        x_dist = bbox_xy[idx, 0]
        y_dist = bbox_xy[idx, 1]
        all_dist_capsuled[idx] = capsule_qolo.distanceLocal(x_dist, y_dist)

    # 2. within_det
    within_det3 = np.sum(np.less(all_dist, 3.0))
    within_det5 = np.sum(np.less(all_dist, 5.0))
    within_det10 = np.sum(np.less(all_dist, 10.0))

    # 3. min_dist/proximity: all_dist_capsuled or all_dist
    min_dist = min(all_dist_capsuled)

    # 4. crowd_density
    crowd_density3 = within_det3 / (np.pi * 3.0 ** 2)
    crowd_density5 = within_det5 / (np.pi * 5.0 ** 2)
    crowd_density10 = within_det10 / (np.pi * 10.0 ** 2)

    return (
        all_det,
        within_det3,
        within_det5,
        within_det10,
        crowd_density3,
        crowd_density5,
        crowd_density10,
        min_dist,
    )


def compute_norm_prox(min_dist_list):
    return np.std(min_dist_list) / np.mean(min_dist_list)
