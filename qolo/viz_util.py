#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   viz_util.py
@Date created  :   2021/11/30
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides function for detection or tracking results preprocessing
for plotting, bbox projection and colorization.
"""
# =============================================================================


import cv2
import numpy as np

#%% Utility functions for colorization, bbox projection
def id2color(id_):
    """convert id to color series"""
    c_hsv = np.empty((1, 1, 3), dtype=np.float32)
    c_hsv[0, :, 0] = float((id_ * 33) % 360)
    c_hsv[0, :, 1] = 1
    c_hsv[0, :, 2] = 1
    c_bgr = cv2.cvtColor(c_hsv, cv2.COLOR_HSV2RGB)[0]
    return tuple(*c_bgr)


# convert box to bbox point set
bbox_lines = [
    [0, 1],
    [0, 3],
    [0, 4],
    [1, 2],
    [1, 5],
    [2, 3],
    [2, 6],
    [3, 7],
    [4, 5],
    [4, 7],
    [5, 6],
    [6, 7],
    [0, 5],
    [1, 4],
]


def boxes3d_to_corners3d(boxes3d, bottom_center=False):
    """
    :param boxes3d: (N, 7) [x, y, z, dx, dy, dz, heading] in LiDAR coords, +x points to right (2 to 1),
                    +y points front  (from 1 to 0), +z points upwards (from 2 to 6),
    :param bottom_center: whether z is on the bottom center of object
    :return: corners3d: (N, 8, 3)
        7 -------- 4
       /|         /|
      6 -------- 5 .
      | |        | |
      . 3 -------- 0
      |/         |/
      2 -------- 1
    """
    boxes_num = boxes3d.shape[0]
    dx, dy, dz = boxes3d[:, 3], boxes3d[:, 4], boxes3d[:, 5]
    x_corners = np.array(
        [
            dx / 2.0,
            dx / 2.0,
            -dx / 2.0,
            -dx / 2.0,
            dx / 2.0,
            dx / 2.0,
            -dx / 2.0,
            -dx / 2.0,
        ],
        dtype=np.float32,
    ).T
    y_corners = np.array(
        [
            dy / 2.0,
            -dy / 2.0,
            -dy / 2.0,
            dy / 2.0,
            dy / 2.0,
            -dy / 2.0,
            -dy / 2.0,
            dy / 2.0,
        ],
        dtype=np.float32,
    ).T
    z_corners = np.array(
        [
            dz / 2.0,
            dz / 2.0,
            dz / 2.0,
            dz / 2.0,
            -dz / 2.0,
            -dz / 2.0,
            -dz / 2.0,
            -dz / 2.0,
        ],
        dtype=np.float32,
    ).T

    ry = boxes3d[:, 6]
    zeros, ones = np.zeros(ry.size, dtype=np.float32), np.ones(
        ry.size, dtype=np.float32
    )
    # counter-clockwisely rotate the frame around z by an angle ry
    # note the transform is done by Vector x Matrix instead of Matrix x Vector,
    # which means the Matrix need to be transposed when interpreted as a linear transform
    rot_list = np.array(
        [
            [np.cos(ry), np.sin(ry), zeros],
            [-np.sin(ry), np.cos(ry), zeros],
            [zeros, zeros, ones],
        ]
    )  # (3, 3, N)
    R_list = np.transpose(rot_list, (2, 0, 1))  # (N, 3, 3)

    temp_corners = np.concatenate(
        (
            x_corners.reshape(-1, 8, 1),
            y_corners.reshape(-1, 8, 1),
            z_corners.reshape(-1, 8, 1),
        ),
        axis=2,
    )  # (N, 8, 3)

    rotated_corners = np.matmul(temp_corners, R_list)  # (N, 8, 3)
    x_corners, y_corners, z_corners = (
        rotated_corners[:, :, 0],
        rotated_corners[:, :, 1],
        rotated_corners[:, :, 2],
    )

    x_loc, y_loc, z_loc = boxes3d[:, 0], boxes3d[:, 1], boxes3d[:, 2]

    x = x_loc.reshape(-1, 1) + x_corners.reshape(-1, 8)
    y = y_loc.reshape(-1, 1) + y_corners.reshape(-1, 8)
    z = z_loc.reshape(-1, 1) + z_corners.reshape(-1, 8)

    corners = np.concatenate(
        (x.reshape(-1, 8, 1), y.reshape(-1, 8, 1), z.reshape(-1, 8, 1)), axis=2
    )

    return corners.astype(np.float32)


def filter_pointcloud_distance(in_cloud, dist=10.0, verbose=False):
    """filter detected pointcloud/pedestrain within the desired distance"""
    r_square_within = (in_cloud[:, 0] ** 2 + in_cloud[:, 1] ** 2) < dist ** 2
    out_cloud = in_cloud[r_square_within, :]
    if verbose:
        print(
            "Filtered/Overall pts: {}/{}".format(
                np.shape(in_cloud)[0], np.shape(out_cloud)[0]
            )
        )
    return out_cloud


def filter_detection_tracking_res(in_boxes, dist=10.0, verbose=False):
    """
    Filter detected pointcloud/pedestrain within the desired distance
    Input:
        in_boxes: (N, 7) [x, y, z, dx, dy, dz, heading] in LiDAR coords
    """
    r_square_within = (in_boxes[:, 0] ** 2 + in_boxes[:, 1] ** 2) < dist ** 2
    out_boxes = in_boxes[r_square_within, :]
    if verbose:
        print(
            "Filtered/Overall boxes: {}/{}".format(
                np.shape(in_boxes)[0], np.shape(out_boxes)[0]
            )
        )
    return out_boxes
