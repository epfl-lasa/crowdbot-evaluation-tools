#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   viz_util.py
@Date created  :   2021/10/20
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides functions to process detection or tracking results and plot
function with mayavi.

(Deprecated due to inconvenient environment configuration of mayavi!!!)
"""
# =============================================================================


import numpy as np

# # for running on headerless server
# from pyvirtualdisplay import Display
# display = Display(visible=False, size=(1280, 1024))
# display.start()

from mayavi import mlab

import cv2

def boxes_get_R(boxes):
    """Get rotation matrix R along z axis that transforms a point from box coordinate
    to world coordinate.

    Args:
        boxes (array[B, 7]): (x, y, z, l, w, h, theta) of each box

    Returns:
        Rs (array[B, 3, 3])
    """
    # NOTE plus pi specifically for JRDB, don't know the reason
    theta = boxes[:, 6] + np.pi
    cs, ss = np.cos(theta), np.sin(theta)
    zeros, ones = np.zeros(len(cs)), np.ones(len(cs))
    Rs = np.array(
        [[cs, ss, zeros], [-ss, cs, zeros], [zeros, zeros, ones]], dtype=np.float32
    )  # (3, 3, B)

    return Rs.transpose((2, 0, 1))

def boxes_to_corners(boxes, resize_factor=1.0, connect_inds=False):
    """Return xyz coordinates of the eight vertices of the bounding box

    First four points are fl (front left), fr, br, bl on top plane. Last four
    points are same order, but for the bottom plane.

          0 -------- 1        __
         /|         /|        //|
        3 -------- 2 .       //
        | |        | |      front
        . 4 -------- 5
        |/         |/
        7 -------- 6

    To draw a box, do something like

    corners, connect_inds = boxes_to_corners(boxes)
    for corner in corners:
        for inds in connect_inds:
            mlat.plot3d(corner[0, inds], corner[1, inds], corner[2, inds],
                        tube_radius=None, line_width=5)

    Args:
        boxes (array[B, 7]): (x, y, z, l, w, h, theta) of each box
        resize_factor (float): resize box lwh dimension
        connect_inds(bool): true will also return a list of indices for drawing
            the box as line segments

    Returns:
        corners_xyz (array[B, 3, 8])
        connect_inds (tuple[list[int]])
    """
    # in box frame
    c_xyz = np.array(
        [
            [1, 1, -1, -1, 1, 1, -1, -1],
            [-1, 1, 1, -1, -1, 1, 1, -1],
            [1, 1, 1, 1, -1, -1, -1, -1],
        ],
        dtype=np.float32,
    )  # (3, 8)
    c_xyz = 0.5 * c_xyz[np.newaxis, :, :] * boxes[:, 3:6, np.newaxis]  # (B, 3, 8)
    c_xyz = c_xyz * resize_factor

    # to world frame
    R = boxes_get_R(boxes)  # (B, 3, 3)
    c_xyz = R @ c_xyz + boxes[:, :3, np.newaxis]  # (B, 3, 8)

    if not connect_inds:
        return c_xyz
    else:
        l1 = [0, 1, 2, 3, 0, 4, 5, 6, 7, 4, 1]
        l2 = [0, 5, 1]
        l3 = [2, 6]
        l4 = [3, 7]
        return c_xyz, (l1, l2, l3, l4)


def id2color(id_):
    c_hsv = np.empty((1, 1, 3), dtype=np.float32)
    c_hsv[0, :, 0] = float((id_ * 33) % 360)
    c_hsv[0, :, 1] = 1
    c_hsv[0, :, 2] = 1
    c_bgr = cv2.cvtColor(c_hsv, cv2.COLOR_HSV2RGB)[0]
    return tuple(*c_bgr)


def plot_frame(lidar, boxes, out_path=None):
    # some nice colors
    gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
    gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
    gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
    gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
    gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
    gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

    if out_path is not None:
        # before you create a figure and it will use an offscreen window for the rendering
        mlab.options.offscreen = True

    fig = mlab.figure(
        figure=None,
        bgcolor=(1, 1, 1),
        fgcolor=(0, 0, 0),
        engine=None,
        size=(1600, 1000),
    )

    # visualize all lidar points -> include scale variations!!!
    mlab.points3d(
        lidar[0],
        lidar[1],
        lidar[2],
        scale_factor=0.05,
        color=gs_blue,
        figure=fig,
    )

    # plot detections
    if boxes.shape[1] == 7:
        corners_xyz, connect_inds = boxes_to_corners(boxes, connect_inds=True)
        for corner_xyz in corners_xyz:
            for inds in connect_inds:
                mlab.plot3d(
                    corner_xyz[0, inds],
                    corner_xyz[1, inds],
                    corner_xyz[2, inds],
                    tube_radius=None,
                    line_width=3,
                    color=gs_yellow,
                    figure=fig,
                )
    # or tracks
    elif boxes.shape[1] == 8:
        ids = boxes[:, -1]
        boxes = boxes[:, :-1]
        corners_xyz, connect_inds = boxes_to_corners(boxes, connect_inds=True)
        for id_, corner_xyz in zip(ids, corners_xyz):
            c = id2color(id_)
            for inds in connect_inds:
                mlab.plot3d(
                    corner_xyz[0, inds],
                    corner_xyz[1, inds],
                    corner_xyz[2, inds],
                    tube_radius=None,
                    line_width=3,
                    color=c,
                    figure=fig,
                )

    mlab.view(focalpoint=(0, 0, 0))
    mlab.move(50, 0, 5.0)
    # mlab.pitch(-10)

    if out_path is not None:
        mlab.savefig(out_path)
    else:
        mlab.show()

    return fig
