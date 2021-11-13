# -*-coding:utf-8 -*-
"""
@File    :   viz_util.py
@Time    :   2021/10/20
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""


import os
import numpy as np

from transformation_utils import boxes_to_corners

# # for running on headerless server
# from pyvirtualdisplay import Display
# display = Display(visible=False, size=(1280, 1024))
# display.start()

from mayavi import mlab

import cv2


def id2color(id_):
    c_hsv = np.empty((1, 1, 3), dtype=np.float32)
    c_hsv[0, :, 0] = float((id_ * 33) % 360)
    c_hsv[0, :, 1] = 1
    c_hsv[0, :, 2] = 1
    c_bgr = cv2.cvtColor(c_hsv, cv2.COLOR_HSV2RGB)[0]
    return tuple(*c_bgr)


# viz with open3d
# ref: http://www.open3d.org/docs/release/index.html#python-api-index
import open3d as o3d


def plot_frame_o3d(lidar, boxes, out_path=None):
    # some nice colors
    gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
    gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
    gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
    gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
    gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
    gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

    fig = mlab.figure(
        figure=None,
        bgcolor=(1, 1, 1),
        fgcolor=(0, 0, 0),
        engine=None,
        size=(1600, 1000),
    )


# viz with mlab
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
