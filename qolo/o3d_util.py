#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   o3d_util.py
@Date created  :   2021/10/26
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides functions to process detection or tracking results and plot
function with open3d.
"""
# =============================================================================
"""
TODO:
1. plot detection result like tracking results! (implement as `pass` now)
2. visualize past trajectories with line-like style (draw 3D balls every N frames currently)
"""
# =============================================================================

import os, copy
from pickle import load, dump

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import open3d as o3d

from viz_util import (
    id2color,
    bbox_lines,
    boxes3d_to_corners3d_lidar,
    filter_pointcloud_distance,
    filter_detection_tracking_res,
)


#%% LineMesh module from https://github.com/isl-org/Open3D/pull/738#issuecomment-564785941 (line_width got deprecated)
def align_vector_to_another(a=np.array([0, 0, 1]), b=np.array([1, 0, 0])):
    """Aligns vector a to vector b with axis angle rotation"""
    if np.array_equal(a, b):
        return None, None
    axis_ = np.cross(a, b)
    axis_ = axis_ / np.linalg.norm(axis_)
    angle = np.arccos(np.dot(a, b))

    return axis_, angle


def normalized(a, axis=-1, order=2):
    """Normalizes a numpy array of points"""
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2 == 0] = 1
    return a / np.expand_dims(l2, axis), l2


class LineMesh(object):
    def __init__(self, points, lines=None, colors=[0, 1, 0], radius=0.15):
        """Creates a line represented as sequence of cylinder triangular meshes

        Arguments:
            points {ndarray} -- Numpy array of ponts Nx3.

        Keyword Arguments:
            lines {list[list] or None} -- List of point index pairs denoting line segments. If None, implicit lines from ordered pairwise points. (default: {None})
            colors {list} -- list of colors, or single color of the line (default: {[0, 1, 0]})
            radius {float} -- radius of cylinder (default: {0.15})
        """
        self.points = np.array(points)
        self.lines = (
            np.array(lines)
            if lines is not None
            else self.lines_from_ordered_points(self.points)
        )
        self.colors = np.array(colors)
        self.radius = radius
        self.cylinder_segments = []

        self.create_line_mesh()

    @staticmethod
    def lines_from_ordered_points(points):
        lines = [[i, i + 1] for i in range(0, points.shape[0] - 1, 1)]
        return np.array(lines)

    def create_line_mesh(self):
        first_points = self.points[self.lines[:, 0], :]
        second_points = self.points[self.lines[:, 1], :]
        line_segments = second_points - first_points
        line_segments_unit, line_lengths = normalized(line_segments)

        z_axis = np.array([0, 0, 1])
        # Create triangular mesh cylinder segments of line
        for i in range(line_segments_unit.shape[0]):
            line_segment = line_segments_unit[i, :]
            line_length = line_lengths[i]
            # get axis angle rotation to allign cylinder with line segment
            axis, angle = align_vector_to_another(z_axis, line_segment)
            # Get translation vector
            translation = first_points[i, :] + line_segment * line_length * 0.5
            # create cylinder and apply transformations
            cylinder_segment = o3d.geometry.TriangleMesh.create_cylinder(
                self.radius, line_length
            )
            cylinder_segment = cylinder_segment.translate(translation, relative=False)
            if axis is not None:
                axis_a = axis * angle
                cylinder_segment = cylinder_segment.rotate(
                    R=o3d.geometry.get_rotation_matrix_from_axis_angle(axis_a),
                    center=cylinder_segment.get_center(),
                )
                # cylinder_segment = cylinder_segment.rotate(
                #     R=o3d.geometry.get_rotation_matrix_from_axis_angle(axis_a), center=True)
                # cylinder_segment = cylinder_segment.rotate(
                #   axis_a, center=True, type=o3d.geometry.RotationType.AxisAngle)
            # color cylinder
            color = self.colors if self.colors.ndim == 1 else self.colors[i, :]
            cylinder_segment.paint_uniform_color(color)

            self.cylinder_segments.append(cylinder_segment)

    def add_line(self, vis):
        """Adds this line to the visualizer"""
        for cylinder in self.cylinder_segments:
            vis.add_geometry(cylinder)

    def remove_line(self, vis):
        """Removes this line from the visualizer"""
        for cylinder in self.cylinder_segments:
            vis.remove_geometry(cylinder)


# "/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/saved_view.pkl"
def load_camera_info(
    camera_param,
    fname=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'saved_view.pkl'),
):
    # ref: # https://github.com/pablospe/render_depthmap_example/blob/main/visualization.py
    with open(fname, "rb") as pickle_file:
        saved_view = load(pickle_file)
    camera_param.extrinsic = saved_view["extrinsic"]
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        saved_view["width"],
        saved_view["height"],
        saved_view["intrinsic"][0][0],
        saved_view["intrinsic"][1][1],
        saved_view["intrinsic"][0][2],
        saved_view["intrinsic"][1][2],
    )
    camera_param.intrinsic = intrinsic
    return camera_param


#%% Plot in qolo frame with Open3D
def plot_robot_frame_o3d(
    lidar,
    boxes=None,
    out_path=None,
    width=1080,
    height=720,
    show_xyz=True,
    filtering=False,
    filter_dist=8.0,
    verbose=False,
):

    # some nice colors
    gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
    gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
    gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
    gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
    gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
    gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

    show_list = []
    origin_xyz = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=np.array([0, 0, 0])
    )
    if show_xyz:
        show_list.append(origin_xyz)

    ## visualize all lidar points -> include scale variations!!!
    if filtering:
        lidar = filter_pointcloud_distance(lidar, filter_dist, verbose)
    displayed_pointcloud = o3d.geometry.PointCloud()
    displayed_pointcloud.points = o3d.utility.Vector3dVector(lidar[:, :3])
    displayed_pointcloud.paint_uniform_color(gs_blue)
    show_list.append(displayed_pointcloud)

    # visualize detected/tracking results
    if boxes:
        if filtering:
            boxes = filter_detection_tracking_res(boxes, filter_dist, verbose)
        if boxes.shape[1] == 7:
            pass
        # or tracks
        elif boxes.shape[1] == 8:
            ids = boxes[:, -1]
            boxes = boxes[:, :-1]
            corners_xyz = boxes3d_to_corners3d_lidar(boxes, bottom_center=False)
            # corners_xyz, connect_inds = boxes_to_corners(boxes, connect_inds=True)

            for id_, corners in zip(ids, corners_xyz):
                c = id2color(id_)
                colors = [c for _ in range(len(bbox_lines))]

                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(corners),
                    lines=o3d.utility.Vector2iVector(bbox_lines),
                )
                line_set.colors = o3d.utility.Vector3dVector(colors)
                show_list.append(line_set)

                line_mesh = LineMesh(corners, bbox_lines, colors, radius=0.02)
                line_mesh_geoms = line_mesh.cylinder_segments
                # show_list.append([*line_mesh_geoms])
                show_list += line_mesh_geoms  # list of TriangleMesh

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="crowdbot", width=width, height=height)
    # vis.create_window(visible = False)
    for item in show_list:
        vis.add_geometry(item)

    # ViewControl: http://www.open3d.org/docs/0.12.0/python_api/open3d.visualization.ViewControl.html
    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=90.0)
    ctr.set_zoom(0.8)  # will be overwritten by camera_param
    # ctr.set_lookat([-3, -3, -1])
    # set viewpoint
    # camera_param = ctr.convert_to_pinhole_camera_parameters()
    # camera_param = load_camera_info(camera_param)
    # ctr.convert_from_pinhole_camera_parameters(camera_param, allow_arbitrary=True)

    # RenderOption
    opt = vis.get_render_option()
    opt.background_color = np.asarray([1, 1, 1])  # b: [0, 0, 0]; w: [1, 1, 1]
    opt.point_size = float(2)
    # opt.line_width = float(8)

    if out_path is not None:
        vis.poll_events()
        vis.update_renderer()
        # https://github.com/isl-org/Open3D/issues/1095
        # o3d.io.write_image(os.path.join(out_path, "test.jpg"), img)
        # vis.capture_screen_image(out_path)
        image = vis.capture_screen_float_buffer(False)
        plt.imsave(out_path, np.asarray(image), dpi=300)
    else:
        vis.run()


#%% Plot in world frame with Open3D
def plot_world_frame_o3d(
    lidar=None,
    pose=None,
    boxes=None,
    out_path=None,
    width=1080,
    height=720,
    show_xyz=False,
    show_origin=True,
    filtering=True,
    filter_dist=8.0,
    verbose=False,
):

    # some nice colors
    gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
    gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
    gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
    gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
    gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
    gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

    show_list = []

    ## visualize with robot pose
    if pose:
        (trans_list, rot_quat_list) = pose
        trans_curr = trans_list[-1, :]
        # print(trans_curr)
        # scipy: (x, y, z, w) from https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html
        # open3d: (w, x, y, z) `Eigen::Quaterniond` from https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
        # https://github.com/isl-org/Open3D/blob/master/cpp/open3d/geometry/Geometry3D.cpp#L216
        # save quat format -> scipy: (x, y, z, w)
        # used quat format -> open3d: (w, x, y, z)

        scipy_rots = R.from_quat(rot_quat_list)
        # compute relative rotation to frame1
        scipy_rots_aligned = scipy_rots.reduce(
            left=R.from_quat(rot_quat_list[0, :]).inv()
        )
        rot_quat_list_aligned = scipy_rots_aligned.as_quat()
        # different quat description in o3d and scipy
        rot_quat_list_aligned[:, [0, 1, 2, 3]] = rot_quat_list_aligned[:, [1, 2, 3, 0]]
        rot_quat_curr = rot_quat_list_aligned[-1, :]

        # robot_xyz = copy.deepcopy(origin_xyz)
        robot_xyz = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=2.0, origin=np.array([0, 0, 0])
        )
        # http://www.open3d.org/docs/latest/tutorial/Basic/transformation.html
        rot = robot_xyz.get_rotation_matrix_from_quaternion(rot_quat_curr)
        # translate(self, translation, relative=True) numpy.ndarray[numpy.float64[3, 1]]
        # rotate(self, R)
        if show_xyz:
            show_list.append(robot_xyz)

    ## visualize all lidar points -> include scale variations!!!
    if len(lidar):
        if filtering:
            lidar = filter_pointcloud_distance(lidar, filter_dist, verbose)
        displayed_pointcloud = o3d.geometry.PointCloud()
        displayed_pointcloud.points = o3d.utility.Vector3dVector(lidar[:, :3])
        displayed_pointcloud.paint_uniform_color(gs_blue)
        show_list.append(displayed_pointcloud)

    # visualize detected/tracking results
    if len(boxes):
        if filtering:
            boxes = filter_detection_tracking_res(boxes, filter_dist, verbose)
        if boxes.shape[1] == 7:
            pass
        # or tracks
        elif boxes.shape[1] == 8:
            ids = boxes[:, -1]
            boxes = boxes[:, :-1]
            corners_xyz = boxes3d_to_corners3d_lidar(boxes, bottom_center=False)
            # corners_xyz, connect_inds = boxes_to_corners(boxes, connect_inds=True)

            for id_, corners in zip(ids, corners_xyz):
                c = id2color(id_)
                colors = [c for _ in range(len(bbox_lines))]

                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(corners),
                    lines=o3d.utility.Vector2iVector(bbox_lines),
                )
                line_set.colors = o3d.utility.Vector3dVector(colors)
                show_list.append(line_set)

                line_mesh = LineMesh(corners, bbox_lines, colors, radius=0.02)
                line_mesh_geoms = line_mesh.cylinder_segments
                # show_list.append([*line_mesh_geoms])
                show_list += line_mesh_geoms  # list of TriangleMesh

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="crowdbot", width=width, height=height)
    # vis.create_window(visible = False)
    for item in show_list:
        if pose:
            vis.add_geometry(item.rotate(rot, center=(0, 0, 0)).translate(trans_curr))
        else:
            vis.add_geometry(item)

    if pose:
        # for i, trans in enumerate(trans_list):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        mesh.compute_vertex_normals()
        # mode 1: show past 50 frames traj
        # viz_trans_list = trans_list[-50:,:] # trans_list
        # mode 2: show past traj sparsely (every 20 frame)
        viz_trans_list = trans_list[0:-1:20, :]
        traj_list = [copy.deepcopy(mesh) for elem in range(len(viz_trans_list))]
        for trans, traj in zip(viz_trans_list, traj_list):
            vis.add_geometry(traj.translate(trans))

    origin_xyz = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=3.0, origin=np.array([-1, -1, -1])
    )
    if show_origin:
        vis.add_geometry(origin_xyz)

    # ViewControl: http://www.open3d.org/docs/0.12.0/python_api/open3d.visualization.ViewControl.html
    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=90.0)
    ctr.set_zoom(0.8)  # will be overwritten by camera_param
    # ctr.set_lookat([-3, -3, -1])
    # set viewpoint
    camera_param = ctr.convert_to_pinhole_camera_parameters()
    camera_param = load_camera_info(camera_param)
    ctr.convert_from_pinhole_camera_parameters(camera_param, allow_arbitrary=True)

    # RenderOption
    opt = vis.get_render_option()
    opt.background_color = np.asarray([1, 1, 1])  # b: [0, 0, 0]; w: [1, 1, 1]
    opt.point_size = float(2)
    # opt.line_width = float(8)

    if out_path is not None:
        vis.poll_events()
        vis.update_renderer()
        # https://github.com/isl-org/Open3D/issues/1095
        # o3d.io.write_image(os.path.join(out_path, "test.jpg"), img)
        # vis.capture_screen_image(out_path)
        image = vis.capture_screen_float_buffer(False)
        plt.imsave(out_path, np.asarray(image), dpi=300)
    else:
        vis.run()
