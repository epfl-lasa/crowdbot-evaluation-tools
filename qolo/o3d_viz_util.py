# -*-coding:utf-8 -*-
'''
@File    :   o3d_viz_util.py
@Time    :   2021/10/26
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

import os, copy
from pickle import load, dump

import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

#%% LineMesh module from https://github.com/isl-org/Open3D/pull/738#issuecomment-564785941 (line_width got deprecated)
def align_vector_to_another(a=np.array([0, 0, 1]), b=np.array([1, 0, 0])):
    """
    Aligns vector a to vector b with axis angle rotation
    """
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
        self.lines = np.array(
            lines) if lines is not None else self.lines_from_ordered_points(self.points)
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
                self.radius, line_length)
            cylinder_segment = cylinder_segment.translate(
                translation, relative=False)
            if axis is not None:
                axis_a = axis * angle
                cylinder_segment = cylinder_segment.rotate(
                    R=o3d.geometry.get_rotation_matrix_from_axis_angle(axis_a), 
                    center=cylinder_segment.get_center())
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


#%% Utility functions for generate components

# convert id to color series
def id2color(id_):
    c_hsv = np.empty((1, 1, 3), dtype=np.float32)
    c_hsv[0, :, 0] = float((id_ * 33) % 360)
    c_hsv[0, :, 1] = 1
    c_hsv[0, :, 2] = 1
    c_bgr = cv2.cvtColor(c_hsv, cv2.COLOR_HSV2RGB)[0]
    return tuple(*c_bgr)

# convert box to bbox point set
lines = [
    [0, 1], [0, 3], [0, 4],
    [1, 2], [1, 5], [2, 3],
    [2, 6], [3, 7], [4, 5],
    [4, 7], [5, 6], [6, 7],
    [0, 5], [1, 4],
]

def boxes3d_to_corners3d_lidar(boxes3d, bottom_center=False):
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
    x_corners = np.array([dx / 2., dx / 2., -dx / 2., -dx / 2., 
                            dx / 2., dx / 2., -dx / 2., -dx / 2.], dtype=np.float32).T
    y_corners = np.array([dy / 2., -dy / 2., -dy / 2., dy / 2., 
                            dy / 2., -dy / 2., -dy / 2., dy / 2.], dtype=np.float32).T
    z_corners = np.array([dz / 2., dz / 2., dz / 2., dz / 2.,  
                            -dz / 2.,  -dz / 2.,  -dz / 2.,  -dz / 2.], dtype=np.float32).T
    # x_corners = np.array([dx / 2., dx / 2., -dx / 2., -dx / 2., dx / 2., dx / 2., -dx / 2., -dx / 2.], dtype=np.float32).T
    # y_corners = np.array([dy / 2., -dy / 2., -dy / 2., dy / 2., dy / 2., -dy / 2., -dy / 2., dy / 2.], dtype=np.float32).T
    # if bottom_center:
    #     z_corners = np.array([0., 0., 0., 0., dz,  dz,  dz,  dz], dtype=np.float32).T
    # else:
    #     z_corners = np.array([dz / 2., dz / 2., dz / 2., dz / 2., -dz / 2.,  -dz / 2.,  -dz / 2.,  -dz / 2.], dtype=np.float32).T

    ry = boxes3d[:, 6]
    zeros, ones = np.zeros(ry.size, dtype=np.float32), np.ones(ry.size, dtype=np.float32)
    # ry = ry+np.pi/6
    # counter-clockwisely rotate the frame around z by an angle ry
    # note the transform is done by Vector x Matrix instead of Matrix x Vector, 
    # which means the Matrix need to be transposed when interpreted as a linear transform
    rot_list = np.array([[np.cos(ry), np.sin(ry), zeros],
                         [-np.sin(ry), np.cos(ry),  zeros],
                         [zeros,      zeros,        ones]])  # (3, 3, N)
    R_list = np.transpose(rot_list, (2, 0, 1))  # (N, 3, 3)

    temp_corners = np.concatenate((x_corners.reshape(-1, 8, 1), y_corners.reshape(-1, 8, 1),
                                   z_corners.reshape(-1, 8, 1)), axis=2)  # (N, 8, 3)

    rotated_corners = np.matmul(temp_corners, R_list)  # (N, 8, 3)
    x_corners, y_corners, z_corners = rotated_corners[:, :, 0], rotated_corners[:, :, 1], rotated_corners[:, :, 2]

    x_loc, y_loc, z_loc = boxes3d[:, 0], boxes3d[:, 1], boxes3d[:, 2]

    x = x_loc.reshape(-1, 1) + x_corners.reshape(-1, 8)
    y = y_loc.reshape(-1, 1) + y_corners.reshape(-1, 8)
    z = z_loc.reshape(-1, 1) + z_corners.reshape(-1, 8)

    corners = np.concatenate((x.reshape(-1, 8, 1), y.reshape(-1, 8, 1), z.reshape(-1, 8, 1)), axis=2)

    return corners.astype(np.float32)

# filter detected pointcloud/pedestrain within the desired distance
def filter_pointcloud_distance(in_cloud, dist=10., verbose=False):
    r_square_within = (in_cloud[:,0]**2 + in_cloud[:,1]**2) < dist**2
    out_cloud = in_cloud[r_square_within,:]
    if verbose:
        print("Filtered/Overall pts: {}/{}".format(np.shape(in_cloud)[0], np.shape(out_cloud)[0]))
    return out_cloud

def filter_detection_tracking_res(in_boxes, dist=10., verbose=False):
    """
    boxes3d: (N, 7) [x, y, z, dx, dy, dz, heading] in LiDAR coords
    """
    r_square_within = (in_boxes[:,0]**2 + in_boxes[:,1]**2) < dist**2
    out_boxes = in_boxes[r_square_within,:]
    if verbose:
        print("Filtered/Overall boxes: {}/{}".format(np.shape(in_boxes)[0], np.shape(out_boxes)[0]))
    return out_boxes


def load_camera_info(camera_param, fname='/home/crowdbot/Documents/yujie/crowdbot_tools/qolo/saved_view.pkl'):
    # ref: # https://github.com/pablospe/render_depthmap_example/blob/main/visualization.py
    with open(fname, 'rb') as pickle_file:
        saved_view = load(pickle_file)
    camera_param.extrinsic = saved_view['extrinsic']
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(saved_view['width'], saved_view['height'], 
                            saved_view['intrinsic'][0][0], saved_view['intrinsic'][1][1], 
                            saved_view['intrinsic'][0][2], saved_view['intrinsic'][1][2])
    camera_param.intrinsic = intrinsic
    return camera_param

#%% Plot in qolo frame with Open3D
def plot_robot_frame_o3d(lidar, boxes=None, out_path=None, width=1080, height=720, show_xyz=True, filtering=False, filter_dist=8.0, verbose=False):
    
    # some nice colors
    gs_blue = (66.0 / 256, 133.0 / 256, 244.0 / 256)
    gs_red = (234.0 / 256, 68.0 / 256, 52.0 / 256)
    gs_yellow = (251.0 / 256, 188.0 / 256, 4.0 / 256)
    gs_green = (52.0 / 256, 168.0 / 256, 83.0 / 256)
    gs_orange = (255.0 / 256, 109.0 / 256, 1.0 / 256)
    gs_blue_light = (70.0 / 256, 189.0 / 256, 196.0 / 256)

    show_list = []
    origin_xyz = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=np.array([0, 0, 0]))
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
    if len(boxes):
        if filtering:
            boxes = filter_detection_tracking_res(boxes, filter_dist, verbose)
        # TODO: plot detections
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
                colors = [c for _ in range(len(lines))]

                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(corners),
                    lines=o3d.utility.Vector2iVector(lines))
                line_set.colors = o3d.utility.Vector3dVector(colors)
                show_list.append(line_set)
                
                line_mesh = LineMesh(corners, lines, colors, radius=0.02)
                line_mesh_geoms = line_mesh.cylinder_segments
                # show_list.append([*line_mesh_geoms])
                show_list += line_mesh_geoms # list of TriangleMesh

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='crowdbot', width=width, height=height)
    # vis.create_window(visible = False)
    for item in show_list:
        vis.add_geometry(item)
    
    # ViewControl: http://www.open3d.org/docs/0.12.0/python_api/open3d.visualization.ViewControl.html
    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=90.)
    ctr.set_zoom(0.8) # will be overwritten by camera_param
    # ctr.set_lookat([-3, -3, -1])
    # set viewpoint
    # camera_param = ctr.convert_to_pinhole_camera_parameters()
    # camera_param = load_camera_info(camera_param)
    # ctr.convert_from_pinhole_camera_parameters(camera_param, allow_arbitrary=True)

    # RenderOption
    opt = vis.get_render_option()
    opt.background_color = np.asarray([1, 1, 1]) # b: [0, 0, 0]; w: [1, 1, 1]
    opt.point_size = float(2)
    # opt.line_width = float(8)
    
    if out_path is not None:
        vis.poll_events()
        vis.update_renderer()
        # https://github.com/isl-org/Open3D/issues/1095
        # o3d.io.write_image(os.path.join(out_path, "test.jpg"), img)
        # vis.capture_screen_image(out_path)
        image = vis.capture_screen_float_buffer(False)
        plt.imsave(out_path, np.asarray(image), dpi = 300)
    else:
        vis.run()

#%% Plot in world frame with Open3D
def plot_world_frame_o3d(lidar=None, pose=None, boxes=None, out_path=None, 
                        width=1080, height=720, show_xyz=False, show_origin=True, filtering=True, filter_dist=8.0, verbose=False):
    
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
        trans_curr = trans_list[-1,:]
        print(trans_curr)
        # scipy: (x, y, z, w) from https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html
        # open3d: (w, x, y, z) `Eigen::Quaterniond` from https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
        # https://github.com/isl-org/Open3D/blob/master/cpp/open3d/geometry/Geometry3D.cpp#L216
        # save quat format -> scipy: (x, y, z, w)
        # used quat format -> open3d: (w, x, y, z)
        """
        rot_quat_list[:,[0, 1, 2, 3]] = rot_quat_list[:,[1, 2, 3, 0]]
        rot_quat_curr = rot_quat_list[-1,:]

        # from scipy.spatial.transform import Rotation as R
        # scipy_r = R.from_quat(rot_quat_curr)
        # print(scipy_r.as_euler('zxy', degrees=True))
        """

        # TODO: compute relative rotation to frame1
        from scipy.spatial.transform import Rotation as R
        scipy_rots = R.from_quat(rot_quat_list)
        scipy_rots_aligned = scipy_rots.reduce(left=R.from_quat(rot_quat_list[0,:]).inv())
        rot_quat_list_aligned = scipy_rots_aligned.as_quat()
        # TODO: compute relative rotation to frame1
        rot_quat_list_aligned[:,[0, 1, 2, 3]] = rot_quat_list_aligned[:,[1, 2, 3, 0]]
        rot_quat_curr = rot_quat_list_aligned[-1,:]

        # robot_xyz = copy.deepcopy(origin_xyz)
        robot_xyz = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=np.array([0, 0, 0]))
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
        # TODO: plot detections
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
                colors = [c for _ in range(len(lines))]

                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(corners),
                    lines=o3d.utility.Vector2iVector(lines))
                line_set.colors = o3d.utility.Vector3dVector(colors)
                show_list.append(line_set)
                
                line_mesh = LineMesh(corners, lines, colors, radius=0.02)
                line_mesh_geoms = line_mesh.cylinder_segments
                # show_list.append([*line_mesh_geoms])
                show_list += line_mesh_geoms # list of TriangleMesh

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='crowdbot', width=width, height=height)
    # vis.create_window(visible = False)
    for item in show_list:
        if pose:
            vis.add_geometry(item
                            .rotate(rot, center=(0, 0, 0))
                            .translate(trans_curr))
        else:
            vis.add_geometry(item)

    ## TODO: visualize past trajectories (try to program in another way!)
    if pose:
        # for i, trans in enumerate(trans_list):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        mesh.compute_vertex_normals()
        print(np.shape(trans_list)[0])
        # mode 1: show past 50 frames traj
        # viz_trans_list = trans_list[-50:,:] # trans_list
        # mode 2: show past traj sparsely (every 20 frame)
        viz_trans_list = trans_list[0:-1:20,:]
        traj_list = [copy.deepcopy(mesh) for elem in range(len(viz_trans_list))]
        for trans, traj in zip(viz_trans_list, traj_list):
            vis.add_geometry(traj.translate(trans))

    origin_xyz = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=np.array([-1, -1, -1]))
    if show_origin:
        vis.add_geometry(origin_xyz)
    
    # ViewControl: http://www.open3d.org/docs/0.12.0/python_api/open3d.visualization.ViewControl.html
    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=90.)
    ctr.set_zoom(0.8) # will be overwritten by camera_param
    # ctr.set_lookat([-3, -3, -1])
    # set viewpoint
    camera_param = ctr.convert_to_pinhole_camera_parameters()
    camera_param = load_camera_info(camera_param)
    ctr.convert_from_pinhole_camera_parameters(camera_param, allow_arbitrary=True)

    # RenderOption
    opt = vis.get_render_option()
    opt.background_color = np.asarray([1, 1, 1]) # b: [0, 0, 0]; w: [1, 1, 1]
    opt.point_size = float(2)
    # opt.line_width = float(8)
    
    if out_path is not None:
        vis.poll_events()
        vis.update_renderer()
        # https://github.com/isl-org/Open3D/issues/1095
        # o3d.io.write_image(os.path.join(out_path, "test.jpg"), img)
        # vis.capture_screen_image(out_path)
        image = vis.capture_screen_float_buffer(False)
        plt.imsave(out_path, np.asarray(image), dpi = 300)
    else:
        vis.run()
