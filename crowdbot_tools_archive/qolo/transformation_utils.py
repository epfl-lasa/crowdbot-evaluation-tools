import cv2
import numpy as np

"""
Transformations. Following frames are defined:

base: main frame where 3D annotations are done in, x-forward, y-left, z-up
upper_lidar: x-forward, y-left, z-up
lower_lidar: x-forward, y-left, z-up
laser: x-forward, y-left, z-up

Ref: http://download.cs.stanford.edu/downloads/jrdb/Sensor_setup_JRDB.pdf
"""


def _get_R_z(rot_z):
    cs, ss = np.cos(rot_z), np.sin(rot_z)
    return np.array([[cs, -ss, 0], [ss, cs, 0], [0, 0, 1]], dtype=np.float32)


# laser to base
_rot_z_laser_to_base = np.pi / 120
_R_laser_to_base = _get_R_z(_rot_z_laser_to_base)

# upper_lidar to base
_rot_z_upper_lidar_to_base = 0.085
_T_upper_lidar_to_base = np.array([0, 0, 0.33529], dtype=np.float32).reshape(3, 1)
_R_upper_lidar_to_base = _get_R_z(_rot_z_upper_lidar_to_base)

# lower_lidar to base
_rot_z_lower_lidar_to_base = 0.0
_T_lower_lidar_to_base = np.array([0, 0, -0.13511], dtype=np.float32).reshape(3, 1)
_R_lower_lidar_to_base = np.eye(3, dtype=np.float32)


"""
Transformation API
"""


def transform_pts_upper_velodyne_to_base(pts):
    """Transform points from upper velodyne frame to base frame

    Args:
        pts (np.array[3, N]): points (x, y, z)

    Returns:
        pts_trans (np.array[3, N])
    """
    return _R_upper_lidar_to_base @ pts + _T_upper_lidar_to_base


def transform_pts_lower_velodyne_to_base(pts):
    return _R_lower_lidar_to_base @ pts + _T_lower_lidar_to_base


def transform_pts_laser_to_base(pts):
    return _R_laser_to_base @ pts


def transform_pts_base_to_upper_velodyne(pts):
    return _R_upper_lidar_to_base.T @ (pts - _T_upper_lidar_to_base)


def transform_pts_base_to_lower_velodyne(pts):
    return _R_lower_lidar_to_base.T @ (pts - _T_lower_lidar_to_base)


def transform_pts_base_to_laser(pts):
    return _R_laser_to_base.T @ pts


def transform_pts_base_to_stitched_im(pts):
    """Project 3D points in base frame to the stitched image

    Args:
        pts (np.array[3, N]): points (x, y, z)

    Returns:
        pts_im (np.array[2, N])
        inbound_mask (np.array[N])
    """
    im_size = (480, 3760)

    # to image coordinate
    pts_rect = pts[[1, 2, 0], :]
    pts_rect[:2, :] *= -1

    # to pixel
    horizontal_theta = np.arctan2(pts_rect[0], pts_rect[2])
    horizontal_percent = horizontal_theta / (2 * np.pi) + 0.5
    x = im_size[1] * horizontal_percent
    y = (
        485.78 * pts_rect[1] / pts_rect[2] * np.cos(horizontal_theta)
        + 0.4375 * im_size[0]
    )
    # horizontal_theta = np.arctan(pts_rect[0, :] / pts_rect[2, :])
    # horizontal_theta += (pts_rect[2, :] < 0) * np.pi
    # horizontal_percent = horizontal_theta / (2 * np.pi)
    # x = ((horizontal_percent * im_size[1]) + 1880) % im_size[1]
    # y = (
    #     485.78 * (pts_rect[1, :] / ((1 / np.cos(horizontal_theta)) * pts_rect[2, :]))
    # ) + (0.4375 * im_size[0])

    # x is always in bound by cylindrical parametrization
    # y is always at the lower half of the image, since laser is lower than the camera
    # thus only one boundary needs to be checked

    inbound_mask = np.logical_and(y >= 0, y < im_size[0])

    return np.stack((x, y), axis=0).astype(np.int32), inbound_mask


def transform_pts_laser_to_stitched_im(pts):
    pts_base = transform_pts_laser_to_base(pts)
    return transform_pts_base_to_stitched_im(pts_base)


def transform_pts_base_to_cam(pts, R, T):
    pts = pts[[1, 2, 0], :]
    pts[:2, :] *= -1
    pts = R @ pts + T.reshape(3, 1)

    return pts


def transform_pts_base_to_im(pts, R, T, K, width=752, height=480):
    """Project 3D points in base frame to individual image

    Args:
        pts (np.array[3, N]): points (x, y, z)

    Returns:
        pts_im (np.array[2, N])
        inbound_mask (np.array[N])
    """
    pts = transform_pts_base_to_cam(pts, R, T)  # cam coordinate
    in_mask = pts[2] > 0  # points in front of camera
    pts /= pts[2]  # homogenuous coordinate
    pts = (K @ pts).astype(np.int)  # pixel coordinate

    in_mask = np.logical_and(in_mask, pts[0] >= 0)
    in_mask = np.logical_and(in_mask, pts[0] < width)
    in_mask = np.logical_and(in_mask, pts[1] >= 0)
    in_mask = np.logical_and(in_mask, pts[1] < height)

    pts = pts[:2]

    return pts.astype(np.int32), in_mask


def pts_to_rgb(
    pts, dist_range=(0.0, 10.0), close_hsv=(0.0, 1.0, 1.0), far_hsv=(0.0, 0.0, 1.0)
):
    """For plotting, get bgr color based on distance

    Args:
        pts (np.array[3, N]): points (x, y, z)

    Returns:
        c_rgb (np.array[N, 3])
    """
    dist = np.linalg.norm(pts, axis=0)

    dist_normalized = (
        np.clip(dist, dist_range[0], dist_range[1]) / dist_range[1]
    ).reshape(-1, 1)

    c_hsv = (
        np.array(close_hsv).reshape(1, -1) * (1.0 - dist_normalized)
        + np.array(far_hsv).reshape(1, -1) * dist_normalized
    ).astype(np.float32)
    c_hsv = c_hsv[None, ...]
    c_rgb = cv2.cvtColor(c_hsv, cv2.COLOR_HSV2RGB)

    return c_rgb[0]


"""
Polar and cartesian conversion API

Convention: Cartesian is defined as x-forward (center of the scan), y-right, z-downward,
phi is the angle w.r.t. x axis
"""


def polar_xy_to_rphi(x, y):
    return np.hypot(x, y), np.arctan2(y, x)


def polar_rphi_to_xy(r, phi):
    return r * np.cos(phi), r * np.sin(phi)


"""
Boxes3D API

Boxes should be numpy.array[B, 7] with (x, y, z, l, w, h, theta)

Boxes are given in JRDB base frame: x-forward, y-left, z-up
theta is the angle between x-axis and the length side of bounding boxes
"""


def _convert_boxes_kitti_to_jrdb(kitti_boxes):
    """Convert KITTI boxes to JRDB boxes

    x = z'
    y = -x'
    z = -y' + h' / 2
    l = l'
    w = w'
    h = h'
    theta = -theta'

    Args:
        kitti_boxes (array[B, 7]): x', y', z', h', w', l', theta'

    Returns:
        jrdb_boxes (array[B, 7]): x, y, z, l, w, h, theta
    """
    jrdb_boxes = np.stack(
        [
            kitti_boxes[:, 2],
            -kitti_boxes[:, 0],
            -kitti_boxes[:, 1] + 0.5 * kitti_boxes[:, 3],
            kitti_boxes[:, 5],
            kitti_boxes[:, 4],
            kitti_boxes[:, 3],
            -kitti_boxes[:, 6],
        ],
        axis=1,
    )

    return jrdb_boxes


def _convert_boxes_jrdb_to_kitti(jrdb_boxes):
    """Convert JRDB boxes to KITTI boxes

    x' = -y
    y' = -z + h / 2
    z' = x
    h' = h
    w' = w
    l' = l
    theta' = -theta

    Args:
        jrdb_boxes (array[B, 7]): x, y, z, l, w, h, theta

    Returns:
        kitti_boxes (array[B, 7]): x', y', z', h', w', l', theta'
    """
    # NOTE: Use test/test_iou3d() to check that this conversion is done correctly.
    kitti_boxes = np.stack(
        [
            -jrdb_boxes[:, 1],
            -jrdb_boxes[:, 2] + 0.5 * jrdb_boxes[:, 5],
            jrdb_boxes[:, 0],
            jrdb_boxes[:, 5],
            jrdb_boxes[:, 4],
            jrdb_boxes[:, 3],
            -jrdb_boxes[:, 6],
        ],
        axis=1,
    )

    return kitti_boxes


def boxes_to_string(boxes, scores, jrdb_format=True):
    """Obtain a KITTI format string for storing detection result

    Args:
        boxes (array[B, 7])
        scores (array[B])
        jrdb_format (bool): True if the input boxes is in JRDB convention, False
            if in KITTI convention

    Returns:
        s (str)
    """
    if jrdb_format:
        boxes = _convert_boxes_jrdb_to_kitti(boxes)

    s = ""
    for box, score in zip(boxes, scores):
        s += (
            f"Pedestrian 0 0 -1 0 -1 -1 -1 -1 {box[3]} {box[4]} {box[5]} "
            f"{box[0]} {box[1]} {box[2]} {box[6]} {score}\n"
        )
    s = s.strip("\n")

    return s


def boxes_from_string(s, jrdb_format=True, get_num_points=False):
    """Convert a KITTI format string to boxes and detection scores

    Args:
        s (str)
        jrdb_format (bool): True if the output boxes should be in JRDB convention,
            False if in KITTI convention
        get_num_points (bool): True to also return number of points. Useful
            for loading annotations

    Returns:
        boxes (array[B, 7])
        scores (array[B])
    """
    boxes = []
    scores = []

    lines = s.split("\n")
    for line in lines:
        if len(line) == 0:
            continue
        v_list = [float(v) for v in line.split()[-8:]]
        scores.append(v_list[-1])
        boxes.append([v_list[i] for i in [3, 4, 5, 0, 1, 2, 6]])

    boxes = np.array(boxes, dtype=np.float32)
    scores = np.array(scores, dtype=np.float32)

    if jrdb_format and len(boxes) > 0:
        boxes = _convert_boxes_kitti_to_jrdb(boxes)

    if get_num_points:
        num_points = np.array([int(line.split()[3]) for line in lines if len(line) > 0])
        return boxes, scores, num_points
    else:
        return boxes, scores


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
