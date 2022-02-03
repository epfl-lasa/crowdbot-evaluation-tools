#!/usr/bin/env python3
# -*-coding:utf-8 -*-
# =============================================================================
"""
@Author        :   Yujie He
@File          :   process_util.py
@Date created  :   2021/11/09
@Maintainer    :   Yujie He
@Email         :   yujie.he@epfl.ch
"""
# =============================================================================
"""
The module provides functions to convert timestamp into second, interpolate
data, and compute derivatives.
"""
# =============================================================================


import numpy as np

import scipy.signal as signal
from scipy import interpolate
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R


def ts_to_sec(ts):
    """convert ros timestamp into second"""
    return ts.secs + ts.nsecs / float(1e9)


def ts_to_sec_str(ts):
    """convert ros timestamp into second"""
    # return "{}.{:9d}".format(ts.secs, ts.nsecs)
    sec = ts.secs + ts.nsecs / float(1e9)
    return "{:.9f}".format(sec)


# https://docs.scipy.org/doc/scipy/reference/spatial.transform.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html


def interp_rotation(source_ts, interp_ts, source_ori):
    """Apply slerp interpolatation to list of rotation variable"""

    slerp = Slerp(source_ts, R.from_quat(source_ori))
    interp_ori = slerp(interp_ts)
    return interp_ori.as_quat()


def interp_translation(source_ts, interp_ts, source_trans):
    """Apply linear interpolatation to list of translation variable"""

    # The length of y along the interpolation axis must be equal to the length of x.
    # source_ts (N,) source_trans (...,N)
    f = interpolate.interp1d(source_ts, np.transpose(source_trans))
    interp_pos = f(interp_ts)
    return np.transpose(interp_pos)


def compute_motion_derivative(motion_stamped_dict, subset=None):
    """Compute derivative for robot state, such as vel, acc, or jerk"""

    ts = motion_stamped_dict.get("timestamp")
    dval_dt_dict = {"timestamp": ts}

    if subset == None:
        subset = motion_stamped_dict.keys()
    for val in subset:
        if val == "timestamp":
            pass
        else:
            dval_dt = np.gradient(motion_stamped_dict.get(val), ts, axis=0)
            dval_dt_dict.update({val: dval_dt})
    return dval_dt_dict


def smooth1d(
    data, filter='savgol', window=9, polyorder=1, check_thres=False, thres=[-1.2, 1.5]
):
    """
    Smooth datapoints with Savitzky-Golay or moving-average

    filter:
        Type of filter to use when smoothing the velocities.
        Default is Savitzky-Golay, which fits a polynomial of order 'polyorder' to the data within each window
    window:
        Smoothing window size in # of frames
    polyorder:
        Order of the polynomial for the Savitzky-Golay filter.
    thres:
        Speed threshold of qolo.
    """

    if check_thres:
        # method1: zero
        # data[data < thres[0]] = 0
        # data[data > thres[1]] = 0
        # method2: assign valid value that not exceeds threshold to the noisy datapoints
        curr_nearest_valid = 0
        for idx in range(1, len(data)):
            if thres[0] < data[idx] < thres[1]:
                curr_nearest_valid = idx
            else:
                data[idx] = data[curr_nearest_valid]

    if filter == 'savgol':
        data_smoothed = signal.savgol_filter(
            data, window_length=window, polyorder=polyorder
        )
    elif filter == 'moving_average':
        ma_window = np.ones(window) / window
        data_smoothed = np.convolve(data, ma_window, mode='same')
    return data_smoothed


def smooth(
    nd_data,
    filter='savgol',
    window=9,
    polyorder=1,
    check_thres=False,
    thres=[-1.2, 1.5],
):
    """
    Smooth multi-dimension datapoints with Savitzky-Golay or moving-average

    filter:
        Type of filter to use when smoothing the velocities.
        Default is Savitzky-Golay, which fits a polynomial of order 'polyorder' to the data within each window
    window:
        Smoothing window size in # of frames
    polyorder:
        Order of the polynomial for the Savitzky-Golay filter.
    thres:
        Speed threshold of qolo.
    """

    nd_data_smoothed = np.zeros_like(nd_data)
    for dim in range(np.shape(nd_data)[1]):
        nd_data_smoothed[:, dim] = smooth1d(
            nd_data[:, dim],
            filter=filter,
            window=window,
            polyorder=polyorder,
            check_thres=check_thres,
            thres=thres,
        )
    return nd_data_smoothed


def strict_increase(ts, value=1e-5):
    """
    Add small values to ensure list strictly in increasing order
    `np.finfo(float).eps` is not recommended
    also not less than 1e-6
    """
    ts_delta = np.diff(ts)
    # find the index with zero variation
    res_zero_idx = np.where(ts_delta == 0)[0].tolist()
    res_neg_idx = np.where(ts_delta < 0)[0].tolist()
    # add small variation to last index
    for idx in sorted(res_zero_idx + res_neg_idx):
        ts[idx + 1] = ts[idx] + value
    return ts


def check_zero_diff(src_list):
    """commpute zero-difference elements in the list for debugging"""

    total_len = src_list.shape[0]
    diff = np.diff(src_list)
    zero_diff = diff[diff == 0.0]
    zero_diff_len = zero_diff.shape[0]

    print("Zero_diff: {}/{}".format(zero_diff_len, total_len))


def get_xyzrgb_points(cloud_array, remove_nans=True, dtype=np.float):
    """Convert ju

    Args:
        cloud_array ([type]): [description]
        remove_nans (bool, optional): [description]. Defaults to True.
        dtype ([type], optional): [description]. Defaults to np.float.

    Returns:
        [type]: [description]
    """

    # remove crap points
    if remove_nans:
        mask = (
            np.isfinite(cloud_array['x'])
            & np.isfinite(cloud_array['y'])
            & np.isfinite(cloud_array['z'])
            & np.isfinite(cloud_array['r'])
            & np.isfinite(cloud_array['g'])
            & np.isfinite(cloud_array['b'])
        )
        cloud_array = cloud_array[mask]

    # pull out x, y, z, r, g, and b values
    xyz = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    rgb = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    xyz[..., 0] = cloud_array['x']
    xyz[..., 1] = cloud_array['y']
    xyz[..., 2] = cloud_array['z']
    rgb[..., 0] = cloud_array['r']
    rgb[..., 1] = cloud_array['g']
    rgb[..., 2] = cloud_array['b']

    return xyz, rgb
