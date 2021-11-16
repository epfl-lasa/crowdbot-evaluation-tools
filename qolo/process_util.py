# -*-coding:utf-8 -*-
"""
@File    :   process_util.py
@Time    :   2021/11/09
@Author  :   Yujie He
@Version :   1.1
@Contact :   yujie.he@epfl.ch
@State   :   Dev
"""

import numpy as np

import scipy.signal as signal
from scipy import interpolate
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R

# https://docs.scipy.org/doc/scipy/reference/spatial.transform.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.savgol_filter.html


def ts_to_sec(ts):
    """convert ros timestamp into second"""
    return ts.secs + ts.nsecs / float(1e9)


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
        # data[data > thres] = 0
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
