# -*-coding:utf-8 -*-
'''
@File    :   process_util.py
@Time    :   2021/11/09
@Author  :   Yujie He
@Version :   1.1
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy import interpolate
import numpy as np

def interp_rotation(source_ts, interp_ts, source_ori):
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html

    slerp = Slerp(source_ts, R.from_quat(source_ori))
    interp_ori = slerp(interp_ts)
    return interp_ori.as_quat()

def interp_translation(source_ts, interp_ts, source_trans):
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html#scipy.interpolate.interp1d
    # The length of y along the interpolation axis must be equal to the length of x.
    # source_ts (N,) source_trans (...,N)
    f = interpolate.interp1d(source_ts, np.transpose(source_trans))
    interp_pos = f(interp_ts)
    return np.transpose(interp_pos)

# motion_stamped_dict can be pose, twist, or acc
def compute_motion_derivative(motion_stamped_dict, subset=None):
    """
    source_x = motion_stamped_dict.get('x')
    source_zrot = motion_stamped_dict.get('zrot')
    dx_dt = np.gradient(source_x, ts, axis=0)
    dzrot_dt = np.gradient(source_zrot, ts, axis=0)
    dval_dt_dict = {'timestamp': ts, 
                        'x': dx_dt, 
                        'zrot': dzrot_dt}
    """

    ts = motion_stamped_dict.get('timestamp')
    dval_dt_dict = {'timestamp': ts}

    if subset==None:
        subset = motion_stamped_dict.keys()
    for val in subset:
        if val == 'timestamp':
            pass
        else:
            dval_dt = np.gradient(motion_stamped_dict.get(val), ts, axis=0)
            dval_dt_dict.update({val: dval_dt})
    print('Current motion extracted!')
    return dval_dt_dict
