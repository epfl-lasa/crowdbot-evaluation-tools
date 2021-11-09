# -*-coding:utf-8 -*-
'''
@File    :   interp_util.py
@Time    :   2021/11/09
@Author  :   Yujie He
@Version :   1.0
@Contact :   yujie.he@epfl.ch
@State   :   Dev
'''
# -*-coding:utf-8 -*-
'''
@File    :   interp_util.py
@Time    :   2021/11/09
@Author  :   Yujie He
@Version :   1.0
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
