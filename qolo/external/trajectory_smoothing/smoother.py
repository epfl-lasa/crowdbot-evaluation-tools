## ======= Smoothing Functions =========##
#!/usr/bin/env python
import csaps # <-- UNCOMMENT THIS WHEN PYTHON 3 IS COMPATIBLE WITH ROS
import Vec3D
import Bezier_curves
import SplinesC0
import SplinesC1
import SplinesC2
import numpy as np
import math
import functions
import pandas

import bexp
from sklearn.pipeline import Pipeline
from sklearn.linear_model import LinearRegression

def Bezier(x_raw,y_raw,z_raw,time=[]):

    raw_points = []
    count = len(x_raw)
    for i in range(len(x_raw)):
        raw_points.append(Vec3D.Vec3D(x_raw[i],y_raw[i],z_raw[i]))

    curve = Bezier_curves.BezierCurve()
    for index in range(0,count):
        #print(raw_points[index].x)
        curve.append_point(raw_points[index])

    #Compute smoothed points
    c = curve.draw()

    #Smooth x,y,z coordinates for Visualisation
    x_smoothed, y_smoothed, z_smoothed = [], [], []

    for point in c:
        x_smoothed.append(point.x)
        y_smoothed.append(point.y)
        z_smoothed.append(point.z)
    #print(x_smoothed)

    return x_smoothed, y_smoothed, z_smoothed


def Bspline(x_raw,y_raw,z_raw,time=[]):

    raw_points = []
    count = len(x_raw)
    print(count)
    for i in range(count):
        raw_points.append(Vec3D.Vec3D(x_raw[i],y_raw[i],z_raw[i]))

    intervals = [1 for i in range(count-3)]
    curve = SplinesC2.SplineC2(3,intervals)
    for index in range(0,count):
        curve.append_deBoor_point(raw_points[index])

    #Compute smoothed points
    c = curve.draw()

    #Smooth x,y,z coordinates for Visualisation
    x_smoothed, y_smoothed, z_smoothed = [], [], []

    for point in c:
        x_smoothed.append(point.x)
        y_smoothed.append(point.y)
        z_smoothed.append(point.z)

    return x_smoothed, y_smoothed, z_smoothed


def CubicSplines_csaps(x_raw,y_raw,z_raw,time,smooth=None):
    x = x_raw
    y = y_raw
    z = z_raw
    count = len(x)
    w = []
    for i in range(count):
        if i==0 or i==count-1:
            w.append(count*10)
        else:
            w.append(1)

    data = np.vstack((time,x, y, z))
    if smooth==None:
        smooth = None
    #print(smooth)
    case = 1
    if case == 0:
        sp_theta = csaps.MultivariateCubicSmoothingSpline(data, weights=w, smooth=smooth)
        theta_i = np.linspace(sp_theta.t[0],sp_theta.t[-1], 100)
        data_i = sp_theta(theta_i)

        x_smoothed = data_i[1, :]
        y_smoothed = data_i[2, :]
        z_smoothed = data_i[3, :]
    else:
        sx = csaps.UnivariateCubicSmoothingSpline(time, x,weights=w, smooth=smooth)
        sy = csaps.UnivariateCubicSmoothingSpline(time, y,weights=w, smooth=smooth)
        sz = csaps.UnivariateCubicSmoothingSpline(time, z,weights=w, smooth=smooth)

        theta_i = np.linspace(time[0],time[-1], 100)

        x_smoothed = sx(theta_i)
        y_smoothed = sy(theta_i)
        z_smoothed = sz(theta_i)

    return x_smoothed, y_smoothed, z_smoothed,smooth

def Natural_CubicSplines_sklearn(x_raw,y_raw,z_raw,time,n_knots=6):
    x = np.array(x_raw)
    y = np.array(y_raw)
    z = np.array(z_raw)
    time = np.array(time)

    curve = bexp.NaturalCubicSpline(max=min(time), min=max(time), n_knots=n_knots)

    p = Pipeline([
        ('nat_cubic', curve),
        ('regression', LinearRegression(fit_intercept=True))
    ])
    p.fit(time, x)
    x_smoothed = p.predict(time)
    p.fit(time, y)
    y_smoothed = p.predict(time)
    p.fit(time, z)
    z_smoothed = p.predict(time)
    return x_smoothed, y_smoothed, z_smoothed

def CubicSplines_sklearn(x_raw,y_raw,z_raw,time,n_knots=6):
    x = np.array(x_raw)
    y = np.array(y_raw)
    z = np.array(z_raw)
    time = np.array(time)

    curve = bexp.CubicSpline(max=min(time), min=max(time), n_knots=n_knots)

    p = Pipeline([
        ('cubic', curve),
        ('regression', LinearRegression(fit_intercept=True))
    ])
    p.fit(time, x)
    x_smoothed = p.predict(time)
    p.fit(time, y)
    y_smoothed = p.predict(time)
    p.fit(time, z)
    z_smoothed = p.predict(time)
    return x_smoothed, y_smoothed, z_smoothed

def Polynomial_sklearn(x_raw,y_raw,z_raw,time,degree=6):
    x = np.array(x_raw)
    y = np.array(y_raw)
    z = np.array(z_raw)
    time = np.array(time)

    curve = bexp.Polynomial(degree=degree)

    p = Pipeline([
        ('cubic', curve),
        ('regression', LinearRegression(fit_intercept=True))
    ])
    p.fit(time, x)
    x_smoothed = p.predict(time)
    p.fit(time, y)
    y_smoothed = p.predict(time)
    p.fit(time, z)
    z_smoothed = p.predict(time)
    return x_smoothed, y_smoothed, z_smoothed