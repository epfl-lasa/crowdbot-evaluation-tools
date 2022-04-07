#!/usr/bin/env python
import smoother
import numpy as np
import math
import functions
from timeit import default_timer as timer
from constants import *

def main(x,y,z,t,down=[],p=[],case=0):

	outliers,filtered = [], []
	x_raw,y_raw,z_raw,t_raw = x[:],y[:],z[:],t[:]
	if down == []:
		down = DOWNSAMPLING
	if p == []:
		p = 0.8
	if down:
		x_outliers,y_outliers,z_outliers,t_outliers = functions.exclude_outliers(x,y,z,t)
		outliers = [x_outliers,y_outliers,z_outliers]
		x,y,z,t = x_outliers[:],y_outliers[:],z_outliers[:],t_outliers[:]


	if case == 0:
		start = timer()
		x_smoothed,y_smoothed,z_smoothed = smoother.Bezier(x,y,z)
		end = timer()
		time_elapsed = end - start
	elif case == 1:
		start = timer()
		x_smoothed,y_smoothed,z_smoothed = smoother.Bspline(x,y,z)
		end = timer()
		time_elapsed = end - start
	elif case == 2:
		start = timer()
		x_smoothed,y_smoothed,z_smoothed, p = smoother.CubicSplines_csaps(x,y,z,t,SMOOTHING_FACTOR)
		end = timer()
		time_elapsed = end - start
	elif case == 3:
		start = timer()
		x_smoothed,y_smoothed,z_smoothed = smoother.CubicSplines_sklearn(x,y,z,t,1)
		end = timer()
		time_elapsed = end - start
	elif case == 4:
		start = timer()
		x_smoothed,y_smoothed,z_smoothed = smoother.Natural_CubicSplines_sklearn(x,y,z,t,6)
		end = timer()
		time_elapsed = end - start
	else:
		start = timer()
		x_smoothed,y_smoothed,z_smoothed = smoother.Polynomial_sklearn(x,y,z,t,2)
		end = timer()
		time_elapsed = end - start

	print("Time Elapsed for Smoothing: %.4f seconds." %time_elapsed)

	#functions.print2D([x_raw,y_raw,z_raw],[x_smoothed,y_smoothed,z_smoothed],SMOOTHING_FACTOR,case,outliers,filtered)
	#functions.print3D([x_raw,y_raw,z_raw],[x_smoothed,y_smoothed,z_smoothed])

	return x_smoothed,y_smoothed,z_smoothed


if __name__ == '__main__':
	main()
