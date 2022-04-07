#!/usr/bin/env python
from trajectory_smoothing.srv import *
from trajectory_smoothing_msg.msg import OpenPoseReceiverHuman
import rospy

import os
import re
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import trajectory_smoothing_function
import sys
import math

import numpy as np
import functions

from scipy.spatial import distance


def handle_smoothing(req):
	x = req.x
	y = req.y
	z = req.z
	t = req.t
	print(t)
	x_1,y_1,z_1 = trajectory_smoothing_function.main(x,y,z,t)

	x_smooth,y_smooth,z_smooth = [x_1[0]],[y_1[0]],[z_1[0]]

	a = (x_1[0],y_1[0],z_1[0])
	for i in range(len(x_1)):
		b = (x_1[i],y_1[i],z_1[i])
		dst = distance.euclidean(a, b)
		
		if dst > 0.0055:
			#print(dst)
			x_smooth.append(x_1[i])
			y_smooth.append(y_1[i])
			z_smooth.append(z_1[i])
			a = (x_1[i],y_1[i],z_1[i])

	# functions.print2D([x,y,z],[x_smooth,y_smooth,z_smooth])
	functions.print3D([x,y,z],[x_smooth,y_smooth,z_smooth])
	return SmoothResponse(x_smooth,y_smooth,z_smooth)


def smooth_server():
	rospy.init_node('smooth_server')
	s = rospy.Service('smoothing',Smooth,handle_smoothing)
	print("Ready to smooth trajectory")
	rospy.spin()

if __name__ == '__main__':
	smooth_server()