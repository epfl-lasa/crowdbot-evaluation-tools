#!/usr/bin/env python
import rospy

from trajectory_smoothing.srv import *
from trajectory_smoothing_msg.msg import Keypoint3d_list,SmoothRWristCoordsWithRespectToBase
import os
import re
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys
import math

import numpy as np
import functions
import collections 

from geometry_msgs.msg import Point

#lists that will contain the points for smoothing 
x, y, z,t, prob = [], [], [], [], []

#Queues used as the local and overall lists of the motion-detection-algorithm
xde = collections.deque([])
yde = collections.deque([])
zde = collections.deque([])
Vde = collections.deque([])
tde = collections.deque([])

allxde = collections.deque([])
allyde = collections.deque([])
allzde = collections.deque([])
alltde = collections.deque([])

#the smooth_flag and start_flag of the motion-detection-algorithm
smooth = False
start = 0
count = 0

#flag for pushing previous points when the motion is activated
push = True

#flags that chech if we havent receiced any points for over 3 sec and reset the *de queues.
lastcall = 0
restart = False


#useless variable from previous version
max_speed = 0
#useless flags, delete them
flag = 0
stop = False
first_act = 0
#probably useless lists because each time i use only the last point of all the lists
stdx,stdy,stdz = [],[],[]
allstdx,allstdy,allstdz = [],[],[]


def velocity(x1,x0,y1,y0,z1,z0,t1,t0):
	Vx = (x1 - x0)/(t1- t0)
	Vy = (y1 - y0)/(t1 - t0)
	Vz = (z1 - z0)/(t1 - t0)
	V = np.sqrt(Vx**2 + Vy**2 + Vz**2)
	return V

def callback(data):
	global x,y,z,t,smooth,x_smooth,y_smooth,z_smooth,max_speed,i,count,flag,start,lastcall,xde,yde,zde,tde,stop,first_act,allxde,allyde,allzde,alltde,push,stdx,stdy,stdz,allstdx,allstdy,allstdz,restart
	if (not smooth):
		
		for i in range(len(data.keypoints)):
			if (data.keypoints[i].name == "RWrist"):
				tmp_x = data.keypoints[i].points.point.x
				tmp_y = data.keypoints[i].points.point.y
				tmp_z = data.keypoints[i].points.point.z
				seconds = rospy.get_time()
				continue

		#check if listening to points should restart(timeout = 3secs)
		if seconds - lastcall > 3 or restart:
			if restart:
				print("Now listening to NEW points...")
			else:
				print("Listened to nothing for 3 secs")
			restart = False
			x, y, z,t = [], [], [], []
			smooth = False
			max_speed = 0
			i = -1
			start = 0
			count = 0
			flag = 0
			first_act = 0
			push = True
			tde = collections.deque([])
			xde = collections.deque([])
			yde = collections.deque([])
			zde = collections.deque([])
			alltde = collections.deque([])
			allxde = collections.deque([])
			allyde = collections.deque([])
			allzde = collections.deque([])
			stdx,stdy,stdz = [],[],[]
			allstdx,allstdy,allstdz = [],[],[]
			lastcall = seconds

		if tmp_x != 0 or tmp_y != 0 or tmp_z != 0:
			print("Listened to VALID point")

			i += 1
			if abs(tmp_x) < .6 and abs(tmp_y) < .6 and abs(tmp_z) < .6:
				#store points to local and overall lists
				if len(xde) == 6:
					xde.pop()
				if len(yde) == 6:
					yde.pop()
				if len(zde) == 6:
					zde.pop()
				if len(tde) == 6:
					tde.pop()
				tde.appendleft(seconds)
				xde.appendleft(tmp_x)
				yde.appendleft(tmp_y)
				zde.appendleft(tmp_z)
				alltde.appendleft(seconds)
				allxde.appendleft(tmp_x)
				allyde.appendleft(tmp_y)
				allzde.appendleft(tmp_z)


				if len(xde) >= 2:

					stdx.append(np.std(xde))
					stdy.append(np.std(yde))
					stdz.append(np.std(zde))
					allstdx.append(np.std(allxde))
					allstdy.append(np.std(allyde))
					allstdz.append(np.std(allzde))
					print("====")
					print("Std X = %f" %float(stdx[-1]))
					print("Std Y = %f" %float(stdy[-1]))
					print("Std Z = %f" %float(stdz[-1]))
					print("---")
					#check if start_flag should be set
					if start == 0 and (stdx[-1] > 0.01 or stdy[-1] > 0.01 or stdz[-1] > 0.01):
						#step 5 of the motion detection algorithm
						if first_act == 0:
							first_act = i
						if stdz[-1] > allstdy[-1] or stdx[-1] > allstdy[-1]:
							start = 1
							pushback = 4
							jstart = i
							print("<========STARTING MOTION========>")
						elif stdy[-1] > 0.07:
							start = 1
							pushback = 8
							jstart = i
							print("<========STARTING MOTION========>")

					if start == 1:

						if push == True:
							#when them start_flag is set for the first time push the previous points
							#this is done only once
							for k in range(1,pushback):
								x.append(allxde[pushback-k-1])
								y.append(allyde[pushback-k-1])
								z.append(allzde[pushback-k-1])
								t.append(alltde[pushback-k-1])
							push = False

						#append the current point
						x.append(allxde[0])
						y.append(allyde[0])
						z.append(allzde[0])
						t.append(alltde[0])

						#check the ending condition
						if stdx[-1] > 0.01 or stdy[-1] > 0.01 or stdz[-1] > 0.01:
							count = 0
						else:
							count += 1
			if count >= 1:
				lastcall = rospy.get_time()
				restart = True
				print("<========MOTION ENDED========>")
				print("Stopped moving at sample %d" %len(x))
				smooth = True
			else:
				lastcall = rospy.get_time()
				smooth = False


def smoother_node():
	global smooth,x,y,z,t,i,count,flag,start,max_speed
	rospy.init_node('smoother_node')
	#rospy.set_param('robot_frame_coords_msg',"/openpose_ros_receiver/robot_frame_coords_msg")
	global sub_handler
	sub_handler = rospy.Subscriber('/openpose_ros_receiver/robot_frame_coords_msg', Keypoint3d_list, callback)
	pub = rospy.Publisher('smooth_robot_frame_coords_msg', SmoothRWristCoordsWithRespectToBase, queue_size=1)
	msg = SmoothRWristCoordsWithRespectToBase()
	rate = rospy.Rate(10.0)
	while(not rospy.is_shutdown()):
		if smooth:
			#print("I WAS CALLED")
			rospy.loginfo("WAITING")
			rospy.wait_for_service('smoothing')
			duration = t[-1] - t[0]
			time = np.linspace(0,duration,len(x))
			try:
				smoothing = rospy.ServiceProxy('smoothing',Smooth)
				resp = smoothing(x,y,z,time)
				# msg.x,msg.y,msg.z = resp.x_smooth, resp.y_smooth, resp.z_smooth
				temp = zip(resp.x_smooth, resp.y_smooth, resp.z_smooth)
				# print temp
				# input()
				for point in temp:
					point_msg = Point()
					point_msg.x = point[0]
					point_msg.y = point[1]
					point_msg.z = point[2]
					msg.points.append(point_msg)
			except rospy.ServiceException, e:
				print("Service call failed: %s"%e)
			print("sending smooth data...")
			
			# Here we publish the smoothed coords
			#while not rospy.is_shutdown():
			pub.publish(msg)
			msg.points = []
			#rospy.sleep(5)
			print("Ready to smooth NEW trajectory")
			smooth = False
		rate.sleep()
			
	print("main done")
	rospy.spin()

if __name__ == '__main__':
	smoother_node()
