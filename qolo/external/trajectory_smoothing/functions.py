#!/usr/bin/env python
import sys
import math
import numpy as np
import ast
from scipy.signal import butter, lfilter, freqz, lfilter_zi, filtfilt
#from sympy import Derivative
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer

import smoother
import Vec3D
import Bezier_curves

def read_files(files):
	num_files = len(files)
	raw_coord_list = []
	time_list = []
	for file in files:
		if 'x_' in file or 'y_' in file:
			x,y,z,t = read_BT_file(file)
		else:
			x,y,z,t = read_OP_file(file)
		raw_coord_list.append([x,y,z])
		time_list.append(t)
	return raw_coord_list,time_list


def read_OP_file(file):
	#OpenPose case
	tmp = open(file, "r")
	fl = tmp.readlines()
	x = []
	y = []
	z = []
	time = []
	for i in range(len(fl)):
		if 'Wrist' in fl[i]:
			tmp1 = fl[i-2].split()
			tmp2 = fl[i-1].split()
			t = float(tmp1[-1])+(0.000000001*float(tmp2[-1]))
			time.append(t)
			x.append(float(fl[i+2][4:len(fl[i])-1]))
			y.append(float(fl[i+3][4:len(fl[i])-1]))
			z.append(float(fl[i+4][4:len(fl[i])-1]))
	time = np.array(time) - time[0]
	return x,y,z,time

def read_BT_file(file):
	f = open(file, "r")
	fl = f.readlines()
	x = []
	y = []
	z = []
	time = []
	for i in range(len(fl)):
		if 'Frame' in fl[i]:
			if '[]' in fl[i+1]:
				continue
			temp = fl[i+1]
			if temp:
				temp = temp[3:]
				a = ast.literal_eval(temp)
				cox=[]
				coy=[]
				coz=[]
				for k in range(len(a)):
					cox.append(a[k][0][0])
					coy.append(a[k][0][1])
					coz.append(a[k][0][2])
				if cox[8] != 0.0:
					time.append(float(fl[i-1][41:56]))
					x.append(cox[8]/1000)
					y.append(coy[8]/1000)
					z.append(coz[8]/1000)
	if time != []:
		time = np.array(time) - time[0]
	method = "BT"
	return x,y,z,time

def print2D(raw_coords,smooth_coords,p=[],case=0,outliers=[],filtered=[]):

	f_info = 'Smoothed trajectory'
	fig,ax = plt.subplots()
	ax.set_aspect('equal')
	ax.grid('on')
	ax.scatter(np.array(raw_coords[0]),np.array(raw_coords[1]),alpha=0.3,label="Initial Points")
	if outliers != []:
		ax.scatter(np.array(outliers[0]),np.array(outliers[1]),alpha=0.3,label="Points after excluding outliers")
	if filtered != []:
		ax.scatter(np.array(filtered[0]),np.array(filtered[1]),alpha=0.3,label="Points after Filtering")
	if case == 0 or case == 1:
		if case == 0:
			plt.title('Single Bezier Smoothing')
		else:
			plt.title('Piecewise Bezier Smoothing')
		ax.scatter(np.array(smooth_coords[0]),
			np.array(smooth_coords[1]),linewidth=1.5,label='%s' %(f_info))
	elif case == 2:
		ax.plot(np.array(smooth_coords[0]),
			np.array(smooth_coords[1]),linewidth=1.5,label='%s, p = %.4f' %(f_info,p))
		plt.title('Cubic Spline Smoothing')
	ax.set_xlabel('X - axis (m)')
	ax.set_ylabel('Y - axis (m)')
	ax.legend()
	#manager = plt.get_current_fig_manager()
	#manager.window.showMaximized()
	plt.show()



def printDemo2D(raw_coord_list,smooth_coord_list,num_files,f_info,case=-1,scatter=True):
	f_info = ['line','line','line','line','line','line','line','line','line','line','line','line']
	p = [0,0,0,0]
	fig1, ax = plt.subplots()
	ax.scatter(np.array(raw_coord_list[0][0]),np.array(raw_coord_list[0][1]),alpha=0.3,label="Initial Points")
	if scatter:
		if num_files <= 2:
			for i in range(num_files):
				ax.scatter(np.array(raw_coord_list[i][0]),np.array(raw_coord_list[i][1]),alpha=0.3,label="Initial Points")
	ax.set_aspect('equal')
	ax.grid('on')
	print(case)
	if case == 0 or case == 1:
		if case == 0:
			plt.title('Single Bezier Smoothing')
		else:
			plt.title('Piecewise Bezier Smoothing')
		for i in range(num_files):
			ax.plot(np.array(smooth_coord_list[i][0]),
				np.array(smooth_coord_list[i][1]),linewidth=1.0,label='%s' %(f_info[i]))
	elif case == 2:
		for i in range(num_files):
			ax.plot(np.array(smooth_coord_list[i][0]),
				np.array(smooth_coord_list[i][1]),linewidth=1.0,label='%s' %(f_info[i]))
		plt.title('Cubic Spline Smoothing')
	else:
		for i in range(num_files):
			ax.plot(np.array(smooth_coord_list[i][0]),
				np.array(smooth_coord_list[i][1]))
		plt.title('Initial Trajectories WITHOUT smoothing')
	ax.set_xlabel('X - axis (m)')
	ax.set_ylabel('Y - axis (m)')
	ax.legend()
	manager = plt.get_current_fig_manager()
	manager.window.showMaximized()
	plt.show()

def print3D(raw_coord_list,smooth_coord_l):
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.plot(np.array(smooth_coord_l[0]),np.array(smooth_coord_l[1]), np.array(smooth_coord_l[2]))
	ax.scatter(np.array(raw_coord_list[0]),np.array(raw_coord_list[1]), np.array(raw_coord_list[2]))
	ax.plot([0,0],[-0.5,.5],color='dimgrey')
	ax.plot([-.6,.6],[0,0],color='dimgrey')
	ax.plot([0,0],[0,0],[-0.1,0.3],color='dimgrey')
	ax.set_xlabel('X - axis')
	ax.set_ylabel('Y - axis')
	ax.set_zlabel('Z - axis')
	#ax.set_xlim([-0.6,0.6])
	#ax.set_ylim([-0.6,0.6])
	#ax.set_zlim([-0.1,0.3])
	plt.show()


#<--------------------- ButterWorth Filter ------------------------>#
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y



#<----------------------------------------------------------------->#
def filtering(x_raw,y_raw,z_raw,time,cutoff=1.5,plot=False):
	order = 6
	fs = 20.0       # sample rate, Hz
	# desired cutoff frequency of the filter, Hz
	points = []
	x = butter_lowpass_filter(x_raw, cutoff, fs, order)
	y = butter_lowpass_filter(y_raw, cutoff, fs, order)
	z = butter_lowpass_filter(z_raw, cutoff, fs, order)
	if plot:
		plotter(x_raw,x,time)
	return x,y,z,time
#<----------------- Downsampling Auxiliaries ---------------------->#

def exclude_outliers(x_raw,y_raw,z_raw,time):
	mean_z, std_z = np.mean(z_raw), np.std(z_raw)
	mean_y, std_y = np.mean(y_raw), np.std(y_raw)
	mean_x, std_x = np.mean(x_raw), np.std(x_raw)
	print(std_x,std_y,std_z)
	c = len(x_raw)
	x, y, z, t = [], [], [], []
	count = 0
	for i in range(0,c):
		if (abs(z_raw[i] - mean_z) < 1.5*std_z):
			t.append(time[i])
			x.append(x_raw[i])
			y.append(y_raw[i])
			z.append(z_raw[i])
			count+=1
	return x,y,z,t



def vel_downsampling(x_raw,y_raw,z_raw,time,plot=True):

	Vx, Vy, Vz, V,angles = First_derivative(x_raw,y_raw,z_raw,time)
	ax, ay, az = Second_derivative(x_raw,y_raw,z_raw,time)

	meanVx, meanVy, meanVz,meanV = np.mean(Vx),np.mean(Vy),np.mean(Vz),np.mean(V)
	meanax, meanay, meanaz = np.mean(ax),np.mean(ay),np.mean(az)

	stdVx, stdVy, stdVz,stdV = np.std(Vx),np.std(Vy),np.std(Vz),np.std(V)
	stdax, stday, stdaz = np.std(ax),np.std(ay),np.std(az)

	if plot:
		plot_kinematics([[x_raw,y_raw,z_raw]],time,case='pos')
	maxV = np.max(V)

	c = len(x_raw)
	x, y, z, t = [], [], [], []
	count = 0
	for i in range(0,c):
		if (abs(V[i] - meanV) < stdV) and abs(az[i] - meanaz) < stdaz:
			t.append(time[i])
			x.append(x1[i])
			y.append(y1[i])
			z.append(z1[i])
			count+=1

	return x,y,z,t


def arbitrary_downsampling(c_l,t_l,r_c_l,r_l,num_files,pos):
	raw_coord_l = [[[],[],[]] for i in range(1, num_files)]
	count_l = [[] for i in range(1, num_files)]
	raw_list = [[] for i in range(1, num_files)]
	time_l = [[] for i in range(1, num_files)]
	for j in range(0,num_files-1):
		x, y, z = [], [], []
		#print(r_c_l[0][2])
		current = c_l[j]
		x_old, y_old, z_old = r_c_l[j][0], r_c_l[j][1], r_c_l[j][2]
		c = 0
		for i in range(0,current-2,int(1/pos)):
			x.append(x_old[i])
			y.append(y_old[i])
			z.append(z_old[i])
			time_l[j].append(t_l[j][i])
			raw_list[j].append(r_l[j][i])
			c+=1
		#insert last point.
		x.append(x_old[c_l[j]-1])
		y.append(y_old[c_l[j]-1])
		z.append(z_old[c_l[j]-1])
		time_l[j].append(t_l[j][c_l[j]-1])
		raw_list[j].append(r_l[j][c_l[j]-1])
		c+=1
		raw_coord_l[j][0] = x
		raw_coord_l[j][1] = y
		raw_coord_l[j][2] = z
		count_l[j] = c
	return raw_coord_l,raw_list,count_l,time_l

#<----------------------------------------------------------------->#
def T(y,y0,x,x0):
	lamda = (y - y0)/(x - x0)
	beta = -lamda*x0 + y0
	return lamda,beta

def scale(t,N,x):
	lamda, beta = T(t[-1],t[0],N,0)
	y = lamda * np.array(x) + beta
	return y


def First_derivative(x,y,z,time):
	"""
	<===== First Derivative ======>
	Computes Velocity of points along the 3-axis.
	"""
	t = time[:]
	N = len(t)
	Vx, Vy, Vz = [], [], []
	for i in range(N):
		if i < N-1:
			Vx.append((x[i+1]-x[i])/(t[i+1]-t[i]))
			Vy.append((y[i+1]-y[i])/(t[i+1]-t[i]))
			Vz.append((z[i+1]-z[i])/(t[i+1]-t[i]))
		else:
			Vx.append((x[i]-x[i-1])/(t[i]-t[i-1]))
			Vy.append((y[i]-y[i-1])/(t[i]-t[i-1]))
			Vz.append((z[i]-z[i-1])/(t[i]-t[i-1]))
	#print("Vx,n=")
	#print(len(Vx),N)
	V = np.sqrt(np.array(Vx)**2+np.array(Vy)**2+np.array(Vz)**2)
	cosx = Vx/V
	cosy = Vy/V
	cosz = Vz/V
	angles = np.vstack((cosx,cosy,cosz))

	return Vx, Vy, Vz, V, angles

def Second_derivative(x,y,z,time):
	ax, ay, az = [], [], []
	Vx, Vy, Vz, V, angles = First_derivative(time,x,y,z)
	t = time[:]
	N = len(t)
	for i in range(N):
		if i < N-1:
			ax.append((Vx[i+1]-Vx[i])/(t[i+1]-t[i]))
			ay.append((Vy[i+1]-Vy[i])/(t[i+1]-t[i]))
			az.append((Vz[i+1]-Vz[i])/(t[i+1]-t[i]))
		else:
			ax.append((Vx[i]-Vx[i-1])/(t[i]-t[i-1]))
			ay.append((Vy[i]-Vy[i-1])/(t[i]-t[i-1]))
			az.append((Vz[i]-Vz[i-1]))
	return ax, ay, az