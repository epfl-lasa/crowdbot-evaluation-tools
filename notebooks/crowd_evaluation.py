# !/usr/bin/env python3
'''
Class to evaluate different metrics during evaluation
'''

import math
import matplotlib.pyplot as plt
import numpy as np
import sys, os
import warnings

def filter_moving_average(input_array, width, axis=1):
    ''' Along axis=0. '''
    if len(input_array.shape) == 1:
        return np.convolve(input_array, np.ones(width), 'valid') / width
    
    output_array = np.zeros((input_array.shape[0], input_array.shape[1] - width + 1))
    for dd in range(input_array.shape[0]):
        output_array[dd, :] = np.convolve(input_array[dd, :], np.ones(width), 'valid') / width
        
    return output_array

def loadArrayIndexRange(index_range, npy_filepath):
	arr = np.load(npy_filepath,allow_pickle=True)
	return arr[index_range[0]:index_range[1]]

class Capsule:
	def __init__(self, y_front, y_back, r):
		self.y_front = y_front
		self.y_back = y_back
		self.r = r
	
	def distanceLocal(self, x, y):
		if y > self.y_front:
			return math.sqrt(x*x + (y - self.y_front)*(y - self.y_front)) - self.r
		elif y < self.y_back:
			return math.sqrt(x*x + (y - self.y_back)*(y - self.y_back)) - self.r
		else:
			return math.fabs(x) - self.r

	def distanceGlobal(self, x_obs, y_obs, x_rob, y_rob, phi_rob):
		Rinv = np.array([
			[np.cos(phi_rob - np.pi/2), np.sin(phi_rob - np.pi/2)],
			[-np.sin(phi_rob - np.pi/2), np.cos(phi_rob - np.pi/2)]])
		p_rel_global = np.array([x_obs - x_rob, y_obs - y_rob])
		p_rel_local = np.matmul(Rinv, p_rel_global)
		return self.distanceLocal(p_rel_local[0], p_rel_local[1])
    

def evalMetricCrowdDensity(npy_files_directory, via_tracked_persons=False, plot_result=False):
	cluster_index_ranges = np.load(os.path.join(npy_files_directory, "cluster_index_ranges.npy"))

	t = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "time.npy"))
	
	# load Qolo-related data
	pose2D =loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "pose2D.npy"))
	x = pose2D[:, 0]
	y = pose2D[:, 1]
	#phi = pose2D[:, 2]
	
	if not via_tracked_persons:
		persons = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "detected_persons_sync.npy"))
	else:
		persons = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "tracked_persons.npy"))
	# load obstacles' data
	#tracker_RDS = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "tracker_RDS.npy"))
	#lrf_RDS = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "lrf_RDS.npy"))
	#detected_persons_sync = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "detected_persons_sync.npy"))
	#tracked_persons = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "tracked_persons.npy"))
	#front_drow_det = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "front_drow_det.npy"))
	#rear_drow_det = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "rear_drow_det.npy"))
	#yolo_det = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "yolo_det.npy"))

	# evaluate the crowd's density
	radius_local_eval = 2.5
	area_local_eval = np.pi*radius_local_eval**2
	n = t.shape[0]
	density = np.empty([n])
	for i in range(n):
		X_det_rel = persons[i].getX() - x[i] 
		Y_det_rel = persons[i].getY() - y[i]
		dist_squared = X_det_rel*X_det_rel + Y_det_rel*Y_det_rel
		people_count = np.sum(np.less(dist_squared, radius_local_eval**2))
		density[i] = people_count/area_local_eval
    
	# evaluate the crowd's density
	radius_local_eval = 5.0
	area_local_eval = np.pi*radius_local_eval**2
	densityl = np.empty([n])
	for i in range(n):
		X_det_rel = persons[i].getX() - x[i] 
		Y_det_rel = persons[i].getY() - y[i]
		dist_squared = X_det_rel*X_det_rel + Y_det_rel*Y_det_rel
		people_count = np.sum(np.less(dist_squared, radius_local_eval**2))
		densityl[i] = people_count/area_local_eval
	if plot_result:
		den1, = plt.plot(t-t[0], density, label='5m area')
		den2, = plt.plot(t-t[0], densityl, label='10m area')
		plt.legend([den1, den2], ['5m area', '10m area'])
		plt.gca().set_xlabel("t [s]")
		plt.gca().set_ylabel("Density [1/m^2]")
		plt.title("Crowd Density")
		plt.show()

	return density


def evalMetricProximity(npy_files_directory, plot_result=False):
	cluster_index_ranges = np.load(os.path.join(npy_files_directory, "cluster_index_ranges.npy"))

	t = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "time.npy"))
	
	# load Qolo-related data
	pose2D =loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "pose2D.npy"))
	x = pose2D[:, 0]
	y = pose2D[:, 1]
	phi = pose2D[:, 2]
	
	# load obstacles' data
	lrf_RDS = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "lrf_RDS.npy"))

	# determine closest distance at each point in time
	capsule_qolo = Capsule(0.18, -0.5, 0.45)
	n = t.shape[0]
	min_dist = np.ones([n])*np.inf
	for i in range(n):
		X_lrf = lrf_RDS[i].getX()
		Y_lrf = lrf_RDS[i].getY()
		m = X_lrf.shape[0]
		for j in range(m):
			d = capsule_qolo.distanceGlobal(X_lrf[j], Y_lrf[j], x[i], y[i], phi[i])
			if d < min_dist[i]:
				min_dist[i] = d

	if plot_result:
		plt.plot(t-t[0], min_dist)
		plt.gca().set_xlabel("t [s]")
		plt.gca().set_ylabel("Min. distance [m]")
		plt.title("Minimum distance of obstacles to bounding capsule")
		plt.show()

	return min_dist


def evalMetricsPathAndTimeToGoal(npy_files_directory, plot_result=False):
	cluster_index_ranges = np.load(os.path.join(npy_files_directory, "cluster_index_ranges.npy"))

	t = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "time.npy"))
	
	# load Qolo-related data
	pose2D =loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "pose2D.npy"))
	x = pose2D[:, 0]
	y = pose2D[:, 1]
	phi = pose2D[:, 2]

	# calculate the goal position in global coordinates
	phi_init = np.sum(phi[9:19])/10.0
	goal = np.array([np.cos(phi_init), np.sin(phi_init)])*20.0

	# determine when the closest point to the goal is reached
	n = t.shape[0]
	min_dist = np.inf
	i_min_dist = -1
	for i in range(n):
		distance_to_goal = np.sqrt((x[i] - goal[0])**2 + (y[i] - goal[1])**2)
		if distance_to_goal < min_dist:
			min_dist = distance_to_goal
			i_min_dist = i

	# compute the path length to the closest point
	path_length_to_goal = np.sum(np.sqrt(np.diff(x[:i_min_dist])**2 + np.diff(y[:i_min_dist])**2))
	time_duration_to_goal = t[i_min_dist] - t[0]

	if plot_result:
		plt.plot(x[:i_min_dist], y[:i_min_dist], "b",
			label="path (l=%.1f m, t=%.1f s)" % (path_length_to_goal, time_duration_to_goal))
		plt.plot([goal[0]], [goal[1]], "kx", label="goal")
		plt.plot([0.0], [0.0], "ks", label="start")
		plt.gca().legend()
		plt.gca().set_xlabel("x [m]")
		plt.gca().set_ylabel("y [m]")
		plt.title("Path. Closest distance to the goal=%.1f m" % min_dist)
		plt.show()

	return (path_length_to_goal, time_duration_to_goal, min_dist)

def command_fluency(commands):
    Command_U = commands[:,[0,1]]
    vec_size = commands.shape[0]
    # print (Command_U)
    max_v = np.amax(Command_U[:,[0]])
    max_w = np.amax(Command_U[:,[1]])
    # print(max_v,max_w)
    # ccount = np.count_nonzero(Command_U,0)
#     ccount=0;
#     for i_indx in range(n):
#         if Command_U[i_indx,[0]] or Command_U[i_indx,[1]]:
#             ccount=ccount+1
    j_indx=0
    fluency_v = []
    fluency_w = []

    for i_indx in range(2,vec_size):
        if Command_U[i_indx,[0]] or Command_U[i_indx,[1]]:
    #         print (i_indx,Command_U[i_indx,[0]])
            fluency_v.append(1 - np.abs(Command_U[i_indx,[0]] - Command_U[i_indx-1,[0]])/max_v)
            fluency_w.append(1 - np.abs(Command_U[i_indx,[1]] - Command_U[i_indx-1,[1]])/max_w)
            j_indx=j_indx+1

    # print (np.array(fluency_v))
    fluencyv = np.mean(np.array(fluency_v))
    fleuncyv_sd = np.std(np.array(fluency_v))
    fluencyw = np.mean(np.array(fluency_w))
    fleuncyw_sd = np.std(np.array(fluency_w))

    if fluencyv < fluencyw:
        return (fluencyv,fleuncyv_sd)
    else:
        return (fluencyw,fleuncyw_sd)


def similarity(commands):
    Command_U = commands[:,[0,1]]
    Command_R = commands[:,[2,3]]
    vec_size = commands.shape[0]
    # print (Command_U)
    max_v = np.amax(Command_U[:,[0]])
    max_w = np.amax(Command_U[:,[1]])

    angle_U = np.arctan2(Command_U[:,[0]]/max_v,Command_U[:,[1]]/max_w);
    angle_R = np.arctan2(Command_R[:,[0]]/max_v,Command_R[:,[1]]/max_w);
    angle_diff = angle_R - angle_U;

    ccount=0;
    omega_diff = []
    vel_diff = []
    agreement_vec = []
    for i_indx in range(2,vec_size):
        if Command_U[i_indx,[0]] or Command_U[i_indx,[1]]:
            vel_diff.append (np.abs(Command_R[i_indx,[0]] - Command_U[i_indx,[0]])/max_v)
            omega_diff.append (np.abs(Command_R[i_indx,[1]] - Command_U[i_indx,[1]])/max_w)
            agreement_vec.append (1 - ( abs(angle_diff[i_indx]) /np.pi ))
            ccount=ccount+1;
    command_diff = np.column_stack((np.array(vel_diff), np.array(omega_diff)))
    # print(command_diff)
    command_vec = np.linalg.norm(command_diff, axis=1)
    # print(len(command_vec))

    vel_diff = np.array(vel_diff)
    omega_diff = np.array(omega_diff)
    agreement_vec = np.array(agreement_vec)
    # print (vel_diff, omega_diff)
    directional_agreement = [np.mean(agreement_vec), np.std(agreement_vec)]


    linear_dis = ([np.mean(vel_diff), np.std(vel_diff)])
#     print(linear_dis)
    heading_dis = ([np.mean(omega_diff), np.std(omega_diff)])
#     print(heading_dis)
    # disagreement(1,1) = sum(vecnorm(Command_R_norm - Command_U_norm)) / Ccount; % L2-Norm difference

    disagreement = ([np.mean(np.array(command_vec)), np.std(np.array(command_vec))])
#     print(disagreement)
    # contribution = []
    # j_indx = 1
    # for i_indx in range(2,vec_size):
    #     if Command_U[i_indx,[0]] or Command_U[i_indx,[1]]:
    #         if  Command_U[i_indx,[0]]==0:
    #             contribution.append((Command_U[i_indx,[1]] - Command_R[i_indx,[1]])/ Command_U[i_indx,[1]]);
    #         elif  Command_U[i_indx,[1]]==0:
    #             contribution.append((Command_U[i_indx,[0]] - Command_R[i_indx,[0]])/ Command_U[i_indx,[0]]);
    #         else:
    #             contribution.append (np.linalg.norm([(Command_U[i_indx,[0]] - Command_R[i_indx,[0]])/ Command_U[i_indx,[0]], (Command_U[i_indx,[1]] - Command_R[i_indx,[1]])/ Command_U[i_indx,[1]]]) )

    # # print(contribution)
    # contribution_m = np.mean(np.array(contribution))

    # print(contribution_m)

    return (linear_dis, heading_dis, disagreement) #Contribution


def MetricsSharedControl(npy_files_directory, plot_result=False):
    cluster_index_ranges = np.load(os.path.join(npy_files_directory, "cluster_index_ranges.npy"))
    t = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "time.npy"))
    n = t.shape[0]
    # load Qolo-related data
    pose2D =loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "pose2D.npy"))
    x = pose2D[:, 0]
    y = pose2D[:, 1]
    #phi = pose2D[:, 2]

    # load Commands/Execution from data commands.npy
    # 	(N, 4)-array, where each row [v_n, w_n, v_c, w_c] 
    commands = loadArrayIndexRange(cluster_index_ranges[0], os.path.join(npy_files_directory, "commands.npy"))
    # print(commands.shape[0])
    # print(commands[:,[0]])
    # print(len(commands))
    fluency = command_fluency(commands)
    # print(fluency)
    linear_dis, heading_dis, disagreement = similarity(commands)
    
    return (fluency, linear_dis, heading_dis, disagreement)
    
    
    
    
    

