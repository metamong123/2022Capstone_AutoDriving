#!/usr/bin/python
#-*- coding: utf-8 -*-

from object_msgs.msg import Object
import rospy
import numpy as np
import math, time
import tf
import matplotlib.pyplot as plt
import rospkg
import sys
from ackermann_msgs.msg import AckermannDriveStamped

from scipy.interpolate import interp1d

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object, ObjectArray

import pickle
import argparse

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
from map_visualizer import Converter

rn_id = dict()

# rn_id[5] = {'right': [0, 1, 2, 3, 4, 5, 6]}  # ego route
rn_id[5] = {'right': [0]}

def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi


def interpolate_waypoints(wx, wy, space=0.5):
	_s = 0
	s = [0]
	for i in range(1, len(wx)):
		prev_x = wx[i - 1]
		prev_y = wy[i - 1]
		x = wx[i]
		y = wy[i]

		dx = x - prev_x
		dy = y - prev_y

		_s = np.hypot(dx, dy)
		s.append(s[-1] + _s)

	fx = interp1d(s, wx)
	fy = interp1d(s, wy)
	ss = np.linspace(0, s[-1], num=int(s[-1] / space) + 1, endpoint=True)

	dxds = np.gradient(fx(ss), ss, edge_order=1)
	dyds = np.gradient(fy(ss), ss, edge_order=1)
	wyaw = np.arctan2(dyds, dxds)

	return {
		"x": fx(ss),
		"y": fy(ss),
		"yaw": wyaw,
		"s": ss
	}


class State:

	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=1.04):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
		self.dt = dt
		self.WB = WB

	def update(self, a, delta):
		dt = self.dt
		WB = self.WB

		self.x += self.v * math.cos(self.yaw) * dt
		self.y += self.v * math.sin(self.yaw) * dt
		self.yaw += self.v / WB * math.tan(delta) * dt
		self.yaw = pi_2_pi(self.yaw)
		self.v += a * dt
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

	def calc_distance(self, point_x, point_y):
		dx = self.rear_x - point_x
		dy = self.rear_y - point_y
		return math.hypot(dx, dy)

	def get_ros_msg(self, a, steer, v):
		dt = self.dt

		c = AckermannDriveStamped()
		c.header.frame_id = "/map"
		c.header.stamp = rospy.Time.now()
		c.drive.steering_angle = steer
		c.drive.acceleration = a
		# c.drive.speed = self.v
		c.drive.speed = v + a * dt
		# print("ackermann_speed:"+str(v))
		return c


def get_ros_msg(x, y, yaw, v, a, steer, id):
	quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

	m = Marker()
	m.header.frame_id = "/map"
	m.header.stamp = rospy.Time.now()
	m.id = id
	m.type = m.CUBE

	m.pose.position.x = x + 1.3 * math.cos(yaw)
	m.pose.position.y = y + 1.3 * math.sin(yaw)
	m.pose.position.z = 0.75
	m.pose.orientation = Quaternion(*quat)

	m.scale.x = 1.600
	m.scale.y = 1.160
	m.scale.z = 0.110

	m.color.r = 93 / 255.0
	m.color.g = 122 / 255.0
	m.color.b = 177 / 255.0
	m.color.a = 0.97

	o = Object()
	o.header.frame_id = "/map"
	o.header.stamp = rospy.Time.now()
	o.id = id
	o.classification = o.CLASSIFICATION_CAR
	o.x = x
	o.y = y
	o.yaw = yaw
	o.v = v
	o.L = m.scale.x
	o.W = m.scale.y
 
	dt=0.1
	c = AckermannDriveStamped()
	c.header.frame_id = "/map"
	c.header.stamp = rospy.Time.now()
	c.drive.steering_angle = steer
	c.drive.acceleration = a
	c.drive.speed = v + a*dt

	return {
		"object_msg": o,
		"marker_msg": m,
		"quaternion": quat,
		"ackermann_msg" : c
	}

#obs_init1 = Object(x=1, y=11, yaw=1, L=4, W=5)
#obs_init2 = Object(x=3, y=33, yaw=1, L=3, W=3)
# hightech
#obj_msg = Object(x=962581.2429941624, y=1959229.97720466, yaw=1.2871297862692013, L=1.600, W=1.04)
# playground short
# obj_msg = Object(x=962692.1184323871, y=1959011.6193129763, yaw=1.2871297862692013, L=4.475, W=1.85)
# playground long
# obj_msg = Object(x=962689.2030317801, y=1959006.1865985924, yaw=1.2871297862692013, L=4.475, W=1.85)

obj_msg = Object(x=962587.11409, y=1959260.09207, yaw=1.2871297862692013, v=1,L=1.600, W=1.04)
# obj_msg = Object(x=962620.042756, y=1959328.22085, yaw=1.2871297862692013, L=4.475, W=1.85)

obs_info = []
def callback_obstacle(msg):
	global obs_info
	obs_info = []
	for o in msg.object_list:
		obj = [o.x, o.y, o.yaw, o.L, o.W]

		'''
		#####(x, y) 좌표가 중심이 아니라 시작점인지?
		yaw = o.yaw           
		center_x = o.x + 1.3 * math.cos(yaw)
		center_y = o.y + 1.3 * math.sin(yaw)
		'''
		#id(=i)가 문자열이어야 하는지 확인 필요
		obs_info.append(obj)

def callback3(msg):
	global obj_msg
	obj_msg = msg
 

if __name__ == "__main__":
	a_list=[]
	v_list=[]
	steer_list=[]
	parser = argparse.ArgumentParser(description='Spawn a CV agent')

	parser.add_argument("--id", "-i", type=int, help="agent id", default=1)
	parser.add_argument("--route", "-r", type=int,
						help="start index in road network. select in [1, 3, 5, 10]", default=5)
	parser.add_argument("--dir", "-d", type=str, default="right", help="direction to go: [left, straight, right]")
	args, unknown = parser.parse_known_args()

	rospy.init_node("three_cv_agents_node_" + str(args.id))
	obstacle_sub = rospy.Subscriber("obstacles", ObjectArray, callback_obstacle, queue_size=1)
	sub_state = rospy.Subscriber("/objects/car_1", Object, callback3, queue_size=1)
	WB = 1.04

	'''
	while sub_obs2.get_num_connections() == 0 or sub_obs3.get_num_connections() == 0:
		continue
	'''

	id = args.id
	tf_broadcaster = tf.TransformBroadcaster()
	opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
	cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
	control_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)

	start_node_id = args.route
	route_id_list = rn_id[start_node_id][args.dir]

	with open(path_map + "/src/route.pkl", "rb") as f: #global
		nodes= pickle.load(f)

	# with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/route.pkl", "rb") as f:
	# 	nodes = pickle.load(f)

	# nodes[6]={}
	# nodes[6]={'x':nodes[0]['x'][600:], 'y':nodes[0]['y'][600:], 's':nodes[0]['s'][600:], 'yaw':nodes[0]['yaw'][600:]}
	# nodes[5]={}
	# nodes[5]={'x':nodes[0]['x'][500:600], 'y':nodes[0]['y'][500:600], 's':nodes[0]['s'][500:600], 'yaw':nodes[0]['yaw'][500:600]}
	# nodes[4]={}
	# nodes[4]={'x':nodes[0]['x'][400:500], 'y':nodes[0]['y'][400:500], 's':nodes[0]['s'][400:500], 'yaw':nodes[0]['yaw'][400:500]}
	# nodes[3]={}
	# nodes[3]={'x':nodes[0]['x'][300:400], 'y':nodes[0]['y'][300:400], 's':nodes[0]['s'][300:400], 'yaw':nodes[0]['yaw'][300:400]}
	# nodes[2]={}
	# nodes[2]={'x':nodes[0]['x'][200:300], 'y':nodes[0]['y'][200:300], 's':nodes[0]['s'][200:300], 'yaw':nodes[0]['yaw'][200:300]}
	# nodes[1]={}
	# nodes[1]={'x':nodes[0]['x'][100:200], 'y':nodes[0]['y'][100:200], 's':nodes[0]['s'][100:200], 'yaw':nodes[0]['yaw'][100:200]}
	# nodes[0]['x'] = nodes[0]['x'][:100]
	# nodes[0]['y'] = nodes[0]['y'][:100]
	# nodes[0]['s'] = nodes[0]['s'][:100]
	# nodes[0]['yaw'] = nodes[0]['yaw'][:100]
 
	error_icte=0
	prev_cte =0
	cte = 0

	
	link_i=-1
	link_len=[]
	for i in range(len(nodes)):
		link_i+=len(nodes[i]["x"])
		link_len.append(link_i)


	link_ind=0

	wx = []
	wy = []
	wyaw = []

	for _id in route_id_list:
		wx.append(nodes[_id]["x"][1:])
		wy.append(nodes[_id]["y"][1:])
		wyaw.append(nodes[_id]["yaw"][1:])
	wx = np.concatenate(wx)
	wy = np.concatenate(wy)
	wyaw = np.concatenate(wyaw)


	# ws = np.zeros(wx.shape)
	# for i in range(len(ws)):
	# 	x = wx[i]
	# 	y = wy[i]
	# 	sd = get_frenet(x, y, wx, wy)
	# 	ws[i] = sd[0]

	waypoints = interpolate_waypoints(wx, wy, space=0.5)
	#waypoints = {"x": wx, "y": wy, "yaw": wyaw, "s" : ws}

	mapx = waypoints["x"]
	mapy = waypoints["y"]
	mapyaw = waypoints["yaw"]
	maps = waypoints["s"]

	v=0
	prev_ind=0
	# ind = 10
	target_speed = 10.0 / 3.6
	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=obj_msg.v, dt=0.1)
	state.x=obj_msg.x
	state.y=obj_msg.y
	state.yaw=obj_msg.yaw
	#############
	state.v=obj_msg.v
	#############
	msg = state.get_ros_msg(0, 0, 1.0) #a,steer,v
	control_pub.publish(msg)
 	#state = obj_car
	v_list.append(state.v)
	my_wp=0
	#my_wp = get_closest_waypoints(state.x, state.y, mapx[:100], mapy[:100],my_wp)
	my_wp = get_closest_waypoints(state.x, state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(10)
	ai = 0

	# if my_wp >= (link_len[link_ind]-1):
	# 	link_ind+=1
	if my_wp >= (link_len[link_ind]-10):
		link_ind+=1

	prev_ind = link_ind-2
	# s, d = get_frenet(state.x, state.y, mapx[:100], mapy[:100],my_wp)
	# x, y, road_yaw = get_cartesian(s, d, mapx[:100], mapy[:100],maps[:100])
	s, d = get_frenet(state.x, state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)
	x, y, road_yaw = get_cartesian(s, d, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],maps[:link_len[link_ind]])
	
	yawi = state.yaw - road_yaw
	si = s
	si_d = state.v * math.cos(yawi)
	si_dd = ai * math.cos(yawi)
	sf_d = target_speed
	sf_dd = 0

	di = d
	di_d = state.v * math.sin(yawi)
	di_dd = ai * math.sin(yawi)
	df_d = 0
	df_dd = 0
	
	opt_d = d
	prev_opt_d = d

	opt_frenet_path = Converter(r=0, g=255/255.0, b=100/255.0, a=1, scale=0.5)
	cand_frenet_paths = Converter(r=0, g=100/255.0, b=100/255.0, a=0.4, scale= 0.5)

	while not rospy.is_shutdown():
		# generate acceleration ai, and steering di
		# YOUR CODE HERE

		path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx, mapy, maps, opt_d, target_speed)
		# update state with acc, delta
		if opt_ind == -1: ## No solution!
			my_wp = get_closest_waypoints(state.x,state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)
  
			# if my_wp >= (link_len[link_ind]-10):
			# 	if link_ind==42:
			# 		link_ind=42
			# 	else:
			# 		link_ind+=1
			# # prev_ind = link_ind-2
			# print("현재 링크 번호: "+ str(link_ind))
  
			s, d = get_frenet(state.x, state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)
			x, y, road_yaw = get_cartesian(s, d, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],maps[:link_len[link_ind]])
			steer = road_yaw - state.yaw
			a = 0
			opt_d = prev_opt_d
		else:
			## PID control

			error_pa = target_speed - state.v
			error_da = state.v - prev_v
			error_ia += target_speed - state.v
			kp_a = 0.5
			kd_a = 0.7
			ki_a = 0.01
			a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia
			prev_cte = cte
			error_icte += cte
			# steering angle pid control
			steer, cte, _ = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, WB, error_icte, prev_cte)
			# steer, _ = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, WB)
			
			ways = []
			for p in path:
				way = {
					"x" : p.x,
					"y" : p.y
				}	
				ways.append(way)
			opt_frenet_path.make_marker_array([ways[opt_ind]])
			cand_frenet_paths.make_marker_array(ways)

			opt_d = path[opt_ind].d[-1]
			prev_opt_d = path[opt_ind].d[-1]
		
		ai=a
		# vehicle state --> topic msg
		# state.update(a, steer)
		if ((my_wp < (link_len[-1] -10)) & (obj_msg.v <= 1)):
			msg = state.get_ros_msg(0, steer, 1.0)
		else:
			msg = state.get_ros_msg(a, steer, obj_msg.v)
		control_pub.publish(msg)
		#a_list.append(a)
		#steer_list.append(steer)
		#v_list.append(v)
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer) + ",a = "+str(a))
		prev_v = state.v
		state.x=obj_msg.x
		state.y=obj_msg.y
		state.yaw=obj_msg.yaw
		state.v=obj_msg.v
		my_wp = get_closest_waypoints(state.x,state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)

		# if my_wp >= (link_len[link_ind]-10):
		# 	if link_ind==40:
		# 		link_ind=40
		# 	else:
		# 		link_ind+=1
		# # prev_ind = link_ind-2
		# print("현재 링크 번호: "+ str(link_ind))

		# if my_wp == 270:
		# 	with open("/home/nsclmds/a_list.text", "wb") as f:
		# 		pickle.dump(a_list, f)
		# 	with open("/home/nsclmds/v_list.text", "wb") as f:
		# 		pickle.dump(v_list, f)
		# 	with open("/home/nsclmds/steer_list.text", "wb") as f:
		# 		pickle.dump(steer_list, f)
    
		s, d = get_frenet(state.x, state.y, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],my_wp)
		x, y, road_yaw = get_cartesian(s, d, mapx[:link_len[link_ind]], mapy[:link_len[link_ind]],maps[:link_len[link_ind]])
		yaw_diff = state.yaw - road_yaw

		si = s
		si_d = state.v * math.cos(yaw_diff)
		si_dd = ai * math.cos(yaw_diff)
		sf_d = target_speed
		sf_dd = 0
		
		di = d
		di_d = state.v * math.sin(yaw_diff)
		di_dd = ai * math.sin(yaw_diff)
		df_d = 0
		df_dd = 0

		# vehicle state --> topic msg
		# msg = get_ros_msg(state.x, state.y, state.yaw, state.v, a, steer, id=id)
		# msg = state.get_ros_msg(a, steer, id=id)

		# send tf
		#tf_broadcaster.sendTransform(
		#	(state.x, state.y, 1.5),
		#	msg["quaternion"],
		#	rospy.Time.now(),
		#	"/car_" + str(id), "/map"
		#)

		# publish vehicle state in ros msg
		#object_pub.publish(msg["object_msg"])
		opt_frenet_pub.publish(opt_frenet_path.ma)
		cand_frenet_pub.publish(cand_frenet_paths.ma)
		control_pub.publish(msg)

		r.sleep()
	