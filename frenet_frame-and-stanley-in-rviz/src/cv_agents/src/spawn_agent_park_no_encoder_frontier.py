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
from std_msgs.msg import String, Float64
from rocon_std_msgs.msg import StringArray

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
rn_id[5] = {'global': [i for i in range(5)],'parking':[i for i in range(12)]}

def find_dir(link_dict, link_ind):
	for i in link_dict.keys():
		for j in link_dict[i]:
			if link_ind == j:
				return i

def find_link(link_len, my_wp):
    for i in range(len(link_len)-1):
        if my_wp > link_len[i] and my_wp <= link_len[i+1]:
            return i+1

class ParkingPath:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []

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

	def get_ros_msg(self, a, steer, id):
		dt = self.dt
		v = self.v

		c = AckermannDriveStamped()
		c.header.frame_id = "/map"
		c.header.stamp = rospy.Time.now()
		c.drive.steering_angle = steer
		c.drive.acceleration = a
		c.drive.speed = v

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
	m.scale.y = 1.650
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

def waypoint_topic(my_wp):
	f=Float64()
	f.data = my_wp
	return f

def mode_array(car_mode, move_mode, current_dir, next_dir):
	m = StringArray()
	# current_dir=dir_mode[0]
	# next_dir=dir_mode[1]
	m.strings=[car_mode, move_mode, current_dir, next_dir]
	return m

#obs_init1 = Object(x=1, y=11, yaw=1, L=4, W=5)
#obs_init2 = Object(x=3, y=33, yaw=1, L=3, W=3)
# hightech
# obj_msg = Object(x=962581.2429941624, y=1959229.97720466, yaw=1.2871297862692013, L=4.475, W=1.85)
# playground short
# obj_msg = Object(x=962692.1184323871, y=1959011.6193129763, yaw=1.2871297862692013, L=4.475, W=1.85)
# playground long
# obj_msg = Object(x=962689.2030317801, y=1959006.1865985924, yaw=1.2871297862692013, L=4.475, W=1.85)

obj_msg = Object(x=962587.11409, y=1959260.09207, yaw=1.2871297862692013, v=1,L=1.600, W=1.04)

# obj_msg = Object(x=962620.042756, y=1959328.22085, yaw=1.2871297862692013, L=4.475, W=1.85)

# obs_info = []
# def callback_obstacle(msg):
# 	global obs_info
# 	obs_info = []
# 	for o in msg.object_list:
# 		obj = [o.x, o.y, o.yaw, o.L, o.W]

# 		'''
# 		#####(x, y) 좌표가 중심이 아니라 시작점인지?
# 		yaw = o.yaw           
# 		center_x = o.x + 1.3 * math.cos(yaw)
# 		center_y = o.y + 1.3 * math.sin(yaw)
# 		'''
# 		#id(=i)가 문자열이어야 하는지 확인 필요
# 		obs_info.append(obj)

# obj_msg=Object()
# def callback3(msg):
# 	global obj_msg
# 	obj_msg=msg
# 	# print(obj_msg)


# def callback2(msg):
# 	global mode
# 	mode=msg.data

obs_info = []
# obj_msg=Object()
class TopicReciver:
	def __init__(self):
		self.obstacle_sub = rospy.Subscriber("obstacles", ObjectArray, self.callback_obstacle, queue_size=1)
		self.state_sub = rospy.Subscriber("/objects/car_1", Object, self.callback2, queue_size=1)
	def check_all_connections(self):
		return (self.obstacle_sub.get_num_connections()+self.state_sub.get_num_connections())==2
	def callback_obstacle(self, msg):
		if self.check_all_connections():
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
	def callback2(self, msg):
		if self.check_all_connections():
			global obj_msg
			obj_msg=msg


if __name__ == "__main__":
	a_list=[]
	v_list=[]
	steer_list=[]
	fin_wp = [0,0]
	obs_wp=0
	parser = argparse.ArgumentParser(description='Spawn a CV agent')

	parser.add_argument("--id", "-i", type=int, help="agent id", default=1)
	parser.add_argument("--route", "-r", type=int,
						help="start index in road network. select in [1, 3, 5, 10]", default=5)
	parser.add_argument("--dir", "-d", type=str, default="global", help="direction to go: [global, parking]")
	args, unknown = parser.parse_known_args()

	rospy.init_node("three_cv_agents_node_" + str(args.id))
	topic_receiver=TopicReciver()
	# obstacle_sub = rospy.Subscriber("obstacles", ObjectArray, callback_obstacle, queue_size=1)
	# sub_state = rospy.Subscriber("/objects/car_1", Object, callback3, queue_size=1)
	# mode_sub = rospy.Subscriber("/mode", String, callback2)
	WB = 1.04

	'''
	while sub_obs2.get_num_connections() == 0 or sub_obs3.get_num_connections() == 0:
		continue
	'''

	id = args.id
	tf_broadcaster = tf.TransformBroadcaster()
	opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
	cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
	control_pub = rospy.Publisher("/ackermann_cmd_frenet", AckermannDriveStamped, queue_size=1)
	# car_mode_pub=rospy.Publisher("/car_mode", String, queue_size=1)
	# move_mode_pub=rospy.Publisher("/move_mode", String, queue_size=1)
	mode_pub=rospy.Publisher("/mode_selector", StringArray, queue_size=1)
	waypoint_pub = rospy.Publisher("/waypoint", Float64, queue_size=1)
	# rospy.set_param('move_mode', 'global')
	start_node_id = args.route
	route_id_list = rn_id[start_node_id][args.dir]

	# nodes={'global':{0:[],1:[],2:[],3:[],4:[],5:[],6:[]},'parking':{0:[],1:[]}}
	nodes={'global':{},'parking':{}}
	with open(path_map + "/src/frontier/route.pkl", "rb") as f: #global
		nodes['global']=pickle.load(f)
	
	# with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/route.pkl", "rb") as f:
	# 	nodes['global'] = pickle.load(f)

	# nodes['global'][41]={}
	# nodes['global'][40]={}

	# for i in range(39,16,-1):
	# 	nodes['global'][i+2]=nodes['global'][i]
	
	# nodes['global'][18]={}
	# nodes['global'][17]={}
	# nodes['global'][18]={'x':nodes['global'][16]['x'][215:], 'y':nodes['global'][16]['y'][215:], 's':nodes['global'][16]['s'][215:], 'yaw':nodes['global'][16]['yaw'][215:]}
	# nodes['global'][17]={'x':nodes['global'][16]['x'][165:215], 'y':nodes['global'][16]['y'][165:215], 's':nodes['global'][16]['s'][165:215], 'yaw':nodes['global'][16]['yaw'][165:215]}
	# nodes['global'][16]={'x':nodes['global'][16]['x'][:165], 'y':nodes['global'][16]['y'][:165], 's':nodes['global'][16]['s'][:165], 'yaw':nodes['global'][16]['yaw'][:165]}

	# for i in range(41,25,-1):
	# 	nodes['global'][i+1]=nodes['global'][i]
	# nodes['global'][26]={}
	# nodes['global'][26]={'x':nodes['global'][25]['x'][45:117], 'y':nodes['global'][25]['y'][45:117], 's':nodes['global'][25]['s'][45:117], 'yaw':nodes['global'][25]['yaw'][45:117]}
	# nodes['global'][25]={'x':nodes['global'][25]['x'][:45], 'y':nodes['global'][25]['y'][:45], 's':nodes['global'][25]['s'][:45], 'yaw':nodes['global'][25]['yaw'][:45]}
 
	# for i in range(42,29,-1):
	# 	nodes['global'][i+2]=nodes['global'][i]
	# nodes['global'][31]={}
	# nodes['global'][30]={}
	# nodes['global'][31]={'x':nodes['global'][29]['x'][117:], 'y':nodes['global'][29]['y'][117:], 's':nodes['global'][29]['s'][117:], 'yaw':nodes['global'][29]['yaw'][117:]}
	# nodes['global'][30]={'x':nodes['global'][29]['x'][47:117], 'y':nodes['global'][29]['y'][47:117], 's':nodes['global'][29]['s'][47:117], 'yaw':nodes['global'][29]['yaw'][47:117]}
	# nodes['global'][29]={'x':nodes['global'][29]['x'][:47], 'y':nodes['global'][29]['y'][:47], 's':nodes['global'][29]['s'][:47], 'yaw':nodes['global'][29]['yaw'][:47]} ##node 44개

	# for i in range(44, 32, -1):
	# 	nodes['global'][i+1]=nodes['global'][i]
	# nodes['global'][33]={}
	# nodes['global'][33]={'x':nodes['global'][32]['x'][118:], 'y':nodes['global'][32]['y'][118:], 's':nodes['global'][32]['s'][118:], 'yaw':nodes['global'][32]['yaw'][118:]}
	# nodes['global'][32]={'x':nodes['global'][32]['x'][:118], 'y':nodes['global'][32]['y'][:118], 's':nodes['global'][32]['s'][:118], 'yaw':nodes['global'][32]['yaw'][:118]} ##node 45

	link_dir={'straight':[0,1,2,3,4,5,6,10,11,12,14,16,18,21,22,23,24,25,29,31,32,37,38,40,41,42,43,44,45],'left':[7,8,26,27,28,30,33,34],'right':[9,13,15,17,19,20,35,36,39]}
	
	dir=[]
	# nodes['global'][6]={}
	nodes['global'][4]={'x':nodes['global'][0]['x'][600:], 'y':nodes['global'][0]['y'][600:], 's':nodes['global'][0]['s'][600:], 'yaw':nodes['global'][0]['yaw'][600:]}
	# nodes['global'][5]={}
	nodes['global'][3]={'x':nodes['global'][0]['x'][500:600], 'y':nodes['global'][0]['y'][500:600], 's':nodes['global'][0]['s'][500:600], 'yaw':nodes['global'][0]['yaw'][500:600]}
	# nodes['global'][4]={}
	nodes['global'][2]={'x':nodes['global'][0]['x'][400:500], 'y':nodes['global'][0]['y'][400:500], 's':nodes['global'][0]['s'][400:500], 'yaw':nodes['global'][0]['yaw'][400:500]}
	# nodes['global'][3]={}
	nodes['global'][1]={'x':nodes['global'][0]['x'][300:400], 'y':nodes['global'][0]['y'][300:400], 's':nodes['global'][0]['s'][300:400], 'yaw':nodes['global'][0]['yaw'][300:400]}
	# # nodes['global'][2]={}
	# nodes['global'][2]={'x':nodes['global'][0]['x'][200:300], 'y':nodes['global'][0]['y'][200:300], 's':nodes['global'][0]['s'][200:300], 'yaw':nodes['global'][0]['yaw'][200:300]}
	# # nodes['global'][1]={}
	# nodes['global'][1]={'x':nodes['global'][0]['x'][100:200], 'y':nodes['global'][0]['y'][100:200], 's':nodes['global'][0]['s'][100:200], 'yaw':nodes['global'][0]['yaw'][100:200]}
	nodes['global'][0]['x'] = nodes['global'][0]['x'][:300]
	nodes['global'][0]['y'] = nodes['global'][0]['y'][:300]
	nodes['global'][0]['s'] = nodes['global'][0]['s'][:300]
	nodes['global'][0]['yaw'] = nodes['global'][0]['yaw'][:300]
 
	with open(path_map + "/src/frontier/route_parking1.pkl", "rb") as f: #parking
		nodes['parking']= pickle.load(f)
	nodes['parking'][1]={}
	nodes['parking'][1]=nodes['parking'][0]
	with open(path_map + "/src/frontier/route_parking2.pkl", "rb") as f: #route_
		park_2= pickle.load(f)
		nodes['parking'][2]=park_2[0]
	nodes['parking'][3]={}
	nodes['parking'][3]=nodes['parking'][2]
	with open(path_map + "/src/frontier/route_parking3.pkl", "rb") as f: #parking
		park_4= pickle.load(f)
		nodes['parking'][4]=park_4[0]
	nodes['parking'][5]={}
	nodes['parking'][5]=nodes['parking'][4]
	# with open(path_map + "/src/frontier/parking4.pkl", "rb") as f: #parking
	# 	park_6= pickle.load(f)
	# 	nodes['parking'][6]=park_6[0]
	# nodes['parking'][7]={}
	# nodes['parking'][7]=nodes['parking'][6]
	# with open(path_map + "/src/frontier/parking5.pkl", "rb") as f: #parking
	# 	park_8= pickle.load(f)
	# 	nodes['parking'][8]=park_8[0]
	# nodes['parking'][9]={}
	# nodes['parking'][9]=nodes['parking'][8]
	# with open(path_map + "/src/frontier/parking6.pkl", "rb") as f: #parking
	# 	park_10= pickle.load(f)
	# 	nodes['parking'][10]=park_10[0]
	# nodes['parking'][11]={}
	# nodes['parking'][11]=nodes['parking'][10]
 
 
	with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking1.pkl", "rb") as f: #parking
		nodes['parking']= pickle.load(f)
	nodes['parking'][1]={}
	nodes['parking'][1]=nodes['parking'][0]
	with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking2.pkl", "rb") as f: #parking
		park_2= pickle.load(f)
		nodes['parking'][2]=park_2[0]
	nodes['parking'][3]={}
	nodes['parking'][3]=nodes['parking'][2]
	# with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking3.pkl", "rb") as f: #parking
	# 	park_4= pickle.load(f)
	# 	nodes['parking'][4]=park_4[0]
	# nodes['parking'][5]={}
	# nodes['parking'][5]=nodes['parking'][4]
	# with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking4.pkl", "rb") as f: #parking
	# 	park_6= pickle.load(f)
	# 	nodes['parking'][6]=park_6[0]
	# nodes['parking'][7]={}
	# nodes['parking'][7]=nodes['parking'][6]
	# with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking5.pkl", "rb") as f: #parking
	# 	park_8= pickle.load(f)
	# 	nodes['parking'][8]=park_8[0]
	# nodes['parking'][9]={}
	# nodes['parking'][9]=nodes['parking'][8]
	# with open("/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking6.pkl", "rb") as f: #parking
	# 	park_10= pickle.load(f)
	# 	nodes['parking'][10]=park_10[0]
	# nodes['parking'][11]={}
	# nodes['parking'][11]=nodes['parking'][10]
 


	error_icte=0
	prev_cte =0
	cte = 0

	link_len={'global':[],'parking':[]}
	for i in nodes:
		link_i=-1
		for j in nodes[i].keys():
			if i == 'global':
				link_i+=len(nodes[i][j]["x"])
				link_len[i].append(link_i)
			else:
				if j % 2 ==0:
					link_i=-1
					link_i+=len(nodes[i][j]["x"])
					link_len[i].append(link_i)
				else:
					link_i+=len(nodes[i][j]["x"])
					link_len[i].append(link_i)


	link_ind={'global':0, 'parking':0}
	wx = {'global':[],'parking':{0:[],2:[],4:[]}}
	wy = {'global':[],'parking':{0:[],2:[],4:[]}}
	wyaw = {'global':[],'parking':{0:[],2:[],4:[]}}
	# wx = {'global':[],'parking':{0:[],2:[],4:[],6:[],8:[],10:[]}}
	# wy = {'global':[],'parking':{0:[],2:[],4:[],6:[],8:[],10:[]}}
	# wyaw = {'global':[],'parking':{0:[],2:[],4:[],6:[],8:[],10:[]}}

	for i in nodes.keys():
		for _id in nodes[i].keys():
			if i == 'parking':
				if _id %2==0:
					wx[i][_id].append(nodes[i][_id]["x"][1:])
					wy[i][_id].append(nodes[i][_id]["y"][1:])
					wyaw[i][_id].append(nodes[i][_id]["yaw"][1:])
				else:
					wx[i][_id-1].append(nodes[i][_id]["x"][1:])
					wy[i][_id-1].append(nodes[i][_id]["y"][1:])
					wyaw[i][_id-1].append(nodes[i][_id]["yaw"][1:])
			else:
				wx[i].append(nodes[i][_id]["x"][1:])
				wy[i].append(nodes[i][_id]["y"][1:])
				wyaw[i].append(nodes[i][_id]["yaw"][1:])
		if i == 'parking':
			for j in range(0,5,2):
				wx[i][j] = np.concatenate(wx[i][j])
				wy[i][j] = np.concatenate(wy[i][j])
				wyaw[i][j] = np.concatenate(wyaw[i][j])
		else:
			wx[i] = np.concatenate(wx[i])
			wy[i] = np.concatenate(wy[i])
			wyaw[i] = np.concatenate(wyaw[i])

	# ws = np.zeros(wx.shape)
	# for i in range(len(ws)):
	# 	x = wx[i]
	# 	y = wy[i]
	# 	sd = get_frenet(x, y, wx, wy)
	# 	ws[i] = sd[0]
 
	waypoints={'global':[],'parking':{0:[],2:[],4:[]}}
	for i in nodes:
		if i == 'parking':
			for j in range(0,5,2):
				waypoints[i][j] = interpolate_waypoints(wx[i][j], wy[i][j], space=0.5)
		else:
			waypoints[i] = interpolate_waypoints(wx[i], wy[i], space=0.5)

	mapx={'global':[],'parking':{0:[],2:[],4:[]}}
	mapy={'global':[],'parking':{0:[],2:[],4:[]}}
	mapyaw={'global':[],'parking':{0:[],2:[],4:[]}}
	maps={'global':[],'parking':{0:[],2:[],4:[]}}

	for i in waypoints:
		if i == 'parking':
			for j in range(0,5,2):
				mapx[i][j] = waypoints[i][j]["x"]
				mapy[i][j] = waypoints[i][j]["y"]
				mapyaw[i][j] = waypoints[i][j]["yaw"]
				maps[i][j] = waypoints[i][j]["s"]
		else:
			mapx[i] = waypoints[i]["x"]
			mapy[i] = waypoints[i]["y"]
			mapyaw[i] = waypoints[i]["yaw"]
			maps[i] = waypoints[i]["s"]

	v=0
	prev_ind={'global':0,'parking':0}
	# ind = 10
	target_speed = {'global':10.0 / 3.6, 'parking': 8.0/3.6}
	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
	state.x=obj_msg.x
	state.y=obj_msg.y
	state.yaw=obj_msg.yaw
	#############
	# state.v=obj_msg.v
	#############
	# msg = state.get_ros_msg(0, 0, 1.0) #a,steer,v
	# control_pub.publish(msg)
 	#state = obj_car
	v_list.append(state.v)
	my_wp={'global':0,'parking':0}
	#my_wp = get_closest_waypoints(state.x, state.y, mapx[:100], mapy[:100],my_wp)
	mode='global' ### global인지 parking인지 subscribe 한다고 치면..
	move_mode='forward'
	# rospy.set_param('car_mode', move_mode)
	my_wp[mode] = get_closest_waypoints(state.x, state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
	waypoint_msg=waypoint_topic(my_wp[mode])
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(10)
	ai = 0

	if my_wp[mode] >= (link_len[mode][link_ind[mode]]):
		# rospy.set_param('move_mode', 'finish')
		move_mode='finish'
		print("finish!")
		fin_wp=[my_wp[mode], link_ind[mode]+1]
		# link_ind[mode]+=1

	link_ind[mode]=find_link(link_len[mode], my_wp[mode])
	
	if fin_wp == [my_wp[mode], link_ind[mode]]:
		move_mode='finish'
	else:
		move_mode='forward'
		
	mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))

	# if (fin_wp!=0) and (fin_wp != my_wp[mode])and (mode!='parking'):
	# 	rospy.set_param('move_mode', 'forward')
	# 	fin_wp=0

	prev_ind[mode] = link_ind[mode]-2
	# s, d = get_frenet(state.x, state.y, mapx[:100], mapy[:100],my_wp)
	# x, y, road_yaw = get_cartesian(s, d, mapx[:100], mapy[:100],maps[:100])
	s, d = get_frenet(state.x, state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
	x, y, road_yaw = get_cartesian(s, d, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],maps[mode][:link_len[mode][link_ind[mode]]])
	
	yawi = state.yaw - road_yaw
	si = s
	si_d = state.v * math.cos(yawi)
	si_dd = ai * math.cos(yawi)
	sf_d = target_speed[mode]
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
		# if (mode == 'global') & (my_wp == 150): ################parking mode 시작 웨이포인트 넣기
		# 	mode='parking'
		# 	rospy.set_param('car_mode', mode)
		# elif (mode=='parking') & ((link_ind[mode]==0)or(link_ind[mode]==2)or(link_ind[mode]==4)or(link_ind[mode]==6)) & (my_wp==10): ################parking curve 시작 웨이포인트 넣기
		# 	rospy.set_param('move_mode', 'forward')
		# elif (mode=='parking') & ((link_ind[mode]==1)or(link_ind[mode]==3)or(link_ind[mode]==5)or(link_ind[mode]==7)):
		# 	rospy.set_param('move_mode', 'backward')
		if (mode == 'global') and ((my_wp[mode] >= 240) and (my_wp[mode] < 245)): ################parking mode 시작 웨이포인트 넣기
			# print("11111!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			prev_park_ind=0
			for park_i in range(0,5,2):
				fp=ParkingPath()
				fp.x=mapx['parking'][park_i][:link_len['parking'][park_i]]
				fp.y=mapy['parking'][park_i][:link_len['parking'][park_i]]
				fp.yaw=mapyaw['parking'][park_i][:link_len['parking'][park_i]]			
    
				ways = []
				way = {
					"x" : fp.x,
					"y" : fp.y
				}	
				ways.append(way)
				# parking_path.make_marker_array(ways)
				prev_park_ind=park_i
				print("부딪힘: "+str(collision_check(fp,obs_info,0,0,0)))
				if collision_check(fp,obs_info,0,0,0)==False:
					link_ind['parking']=park_i
					# state = State(x=mapx['parking'][ind], y=mapy['parking'][ind], yaw=mapyaw['parking'][ind], v=1, dt=0.1)
					print("choose: "+str(park_i))
					# move_mode='finish'
					fin_wp == [my_wp['parking'], link_ind['parking']]
					# rospy.set_param('car_mode', mode)
					mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
					mode='parking'
					break
		if (mode=="parking"):
			if ((my_wp[mode]>=10) and (my_wp[mode]<20)):
				move_mode='forward'
		# elif mode == 'parking' and 
		# 		move_mode='forward'
				# mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
				# rospy.set_param('move_mode', 'forward')
			# elif (link_ind[mode]%2==1) and (my_wp[mode]>=23):
			# 	rospy.set_param('move_mode', 'backward')

		if mode =='parking':
			path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]], maps[mode][park_i][:link_len[mode][park_i]], opt_d, target_speed[mode])
		else:
			path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, mapx[mode], mapy[mode], maps[mode], opt_d, target_speed[mode])

		# update state with acc, delta
		if opt_ind == -1: ## No solution!
			if mode =='parking':
				my_wp[mode] = get_closest_waypoints(state.x,state.y, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]], my_wp[mode])
				waypoint_msg=waypoint_topic(my_wp[mode])
			else:
				my_wp[mode] = get_closest_waypoints(state.x,state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
				dir=find_dir(link_dir, link_ind[mode])
				waypoint_msg=waypoint_topic(my_wp[mode])
				
			
			if (mode == 'global') and (my_wp[mode] >= (link_len[mode][link_ind[mode]])):
				# link_ind[mode]=find_link(link_len[mode], my_wp[mode])
				if link_ind[mode]==len(link_len[mode]): #마지막 링크일때
					# rospy.set_param('move_mode', 'finish')
					move_mode='finish'
					mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
					fin_wp = [my_wp[mode], link_ind[mode]]
					# link_ind[mode]=len(link_len[mode])
				# elif ((mode=='parking') and (link_ind['parking']%2==1)): #parking 후진의 마지막 waypoint
				# 	move_mode='finish'
				# 	print("parking finish!")
				# 	my_wp['global'] = get_closest_waypoints(state.x,state.y, mapx['global'][:link_len['global'][link_ind['global']]], mapy['global'][:link_len['global'][link_ind['global']]],my_wp['global'])
				# 	fin_wp = [my_wp['global'], link_ind['global']]
				# 	mode = 'global'
				else:
					move_mode='finish'
					mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
					print("finish!")
					fin_wp=[my_wp[mode], link_ind[mode]+1]
					# link_ind[mode]+=1
			# elif (mode == 'parking') and (link_ind['parking']%2==0) and (my_wp[mode]>=21):
			# 	move_mode='finish'
			# 	print("finish!")
			# 	fin_wp=[my_wp[mode], link_ind[mode]+1]
			# 	link_ind[mode]+=1
			elif (mode == 'parking') and (my_wp[mode]>=21):
				move_mode='finish'
				print("finish!")
				mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
				fin_wp=[my_wp[mode], link_ind[mode]+1]
				if (link_ind['parking']%2==0):
					link_ind[mode]+=1
			elif ((mode=='parking') and (link_ind['parking']%2==1)) and (my_wp[mode]<=19): #parking 후진의 마지막 waypoint
				move_mode='finish'
				print("parking finish!")
				my_wp['global'] = get_closest_waypoints(state.x,state.y, mapx['global'][:link_len['global'][link_ind['global']]], mapy['global'][:link_len['global'][link_ind['global']]],my_wp['global'])
				fin_wp = [my_wp['global'], link_ind['global']]
				msg.drive.jerk=200
				control_pub.publish(msg)
				rospy.sleep(5)
				mode = 'global'
			# elif (mode == 'parking') and (link_ind['parking']%2==0) and (my_wp[mode]>=23):
			# 	move_mode='finish'
			# 	print("finish!")
			# 	fin_wp=[my_wp[mode], link_ind[mode]+1]
			# 	link_ind[mode]+=1
			
			link_ind[mode]=find_link(link_len[mode], my_wp[mode])
   
			if fin_wp == [my_wp[mode], link_ind[mode]]:
				move_mode='finish'
				mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
			# elif (mode == 'parking') and (fin_wp[0] <= my_wp[mode]):
			# 	if my_wp[mode] < 10:
			# 		move_mode='finish'
			else:
				move_mode='forward'
				msg.drive.jerk=0

			mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))

			prev_ind = link_ind[mode]-2
			# rospy.set_param('dir_mode', [dir, find_dir(link_dir, (link_ind[mode]+1))])
			# rospy.set_param('dir_mode', [dir, find_dir(link_dir, (link_ind[mode]+1))])
			print("현재 링크 번호: "+ str(link_ind[mode])+", mode: "+str(mode)+", 링크 방향: "+str(dir))
			# if (fin_wp!=0) and (fin_wp != my_wp[mode]) and (mode!='parking'):
			# 	rospy.set_param('move_mode', 'forward')
			# 	fin_wp=0
    
			if mode == 'parking':
				s, d = get_frenet(state.x, state.y, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]],my_wp[mode])
				x, y, road_yaw = get_cartesian(s, d, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]],maps[mode][park_i][:link_len[mode][park_i]])
			else:
				s, d = get_frenet(state.x, state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
				x, y, road_yaw = get_cartesian(s, d, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],maps[mode][:link_len[mode][link_ind[mode]]])
			steer = road_yaw - state.yaw
			a = 0
			opt_d = prev_opt_d
		else:
			## PID control

			error_pa = target_speed[mode] - state.v
			error_da = state.v - prev_v
			error_ia += target_speed[mode] - state.v
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
		state.update(a, steer)
		# if ((my_wp[mode] < (link_len[mode][-1] -10)) and (obj_msg.v <= 1)):
		# 	msg = state.get_ros_msg(0, steer, 1.0)
		# else:
		# 	msg = state.get_ros_msg(a, steer, obj_msg.v)
		# control_pub.publish(msg)
		#a_list.append(a)
		#steer_list.append(steer)
		#v_list.append(v)
		msg = state.get_ros_msg(a, steer, id=id)
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer) + ",a = "+str(a))
		prev_v = state.v
		state.x=obj_msg.x
		state.y=obj_msg.y
		state.yaw=obj_msg.yaw
		# state.v=obj_msg.v
		if mode =='parking':
			my_wp[mode] = get_closest_waypoints(state.x,state.y, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]], my_wp[mode])
			waypoint_msg=waypoint_topic(my_wp[mode])
		else:
			my_wp[mode] = get_closest_waypoints(state.x,state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
			dir=find_dir(link_dir, link_ind[mode])
			waypoint_msg=waypoint_topic(my_wp[mode])

		if (mode == 'global') and (my_wp[mode] >= (link_len[mode][link_ind[mode]])):
			if link_ind[mode]==len(link_len[mode]): #마지막 링크일때
				# rospy.set_param('move_mode', 'finish')
				move_mode='finish'
				fin_wp = [my_wp[mode], link_ind[mode]]
				# link_ind[mode]=len(link_len[mode])
			# elif ((mode=='parking') and (link_ind['parking']%2==1)): #parking 후진의 마지막 waypoint
			# 	move_mode='finish'
			# 	print("parking finish!")
			# 	my_wp['global'] = get_closest_waypoints(state.x,state.y, mapx['global'][:link_len['global'][link_ind['global']]], mapy['global'][:link_len['global'][link_ind['global']]],my_wp['global'])
			# 	fin_wp = [my_wp['global'], link_ind['global']]
			# 	mode = 'global'
			else:
				move_mode='finish'
				print("finish!")
				mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
				fin_wp=[my_wp[mode], link_ind[mode]+1]
				# link_ind[mode]+=1
		# elif (mode == 'parking') and (link_ind['parking']%2==0) and (my_wp[mode]>=21):
		# 	move_mode='finish'
		# 	print("finish!")
		# 	fin_wp=[my_wp[mode], link_ind[mode]+1]
		# 	link_ind[mode]+=1
		elif (mode == 'parking') and (my_wp[mode]>=21):
			move_mode='finish'
			print("finish!")
			mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
			fin_wp=[my_wp[mode], link_ind[mode]+1]
			if (link_ind['parking']%2==0):
				link_ind[mode]+=1
		elif ((mode=='parking') and (link_ind['parking']%2==1)) and (my_wp[mode]<=19): #parking 후진의 마지막 waypoint
			move_mode='finish'
			print("parking finish!")
			my_wp['global'] = get_closest_waypoints(state.x,state.y, mapx['global'][:link_len['global'][link_ind['global']]], mapy['global'][:link_len['global'][link_ind['global']]],my_wp['global'])
			fin_wp = [my_wp['global'], link_ind['global']]
			msg.drive.jerk=200
			control_pub.publish(msg)
			rospy.sleep(5)
			mode = 'global'

		link_ind[mode]=find_link(link_len[mode], my_wp[mode])
  
		if (fin_wp == [my_wp[mode], link_ind[mode]]):
			move_mode='finish'
		# elif (mode == 'parking') and (fin_wp[0] <= my_wp[mode]):
		# 		if my_wp[mode] < 10:
		# 			move_mode='finish'
		else:
			move_mode='forward'
			msg.drive.jerk=0

		mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))

		# if (fin_wp!=0) and (fin_wp != my_wp[mode]) and (mode!='parking'):
		# 	rospy.set_param('move_mode', 'forward')
		# 	fin_wp=0
		# prev_ind = link_ind[mode]-2

		# rospy.set_param('dir_mode', [dir, find_dir(link_dir, (link_ind[mode]+1))])
		# rospy.set_param('dir_mode', [dir, find_dir(link_dir, (link_ind[mode]+1))])
		print("현재 링크 번호: "+ str(link_ind[mode])+", mode: "+str(mode)+", 링크 방향: "+str(dir))

		# if my_wp[mode] == 270:
		# 	with open("/home/nsclmds/a_list.text", "wb") as f:
		# 		pickle.dump(a_list, f)
		# 	with open("/home/nsclmds/v_list.text", "wb") as f:
		# 		pickle.dump(v_list, f)
		# 	with open("/home/nsclmds/steer_list.text", "wb") as f:
		# 		pickle.dump(steer_list, f)
    
		if mode == 'parking':
			s, d = get_frenet(state.x, state.y, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]],my_wp[mode])
			x, y, road_yaw = get_cartesian(s, d, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]],maps[mode][park_i][:link_len[mode][park_i]])
		else:
			s, d = get_frenet(state.x, state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
			x, y, road_yaw = get_cartesian(s, d, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],maps[mode][:link_len[mode][link_ind[mode]]])
		
  		# yaw_diff = state.yaw - road_yaw
		yaw_diff = state.yaw - road_yaw

		si = s
		si_d = state.v * math.cos(yaw_diff)
		si_dd = ai * math.cos(yaw_diff)
		sf_d = target_speed[mode]
		sf_dd = 0
		
		di = d
		di_d = state.v * math.sin(yaw_diff)
		di_dd = ai * math.sin(yaw_diff)
		df_d = 0
		df_dd = 0

		# if (mode == 'global') and ((my_wp >= 240) and (my_wp < 250)): ################parking mode 시작 웨이포인트 넣기
		# 	mode='parking'
		# 	rospy.set_param('car_mode', mode)
			# for park_i in range(0,5,2):
			# 	if collision_check([mapx[mode][:link_len[mode][park_i]], mapy[mode][:link_len[mode][park_i]],mapyaw[mode][:link_len[mode][park_i]]],obs_info,0,0,0)==False:
			# 		link_ind[mode]==park_i
			# 		break
			# print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
		# elif (mode=='parking') and (link_ind[mode]==0) and (my_wp==10): ################parking curve 시작 웨이포인트 넣기
		# 	rospy.set_param('move_mode', 'forward')
		# elif (mode=='parking') and (link_ind[mode]==1):
		# 	rospy.set_param('move_mode', 'backward')
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
		# object_pub.publish(msg["object_msg"])
		opt_frenet_pub.publish(opt_frenet_path.ma)
		cand_frenet_pub.publish(cand_frenet_paths.ma)
		control_pub.publish(msg)
		mode_pub.publish(mode_msg)
		waypoint_pub.publish(waypoint_msg)

		r.sleep()
