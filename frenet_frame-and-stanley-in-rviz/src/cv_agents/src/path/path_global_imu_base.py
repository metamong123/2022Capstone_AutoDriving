#!/usr/bin/python
#-*- coding: utf-8 -*-
import pickle
import rospy
import sys
import rospkg
import numpy as np
import math
from scipy.interpolate import interp1d
import time

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")
from map_visualizer import Converter

from visualization_msgs.msg import MarkerArray
from object_msgs.msg import Object, ObjectArray, PathArray
from std_msgs.msg import Float64, Int32MultiArray, String, Int32
from rocon_std_msgs.msg import StringArray

from frenet import *
from path_map import *

## 초기화 지점
mode='global'
# obj_msg=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=0,v=0,L=1.600,W=1.04)

def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi

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

ai=0
obs_info = []
class TopicReciver:
	def __init__(self):
		self.obstacle_sub = rospy.Subscriber("obstacles", ObjectArray, self.callback_obstacle, queue_size=1)
		self.state_sub = rospy.Subscriber("/objects/car_1/imu", Object, self.callback2, queue_size=1)
		self.accel_sub = rospy.Subscriber("/accel", Float64, self.callback3, queue_size=1)
	def check_all_connections(self):
		return (self.obstacle_sub.get_num_connections()+self.state_sub.get_num_connections()+self.accel_sub.get_num_connections())==3
	def callback_obstacle(self, msg):
		if self.check_all_connections():
			global obs_info
			obs_info = []
			for o in msg.object_list:
				obj = [o.x, o.y, o.yaw, o.L, o.W]
				obs_info.append(obj)
	def callback2(self, msg):
		if self.check_all_connections():
			global obj_msg
			global t1
			obj_msg=msg
			t1=time.time()
	def callback3(self, msg):
		if self.check_all_connections():
			global ai
			ai=msg.data



def callback_obstacle(msg):
	global obs_info
	obs_info = []
	for o in msg.object_list:
		obj = [o.x, o.y, o.yaw, o.L, o.W]
		obs_info.append(obj)

def callback2(msg):
	global obj_msg
	global t1
	obj_msg=msg
	t1=time.time()

def callback3(msg):
	global ai
	ai=msg.data


def callback_mode(msg):
	global mode
	if (msg.data == 'delivery_A') or (msg.data == 'delivery_B'):
		mode = 'delivery'
	elif (msg.data == 'diagonal_parking') or (msg.data == 'horizontal_parking'):
		mode = 'parking'
	else:
		mode = msg.data
	print(mode)


def direction_array(current_dir, next_dir):
	m = StringArray()
	m.strings=[current_dir, next_dir]
	return m

def find_dir(link_dict, link_ind):
	for i in link_dict.keys():
		for j in link_dict[i]:
			if link_ind == j:
				return i

def path_array(x, y, yaw):
	p=PathArray()
	p.x.data= x
	p.y.data= y
	p.yaw.data = yaw
	return p

def my_state_array(ind, wp):
	m = Int32MultiArray()
	m.data=[ind, wp]
	return m

def sub_and_pub():
	global opt_frenet_pub,cand_frenet_pub,waypoint_pub,dir_pub,global_path_pub,col_pub,col_msg,s,d,x,y,road_yaw,dir,state,si,si_d,si_dd,sf_d,sf_dd,di,di_d,df_dd,di_dd,df_d,opt_d,prev_opt_d,obstacle_sub,state_sub,accel_sub,mode_sub
	
	obstacle_sub = rospy.Subscriber("obstacles", ObjectArray, callback_obstacle, queue_size=1)
	state_sub = rospy.Subscriber("/objects/car_1/imu", Object, callback2, queue_size=1)
	accel_sub = rospy.Subscriber("/accel", Float64, callback3, queue_size=1)

	mode_sub=rospy.Subscriber("/mode_selector", String, callback_mode, queue_size=10)


	opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
	cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
	waypoint_pub = rospy.Publisher("/waypoint", Int32MultiArray, queue_size=1)
	dir_pub=rospy.Publisher("/link_direction", StringArray, queue_size=1)
	global_path_pub=rospy.Publisher("/optimal_frenet_path_global", PathArray, queue_size=1)
	col_pub=rospy.Publisher("/col", Int32, queue_size=1)

	col_msg=Int32()

	if mode == 'global':
		dir=find_dir(use_map.link_dir, link_ind['global'])
		if dir == 'right' or dir == 'left':
			dir='curve'
	else:
		dir = 'straight'
	
	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)

	s, d = get_frenet(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])
	x, y, road_yaw = get_cartesian(s, d, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],use_map.waypoints['global']['s'][:use_map.link_len['global'][link_ind['global']]])

	yawi = state.yaw - road_yaw
	si = s
	si_d = state.v * math.cos(yawi)
	si_dd = ai * math.cos(yawi)
	sf_d = use_map.target_speed[mode][dir]
	sf_dd = 0

	di = d
	di_d = state.v * math.sin(yawi)
	di_dd = ai * math.sin(yawi)
	df_d = 0
	df_dd = 0

	opt_d = d
	prev_opt_d = d

if __name__ == "__main__":
    
	rospy.init_node("path")
	my_wp={'global':0,'diagonal_parking':{},'horizontal_parking':[]}
	
	link_ind={}
	link_ind['global']=start_index
	opt_ind=0
	
	if mode == 'global':
		dir=find_dir(use_map.link_dir, link_ind['global'])
		if dir == 'right' or dir == 'left':
			dir='curve'
	else:
		dir = 'straight'

	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)

	my_wp['global']=get_closest_waypoints(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])

	mode_msg=direction_array(find_dir(use_map.link_dir, link_ind['global']), find_dir(use_map.link_dir, (link_ind['global']+1)))

	path_msg=path_array([],[],[])
	# print("현재 링크 번호: "+ str(link_ind['global'])+", 링크 방향: "+str(find_dir(use_map.link_dir, link_ind['global'])))

	s, d = get_frenet(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])
	x, y, road_yaw = get_cartesian(s, d, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],use_map.waypoints['global']['s'][:use_map.link_len['global'][link_ind['global']]])

	yawi = state.yaw - road_yaw
	si = s
	si_d = state.v * math.cos(yawi)
	si_dd = ai * math.cos(yawi)
	sf_d = use_map.target_speed[mode][dir]
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
	# r = rospy.Rate(1)

	while not rospy.is_shutdown():

		sub_and_pub()

		path, opt_ind, col = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, use_map.waypoints['global']['x'], use_map.waypoints['global']['y'],use_map.waypoints['global']['s'], opt_d, use_map.target_speed[mode][dir], use_map.DF_SET[link_ind['global']])
		col_msg.data=col

		if opt_ind == -1:
			path_msg=path_array([],[],[])
		else:
			path_msg = path_array(path[opt_ind].x,path[opt_ind].y,path[opt_ind].yaw)

		if opt_ind == -1: ## No solution!
			print("No solution!")
				
			my_wp['global'] = get_closest_waypoints(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])

			if(my_wp['global'] >= (use_map.link_len['global'][link_ind['global']]-10)):
				if link_ind['global']==(len(use_map.link_len['global'])-1):
					pass
				else:
					link_ind['global']+=1

			mode_msg=direction_array(find_dir(use_map.link_dir, link_ind['global']), find_dir(use_map.link_dir, (link_ind['global']+1)))

			if mode == 'global':
				dir=find_dir(use_map.link_dir, link_ind['global'])
				if dir == 'right' or dir == 'left':
					dir='curve'
			else:
				dir = 'straight'

			s, d = get_frenet(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])
			x, y, road_yaw = get_cartesian(s, d, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],use_map.waypoints['global']['s'][:use_map.link_len['global'][link_ind['global']]])

			opt_d = prev_opt_d
		else:
			ways = []
			for p in path:
				way = {
					"x" : p.x,
					"y" : p.y
				}	
				ways.append(way)

			opt_frenet_path.make_marker_array(ways[opt_ind])
			ways_={'x':[], 'y':[]}
			for way in ways:
				for i in range(len(way['x'])):
					ways_['x'].append(way['x'][i])
					ways_['y'].append(way['y'][i])
			cand_frenet_paths.make_marker_array(ways_)

			opt_d = path[opt_ind].d[-1]
			prev_opt_d = path[opt_ind].d[-1]

		my_wp['global'] = get_closest_waypoints(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])

		if (my_wp['global'] >= (use_map.link_len['global'][link_ind['global']]-10)):
			if link_ind['global']==(len(use_map.link_len['global'])-1):
				pass
			else:
				link_ind['global']+=1

		mode_msg=direction_array(find_dir(use_map.link_dir, link_ind['global']), find_dir(use_map.link_dir, (link_ind['global']+1)))
		
		if mode == 'global':
			dir=find_dir(use_map.link_dir, link_ind['global'])
			if dir == 'right' or dir == 'left':
				dir='curve'
		else:
			dir = 'straight'
			
		print("현재 링크 번호: "+ str(link_ind['global'])+", 링크 방향: "+str(find_dir(use_map.link_dir, link_ind['global'])))
		
		waypoint_msg=my_state_array(link_ind['global'], my_wp['global'])
		opt_frenet_pub.publish(opt_frenet_path.ma)
		cand_frenet_pub.publish(cand_frenet_paths.ma)
		dir_pub.publish(mode_msg)
		waypoint_pub.publish(waypoint_msg)
		global_path_pub.publish(path_msg)
		col_pub.publish(col_msg)

		#rospy.sleep(1)