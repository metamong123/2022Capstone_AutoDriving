#!/usr/bin/python
#-*- coding: utf-8 -*-
import pickle
import rospy
import sys
import rospkg
import numpy as np
import math
from scipy.interpolate import interp1d

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")
sys.path.append(path_frenet+"/src/path/")
from map_visualizer import Converter

from visualization_msgs.msg import MarkerArray
from object_msgs.msg import Object, ObjectArray
from std_msgs.msg import Float64, Int32MultiArray, Float64MultiArray
from rocon_std_msgs.msg import StringArray

from frenet import *
from global_path import *

## 초기화 지점
use_map=kcity()
mode='global'
start_index=0
obj_msg=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=0,v=0,L=1.600,W=1.04)

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
		self.state_sub = rospy.Subscriber("/objects/car_1", Object, self.callback2, queue_size=1)
		self.accel_sub = rospy.Subscriber("/accelerartion", Float64, self.callback3, queue_size=1)
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
			obj_msg=msg
	def callback3(self, msg):
		if self.check_all_connections():
			global ai
			ai=msg

def mode_array(car_mode, current_dir, next_dir):
	m = StringArray()
	m.strings=[car_mode, current_dir, next_dir]
	return m

def find_dir(link_dict, link_ind):
	for i in link_dict.keys():
		for j in link_dict[i]:
			if link_ind == j:
				return i

def path_array(opt_ind,x, y, yaw):
	p=Float64MultiArray()
	p.data=[opt_ind,x,y,yaw]
	return p

def my_state_array(wp, ind):
	m = Int32MultiArray()
	m.data=[wp, ind]
	return m

if __name__ == "__main__":
    
	rospy.init_node("path")

	topic_receiver=TopicReciver()

	opt_frenet_pub = rospy.Publisher("/rviz/optimal_frenet_path", MarkerArray, queue_size=1)
	cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
	waypoint_pub = rospy.Publisher("/waypoint", Int32MultiArray, queue_size=1)
	mode_pub=rospy.Publisher("/mode_selector", StringArray, queue_size=1)
	path_pub=rospy.Publisher("/optimal_frenet_path", Float64MultiArray, queue_size=1)

	my_wp={'global':0, 'parking':0,'delivery':0}
	link_ind={}
	link_ind[mode]=start_index
	
	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
	my_wp[mode]=get_closest_waypoints(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])

	if mode=='global':
		my_wp[mode] = get_closest_waypoints(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
	else:
		my_wp[mode] = get_closest_waypoints(state.x, state.y, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])

	mode_msg=mode_array(mode, find_dir(use_map.link_dir, link_ind[mode]), find_dir(use_map.link_dir, (link_ind[mode]+1)))

	print("현재 링크 번호: "+ str(link_ind[mode])+", mode: "+str(mode)+", 링크 방향: "+str(find_dir(use_map.link_dir, link_ind[mode])))

	if mode == 'global':
		s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
		x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind[mode]]])
	else:
		s, d = get_frenet(state.x, state.y, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
		x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode][link_ind[mode]]['s'][:use_map.link_len[mode][link_ind[mode]]])

	yawi = state.yaw - road_yaw
	si = s
	si_d = state.v * math.cos(yawi)
	si_dd = ai * math.cos(yawi)
	sf_d = use_map.target_speed[mode]
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
		
		if (mode == 'global') and ((my_wp[mode] >= use_map.glo_to_park_start) and (my_wp[mode] < use_map.glo_to_park_finish)):
			for park_i in range(0,use_map.parking_map_num*2,2):

				ways = []
				way = {
					"x" : use_map.waypoints['parking'][park_i]['x'][:use_map.link_len['parking'][park_i]],
					"y" : use_map.waypoints['parking'][park_i]['y'][:use_map.link_len['parking'][park_i]]
				}	
				ways.append(way)

				print("부딪힘: "+str(collision_check(fp,obs_info,0,0,0)))

				if collision_check(fp,obs_info,0,0,0)==False:
					link_ind['parking']=park_i
					print("choose: "+str(park_i))
					mode_msg=mode_array(mode, find_dir(use_map.link_dir, link_ind[mode]), find_dir(use_map.link_dir, (link_ind[mode]+1)))
					mode='parking'
					break

		if mode == 'global':
			path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, use_map.waypoints[mode]['x'], use_map.waypoints[mode]['y'],use_map.waypoints[mode]['s'], opt_d, use_map.target_speed[mode])
			if opt_ind == -1:
				path_msg=path_array(opt_ind,-1,-1,-1)
			else:
				path_msg = path_array(opt_ind, path[opt_ind].x,path[opt_ind].y,path[opt_ind].yaw)
		else:
			path, opt_ind = frenet_optimal_planning(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs_info, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode][link_ind[mode]]['s'][:use_map.link_len[mode][link_ind[mode]]], opt_d, use_map.target_speed[mode])
			if opt_ind == -1:
				path_msg=path_array(opt_ind,-1,-1,-1)
			else:
				path_msg = path_array(opt_ind, path[opt_ind].x,path[opt_ind].y,path[opt_ind].yaw)

		if opt_ind == -1: ## No solution!
			print("No solution!")
			if mode=='global':
				my_wp[mode] = get_closest_waypoints(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
			else:
				my_wp[mode] = get_closest_waypoints(state.x, state.y, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])	
			if (mode == 'global') and (my_wp[mode] >= (use_map.link_len[mode][link_ind[mode]]-10)):
				if link_ind[mode]==len(use_map.link_len[mode]):
					link_ind[mode]=len(link_ind[mode])
				else:
					link_ind[mode]+=1
			elif (mode == 'parking') and (link_ind['parking']%2==0)and (my_wp[mode]>=use_map.parking_stop):
				link_ind[mode]+=1
			elif ((mode=='parking') and (link_ind['parking']%2==1)) and (my_wp[mode]<=use_map.park_to_glo):
				print("parking finish!")
				my_wp['global'] = get_closest_waypoints(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])
				mode = 'global'

			mode_msg=mode_array(mode, find_dir(use_map.link_dir, link_ind[mode]), find_dir(use_map.link_dir, (link_ind[mode]+1)))
			print("현재 링크 번호: "+ str(link_ind[mode])+", mode: "+str(mode)+", 링크 방향: "+str(find_dir(use_map.link_dir, link_ind[mode])))

			if mode == 'global':
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind[mode]]])
			else:
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode][link_ind[mode]]['s'][:use_map.link_len[mode][link_ind[mode]]])

			opt_d = prev_opt_d
		else:
			ways = []
			for p in path:
				way = {
					"x" : p.x,
					"y" : p.y
				}	
				ways.append(way)

			## path msg 자리....

			opt_frenet_path.make_marker_array([ways[opt_ind]])
			cand_frenet_paths.make_marker_array(ways)

			opt_d = path[opt_ind].d[-1]
			prev_opt_d = path[opt_ind].d[-1]

		if mode=='global':
			my_wp[mode] = get_closest_waypoints(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
		else:
			my_wp[mode] = get_closest_waypoints(state.x, state.y, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])	

		if (mode == 'global') and (my_wp[mode] >= (use_map.link_len[mode][link_ind[mode]]-10)):
			if link_ind[mode]==len(use_map.link_len[mode]):
				link_ind[mode]=len(link_ind[mode])
			else:
				link_ind[mode]+=1
		elif (mode == 'parking') and (link_ind['parking']%2==0)and (my_wp[mode]>=use_map.parking_stop):
			link_ind[mode]+=1
		elif ((mode=='parking') and (link_ind['parking']%2==1)) and (my_wp[mode]<=use_map.park_to_glo):
			print("parking finish!")
			my_wp['global'] = get_closest_waypoints(state.x, state.y, use_map.waypoints['global']['x'][:use_map.link_len['global'][link_ind['global']]], use_map.waypoints['global']['y'][:use_map.link_len['global'][link_ind['global']]],my_wp['global'])
			mode = 'global'

		mode_msg=mode_array(mode, find_dir(use_map.link_dir, link_ind[mode]), find_dir(use_map.link_dir, (link_ind[mode]+1)))
		print("현재 링크 번호: "+ str(link_ind[mode])+", mode: "+str(mode)+", 링크 방향: "+str(find_dir(use_map.link_dir, link_ind[mode])))

		if mode == 'global':
			s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
			x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind[mode]]])
		else:
			s, d = get_frenet(state.x, state.y, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],my_wp[mode])
			x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode][link_ind[mode]]['x'][:use_map.link_len[mode][link_ind[mode]]], use_map.waypoints[mode][link_ind[mode]]['y'][:use_map.link_len[mode][link_ind[mode]]],use_map.waypoints[mode][link_ind[mode]]['s'][:use_map.link_len[mode][link_ind[mode]]])

		yaw_diff = state.yaw - road_yaw
		si = s
		si_d = state.v * math.cos(yaw_diff)
		si_dd = ai * math.cos(yaw_diff)
		sf_d = use_map.target_speed[mode]
		sf_dd = 0
		
		di = d
		di_d = state.v * math.sin(yaw_diff)
		di_dd = ai * math.sin(yaw_diff)
		df_d = 0
		df_dd = 0

		waypoint_msg=my_state_array(my_wp[mode], link_ind[mode])

		opt_frenet_pub.publish(opt_frenet_path.ma)
		cand_frenet_pub.publish(cand_frenet_paths.ma)
		mode_pub.publish(mode_msg)
		waypoint_pub.publish(waypoint_msg)
		path_pub.publish(path_msg)

		