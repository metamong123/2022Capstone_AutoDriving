#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import rospkg
import sys
from ackermann_msgs.msg import AckermannDriveStamped

from scipy.interpolate import interp1d

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object, ObjectArray
from std_msgs.msg import Float64
from rocon_std_msgs.msg import StringArray

import pickle
import argparse

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")
from global_path import *
use_map=kcity()
mode='global'

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


obj_msg = Object(x=962587.11409, y=1959260.09207, yaw=1.2871297862692013, v=1,L=1.600, W=1.04)
obs_info = []
class TopicReciver:
	def __init__(self):
		self.state_sub = rospy.Subscriber("/objects/car_1", Object, self.callback2, queue_size=1)
	def check_all_connections(self):
		return (self.state_sub.get_num_connections())==1
	def callback2(self, msg):
		if self.check_all_connections():
			global obj_msg
			obj_msg=msg


if __name__ == "__main__":
	WB = 1.04

	cand_frenet_pub = rospy.Publisher("/rviz/candidate_frenet_paths", MarkerArray, queue_size=1)
	control_pub = rospy.Publisher("/ackermann_cmd_frenet", AckermannDriveStamped, queue_size=1)
	error_icte=0
	prev_cte =0
	cte = 0

	v=0
	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
	
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(10)
	ai = 0

	while not rospy.is_shutdown():
		##PATH랑 opt_ind, road_yaw, mode 받기..!!
		if opt_ind == -1: ## No solution!
			steer = road_yaw - state.yaw
			a = 0
		else:
			## PID control

			error_pa = use_map.target_speed[mode] - state.v
			error_da = state.v - prev_v
			error_ia += use_map.target_speed[mode] - state.v
			kp_a = 0.5
			kd_a = 0.7
			ki_a = 0.01
			a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia
			prev_cte = cte
			error_icte += cte
			
			steer, cte, _ = stanley_control(state.x, state.y, state.yaw, state.v, path[opt_ind].x, path[opt_ind].y, path[opt_ind].yaw, WB, error_icte, prev_cte)
			
		
		ai=a
		state.update(a, steer)
		
		msg = state.get_ros_msg(a, steer, id=id)
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer) + ",a = "+str(a))
		prev_v = state.v
		state.x=obj_msg.x
		state.y=obj_msg.y
		state.yaw=obj_msg.yaw
		# state.v=obj_msg.v
		if mode =='parking':
			my_wp[mode] = get_closest_waypoints(state.x,state.y, mapx[mode][park_i][:link_len[mode][park_i]], mapy[mode][park_i][:link_len[mode][park_i]], my_wp[mode])
		else:
			my_wp[mode] = get_closest_waypoints(state.x,state.y, mapx[mode][:link_len[mode][link_ind[mode]]], mapy[mode][:link_len[mode][link_ind[mode]]],my_wp[mode])
			dir=find_dir(link_dir, link_ind[mode])

		if my_wp[mode] >= (link_len[mode][link_ind[mode]]-10):
			if link_ind[mode]==len(link_len[mode]): #마지막 링크일때
				# rospy.set_param('move_mode', 'finish')
				move_mode='finish'
				fin_wp = [my_wp[mode], link_ind[mode]]
				link_ind[mode]=len(link_len[mode])
			elif ((mode=='parking') and (link_ind['parking']%2==1)): #parking 후진의 마지막 waypoint
				move_mode='finish'
				print("parking finish!")
				my_wp['global'] = get_closest_waypoints(state.x,state.y, mapx['global'][:link_len['global'][link_ind['global']]], mapy['global'][:link_len['global'][link_ind['global']]],my_wp['global'])
				fin_wp = [my_wp['global'], link_ind['global']]
				mode = 'global'
			else:
				move_mode='finish'
				print("finish!")
				fin_wp=[my_wp[mode], link_ind[mode]+1]
				link_ind[mode]+=1
		elif (mode == 'parking') and (link_ind['parking']%2==0) and (my_wp[mode]==parking_stop[park_i]):
			move_mode='finish'
			print("finish!")
			fin_wp=[my_wp[mode], link_ind[mode]+1]
			link_ind[mode]+=1
		elif ((mode=='parking') and (link_ind['parking']%2==1)) and (my_wp[mode]<=park_to_global[park_i]): #parking 후진의 마지막 waypoint
				move_mode='finish'
				print("parking finish!")
				my_wp['global'] = get_closest_waypoints(state.x,state.y, mapx['global'][:link_len['global'][link_ind['global']]], mapy['global'][:link_len['global'][link_ind['global']]],my_wp['global'])
				fin_wp = [my_wp['global'], link_ind['global']]
				mode = 'global'
		elif (mode=='global'):
			for i in range(len(stopline_wp)):
				if my_wp[mode]==stopline_wp[i]:
					move_mode='finish'
					print("finish!")

		if fin_wp == [my_wp[mode], link_ind[mode]]:
			move_mode='finish'
		else:
			move_mode='forward'

		mode_msg=mode_array(mode, move_mode, find_dir(link_dir, link_ind[mode]), find_dir(link_dir, (link_ind[mode]+1)))
		# lane_msg=lane_width_msg(find_dir(lane_width, link_ind[mode]))

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
		sf_d = use_map.target_speed[mode]
		sf_dd = 0
		
		di = d
		di_d = state.v * math.sin(yaw_diff)
		di_dd = ai * math.sin(yaw_diff)
		df_d = 0
		df_dd = 0

		# if (mode == 'global') and ((my_wp >= 240) and (my_wp < 250)): ################parking mode 시작 웨이포인트 넣기
		# 	mode='parking'
		# 	rospy.set_param('car_mode', mode)
			# for park_i in range(0, 11, 2):
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
		# lane_width_pub.publish(lane_width_msg)

		r.sleep()