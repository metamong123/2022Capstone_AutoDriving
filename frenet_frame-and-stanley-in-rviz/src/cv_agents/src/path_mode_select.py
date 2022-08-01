#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospkg
import sys
import rospy
import numpy as np
import math, time

from object_msgs.msg import PathArray, ObjectArray
from std_msgs.msg import Float64, Int32MultiArray, String
from rocon_std_msgs.msg import StringArray

from frenet import *
from stanley_pid import *

##################### load path ###############################

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")

use_map=kcity()

global_path_x=[]
global_path_y=[]
global_path_yaw=[]
def global_path_callback(msg):
	global global_path_x, global_path_y, global_path_yaw
	global_path_x = msg.x.data
	global_path_y = msg.y.data
	global_path_yaw = msg.yaw.data

###################################################################3

mode = 'global'

global_wp = 0
parking_wp1 = 0
parking_wp2 = 0
parking_wp3 = 0
parking_wp4 = 0
parking_wp5 = 0
parking_wp6 = 0
def waypoint_callback(msg):
	global global_wp, parking_wp1, parking_wp2, parking_wp3, parking_wp4, parking_wp5, parking_wp6
	global_wp = msg.data[0]  #global waypoint
	parking_wp1 = msg.data[1] #parking waypoint
	parking_wp2 = msg.data[2]
	parking_wp3 = msg.data[3]
	parking_wp4 = msg.data[4]
	parking_wp5 = msg.data[5]
	parking_wp6 = msg.data[6]
 
current_dir = 0
next_dir = 0
def link_callback(msg):
	global current_dir, next_dir
	current_dir = msg.data[0]
	next_dir = msg.data[1]
    
mode_status = 'going'
def finish_callback(msg):
    global mode_status
    mode_status = msg.data

######## 주차칸에 차량의 위치 파악 ############
def obstacle_callback(msg):
	global obs_info
	obs_info = []
	for o in msg.object_list:
		obj = [o.x, o.y, o.yaw, o.L, o.W]
		obs_info.append(obj)
##########################################

def mode_array(car_mode, current_dir, next_dir):
	m = StringArray()
	m.strings=[car_mode, current_dir, next_dir]
	return m


if __name__ == "__main__":

	rospy.init_node("path_select")

	rospy.Subscriber("/optimal_frenet_path_global", PathArray, global_path_callback)
	rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
	rospy.Subscriber("/link_direction", StringArray, link_callback)
	rospy.Subscriber("/obstacles", ObjectArray, obstacle_callback)

	mode_pub=rospy.Publisher("/mode_selector", StringArray, queue_size=1)
	path_pub = rospy.Publisher("/final_path", PathArray, queue_size=1)

	path_msg = PathArray()
 
	while not rospy.is_shutdown():

		######## mode select based waypoint #######
		if (global_wp <= __ and global_wp >= __):  # parking mode
			mode = 'parking'
			parking_ind = 0  # 초기에 for문 delay 방지를 위해 0으로 정의
		elif (global_wp <= __ and global_wp >= __):  # delivery mode A
			mode = 'delivery_A'
		elif (global_wp <= __ and global_wp >= __):  # delivery mode B
			mode = 'delivery_B'
		else:
			mode = 'global'
   
        ### 미션이 끝나면 finish flag를 받아 global path 로 복귀 ##
		if mode_status == 'finish':
			mode = 'global'
		else:
			pass
		#######################################################

		mode_msg = mode_array(mode, current_dir, next_dir)  # 현재 모드와 link상태를 같이 보냄

		###### 현재 mode 에 따른 final path를 보내줌 ##########
		if mode == 'global':
			path_msg.x.data = global_path_x
			path_msg.y.data = global_path_y
			path_msg.yaw.data = global_path_yaw

		elif mode == 'parking':
			for park_i in range(use_map.parking_map_num):
				
				fp = MakingPath()
				fp.x=use_map.parking_path[park_i].x
				fp.y=use_map.parking_path[park_i].y
				fp.yaw=use_map.parking_path[park_i].yaw
				print("위치" + str(park_i) + "번 차량존재유무 : " + str(collision_check(fp,obs_info,0,0,0)))

				if collision_check(fp,obs_info,0,0,0)==False:
					link_ind['parking']=park_i
					parking_ind=park_i
					print("parking_choose: "+str(park_i))
					break

			if collision_check(fp,obs_info,0,0,0)==True:
				for park_i in range(parking_ind,use_map.parking_map_num,1):
					fp_1=MakingPath()
					fp_1.x=use_map.parking_path[park_i].x
					fp_1.y=use_map.parking_path[park_i].y
					fp_1.yaw=use_map.parking_path[park_i].yaw
					print("위치" + str(park_i) + "번 차량존재유무 : " + str(collision_check(fp_1,obs_info,0,0,0)))

					if collision_check(fp_1,obs_info,0,0,0)==False:
						link_ind['parking']=park_i
						parking_ind=park_i
						print("parking_choose: "+str(park_i))
						fp=fp_1
						break
			path_msg.x.data = use_map.parking_path[parking_ind].x  # parking final path
			path_msg.y.data = use_map.parking_path[parking_ind].y
			path_msg.yaw.data = use_map.parking_path[parking_ind].yaw
  
		elif mode == 'delivery_A':
			path_msg.x.data = use_map.delivery_path[0].x  # A path
			path_msg.y.data = use_map.delivery_path[0].y
			path_msg.yaw.data = use_map.delivery_path[0].yaw
		elif mode == 'delivery_B':
			path_msg.x.data = use_map.delivery_path[1].x  # B path
			path_msg.y.data = use_map.delivery_path[1].y
			path_msg.yaw.data = use_map.delivery_path[1].yaw
   
		################################################################
   
		mode_pub.publish(mode_msg)
		path_pub.publish(path_msg)

		#r.sleep()
	
