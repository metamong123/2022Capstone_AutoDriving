#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospkg
import sys
import rospy
import numpy as np
import math, time

from object_msgs.msg import PathArray, ObjectArray, Object
from std_msgs.msg import Float64, Int32MultiArray, String, Int32
from rocon_std_msgs.msg import StringArray

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
sys.path.append(path_frenet+"/src/")
from frenet import *
from stanley_pd import *
from path_map import *

# delivery mission
# horizonal parking mission

##################### load path ###############################

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

link_ind = 0
global_wp = 0
def waypoint_callback(msg):
	global  link_ind, global_wp
	link_ind = msg.data[0]   #link index
	global_wp = msg.data[1]  #global waypoint
 
current_dir = 'straight'
next_dir = 'straight'
def link_callback(msg):
	global current_dir, next_dir
	current_dir = msg.strings[0]
	next_dir = msg.strings[1]

    
mode_status ="going"
def end_callback(msg):
	global mode_status
	print("1")
	mode_status = msg.data
	print(mode_status)


######## 주차칸에 차량의 위치 파악 ############
obs_info=[]
def obstacle_callback(msg):
	global obs_info
	obs_info = []
	for o in msg.object_list:
		obj = [o.x, o.y, o.yaw, o.L, o.W]
		obs_info.append(obj)
##########################################

state_x=0
state_y=0
def state_callback(msg):
	global state_x, state_y
	state_x = msg.x
	state_y = msg.y

if __name__ == "__main__":

	rospy.init_node("path_select")

	rospy.Subscriber("/optimal_frenet_path_global", PathArray, global_path_callback)
	rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
	rospy.Subscriber("/link_direction", StringArray, link_callback)
	rospy.Subscriber("/obstacles", ObjectArray, obstacle_callback)
	rospy.Subscriber("/mission_status", String, end_callback)
	rospy.Subscriber("/objects/car_1", Object, state_callback, queue_size=1)
	mode_pub = rospy.Publisher("/mode_selector", String, queue_size=10)
	path_pub = rospy.Publisher("/final_path", PathArray, queue_size=1)
	park_pub = rospy.Publisher("/park_ind_wp", Int32MultiArray, queue_size=1)
	traffic_pub = rospy.Publisher("/traffic_mode", String, queue_size=1)

	path_msg = PathArray()
	mode_msg = String()
	park_msg = Int32MultiArray()
	traffic_msg = String()

	parking_ind = 0
	park_wp = 0
	
	mode='global'
	traffic_mode = 'no'
	dist = 0
	traffic_interval = 0
	target_speed = 0
	#mode_msg.data = 'global'
	while not rospy.is_shutdown():

		######## mode select based waypoint #######
		if (not use_map.delivery_map_num==0) and (global_wp <= use_map.glo_to_del_finish[0] and global_wp >= use_map.glo_to_del_start[0]):  # delivery mode A
			mode = 'delivery_A'
		elif (not use_map.delivery_map_num==0) and (global_wp <= use_map.glo_to_del_finish[1] and global_wp >= use_map.glo_to_del_start[1]):  # delivery mode B
			mode = 'delivery_B'
		elif (not use_map.horizontal_parking_map_num==0) and (global_wp <= use_map.glo_to_horizontal_park_finish) and (global_wp >= use_map.glo_to_horizontal_park_start):  # horizontal mode
			mode = 'horizontal_parking'
		
		if mode == 'delivery_A' and (global_wp >= use_map.del_to_glo_start[0]):
			mode = 'global'
		elif mode == 'delivery_B' and (global_wp >= use_map.del_to_glo_start[1]):
			mode = 'global'
		else:
			pass
		######### traffic light mode ################
		super_break = False

		#### traffic 인식 간격 속도별 조정 ####
		if (mode == 'delivery_A' or mode == 'delivery_B'):
			target_speed = use_map.target_speed['delivery'][current_dir]
		else:
			target_speed = use_map.target_speed[mode][current_dir]
		if target_speed >= 15:
			traffic_interval = 7
		elif target_speed >= 10:
			traffic_interval = 5
		elif target_speed > 5:
			traffic_interval = 4
		else:
			traffic_interval = 3
		#####################

		for number1 in range(len(use_map.trafficlight_list)):
			if (global_wp <= use_map.trafficlight_list[number1]) and (global_wp >= use_map.trafficlight_list[number1]-traffic_interval):
				traffic_mode = 'traffic'
				break
			else:
				for number2 in range(len(use_map.notrafficlight_list)):
					if (global_wp <= use_map.notrafficlight_list[number2]) and (global_wp >= use_map.notrafficlight_list[number2]-3):
						traffic_mode = 'notraffic'
						super_break = True
						break
					else:
						traffic_mode = 'no'
				if super_break == True:
					break
				traffic_mode = 'no'
		traffic_msg.data = traffic_mode
		##############################################

        ### 미션이 끝나면 end flag를 받아 global path 로 복귀 ##
		mode_status = rospy.get_param('mission_status')
		if mode_status == 'end':
			print('global start')
			mode = 'global'
			mode_status = 'going'
			rospy.set_param('mission_status', mode_status)
		else:
			pass

		mode_msg.data = mode
		mode_pub.publish(mode_msg)
		

		if mode == 'delivery_A':
			path_msg.x.data = use_map.delivery_path[0][0]  # A path
			path_msg.y.data = use_map.delivery_path[0][1]
			path_msg.yaw.data = use_map.delivery_path[0][2]
		elif mode == 'delivery_B':
			path_msg.x.data = use_map.delivery_path[1][0]  # B path
			path_msg.y.data = use_map.delivery_path[1][1]
			path_msg.yaw.data = use_map.delivery_path[1][2]
   
		else: # mode = 'global' or 'dynamic_object'
			path_msg.x.data = global_path_x
			path_msg.y.data = global_path_y
			path_msg.yaw.data = global_path_yaw

		path_pub.publish(path_msg)
		traffic_pub.publish(traffic_msg)		
		rospy.sleep(0.1)