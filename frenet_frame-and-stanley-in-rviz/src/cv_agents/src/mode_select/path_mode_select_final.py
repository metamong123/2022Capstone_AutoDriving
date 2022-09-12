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
	rospy.Subscriber("/objects/car_1/gps", Object, state_callback, queue_size=1)
	mode_pub = rospy.Publisher("/mode_selector", String, queue_size=10)
	path_pub = rospy.Publisher("/final_path", PathArray, queue_size=1)
	park_pub = rospy.Publisher("/park_ind_wp", Int32MultiArray, queue_size=1)
	dc_pub = rospy.Publisher("/dc", String, queue_size=1)
	path_msg = PathArray()
	mode_msg = String()
	park_msg = Int32MultiArray()
	dc_msg = String()
	parking_ind = 0
	parking = False
	park_wp = 0
	r = rospy.Rate(10)
	mode='global'
	dist = 0
	dc = 'no'

	#mode_msg.data = 'global'
	while not rospy.is_shutdown():

		### 미션이 끝나면 end flag를 받아 global path 로 복귀 ##
		mode_status = rospy.get_param('mission_status')
		if mode_status == 'end':
			print('global start')
			mode = 'global'
			mode_status = 'going'
			rospy.set_param('mission_status', mode_status)
		else:
			pass

		parking_ind = 0 # 수평주차할 위치
		######## mode select based waypoint #######
		if (not use_map.delivery_map_num==0) and (global_wp <= use_map.glo_to_del_finish[0] and global_wp >= use_map.glo_to_del_start[0]):  # delivery mode A
			mode = 'delivery_A'
			dc = 'slow'
		elif (not use_map.delivery_map_num==0) and (global_wp <= use_map.glo_to_del_finish[1] and global_wp >= use_map.glo_to_del_start[1]):  # delivery mode B
			mode = 'delivery_B'
			dc = 'slow'
		elif (not use_map.horizontal_parking_map_num==0) and (global_wp <= use_map.glo_to_horizontal_park_start[parking_ind] and parking = False): # horizontal parking mode
			mode = 'horizontal_parking'
		else:
			dc = 'no'
		if mode == 'delivery_A' and (global_wp >= use_map.del_to_glo_start[0]):
			mode = 'global'
		elif mode == 'delivery_B' and (global_wp >= use_map.del_to_glo_start[1]):
			mode = 'global'
		else:
			pass


		mode_msg.data = mode
		dc_msg.data = dc
		mode_pub.publish(mode_msg)
		dc_pub.publish(dc_msg)
		

		if mode == 'delivery_A':
			path_msg.x.data = use_map.delivery_path[0][0]  # A path
			path_msg.y.data = use_map.delivery_path[0][1]
			path_msg.yaw.data = use_map.delivery_path[0][2]
		elif mode == 'delivery_B':
			path_msg.x.data = use_map.delivery_path[1][0]  # B path
			path_msg.y.data = use_map.delivery_path[1][1]
			path_msg.yaw.data = use_map.delivery_path[1][2]
		elif mode == 'parking'
   
		else: # mode = 'global' or 'dynamic_object'
			path_msg.x.data = global_path_x
			path_msg.y.data = global_path_y
			path_msg.yaw.data = global_path_yaw

		path_pub.publish(path_msg)
		
		r.sleep()