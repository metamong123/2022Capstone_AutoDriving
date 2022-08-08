#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospkg
import sys
import rospy
import numpy as np
import math, time

from object_msgs.msg import PathArray, ObjectArray
from std_msgs.msg import Float64, Int32MultiArray, String, Int32
from rocon_std_msgs.msg import StringArray

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *

##################### load path ###############################
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
 
current_dir = 'straight'
next_dir = 'straight'
def link_callback(msg):
	global current_dir, next_dir
	current_dir = msg.strings[0]
	next_dir = msg.strings[1]
    
mode_status = 'going'
def end_callback(msg):
    global mode_status
    mode_status = msg.data

######## 주차칸에 차량의 위치 파악 ############
obs_info=[]
def obstacle_callback(msg):
	global obs_info
	obs_info = []
	for o in msg.object_list:
		obj = [o.x, o.y, o.yaw, o.L, o.W]
		obs_info.append(obj)
##########################################

if __name__ == "__main__":

	rospy.init_node("path_select")

	rospy.Subscriber("/optimal_frenet_path_global", PathArray, global_path_callback)
	rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
	rospy.Subscriber("/link_direction", StringArray, link_callback)
	rospy.Subscriber("/obstacles", ObjectArray, obstacle_callback)
	rospy.Subscriber("/mission_status", String, end_callback)

	mode_pub = rospy.Publisher("/mode_selector", String, queue_size=1)
	path_pub = rospy.Publisher("/final_path", PathArray, queue_size=1)
	park_pub = rospy.Publisher("/park_ind", Float64, queue_size=1)

	path_msg = PathArray()
	mode_msg = String()
	park_msg = Float64()

	parking_ind = 0

	mode='global'
	
	while not rospy.is_shutdown():

		######## mode select based waypoint #######
		if (global_wp <= use_map.glo_to_park_finish and global_wp >=use_map.glo_to_park_start):  # parking mode
			mode_msg.data = 'parking'
			mode = 'parking'
		elif (global_wp <= use_map.glo_to_del_finish[0] and global_wp >= use_map.glo_to_del_start[0]):  # delivery mode A
			mode_msg.data = 'delivery'
			mode = 'delivery_A'
		elif (global_wp <= use_map.glo_to_del_finish[1] and global_wp >= use_map.glo_to_del_start[1]):  # delivery mode B
			mode_msg.data = 'delivery'
			mode = 'delivery_B'
   
        ### 미션이 끝나면 end flag를 받아 global path 로 복귀 ##
		if mode_status == 'end':
			mode_msg.data = 'global'
			mode = 'global'

		mode_msg.data=mode

		if mode == 'global':
			path_msg.x.data = global_path_x
			path_msg.y.data = global_path_y
			path_msg.yaw.data = global_path_yaw

		elif mode == 'parking':

			for park_i in range(use_map.parking_map_num):
				
				fp = MakingPath()
				fp.x=use_map.parking_path[park_i][0]
				fp.y=use_map.parking_path[park_i][1]
				fp.yaw=use_map.parking_path[park_i][2]
				print("위치" + str(park_i) + "번 차량존재유무 : " + str(collision_check(fp,obs_info,0,0,0)))

				if collision_check(fp,obs_info,0,0,0)==False:
					parking_ind=park_i
					print("parking_choose: "+str(park_i))
					break

			if collision_check(fp,obs_info,0,0,0)==True:
				for park_i in range(parking_ind,use_map.parking_map_num,1):
					fp_1=MakingPath()
					fp_1.x=use_map.parking_path[park_i][0]
					fp_1.y=use_map.parking_path[park_i][1]
					fp_1.yaw=use_map.parking_path[park_i][2]
					print("위치" + str(park_i) + "번 차량존재유무 : " + str(collision_check(fp_1,obs_info,0,0,0)))

					if collision_check(fp_1,obs_info,0,0,0)==False:
						parking_ind=park_i
						print("parking_choose: "+str(park_i))
						fp=fp_1
						break
			park_msg.data = parking_ind #현재 이동하는 parking index보내줌
			path_msg.x.data = fp.x  # parking final path
			path_msg.y.data = fp.y
			path_msg.yaw.data = fp.yaw
  
		elif mode == 'delivery_A':
			path_msg.x.data = use_map.delivery_path[0][0]  # A path
			path_msg.y.data = use_map.delivery_path[0][1]
			path_msg.yaw.data = use_map.delivery_path[0][2]
		elif mode == 'delivery_B':
			path_msg.x.data = use_map.delivery_path[1][0]  # B path
			path_msg.y.data = use_map.delivery_path[1][1]
			path_msg.yaw.data = use_map.delivery_path[1][2]
   
		mode_pub.publish(mode_msg)
		path_pub.publish(path_msg)
		park_pub.publish(park_msg)
		#r.sleep()
	
