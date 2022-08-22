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

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *

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

global_wp = 0
# parking_wp1 = 0
# parking_wp2 = 0
# parking_wp3 = 0
# parking_wp4 = 0
# parking_wp5 = 0
# parking_wp6 = 0
# global global_wp, parking_wp1, parking_wp2, parking_wp3, parking_wp4, parking_wp5, parking_wp6
def waypoint_callback(msg):
	global global_wp
	global_wp = msg.data[1]  #global waypoint
# 	parking_wp1 = msg.data[2] #parking waypoint
# 	parking_wp2 = msg.data[3]
# 	parking_wp3 = msg.data[4]
# 	parking_wp4 = msg.data[5]
# 	parking_wp5 = msg.data[6]
# 	parking_wp6 = msg.data[7]
 
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

	parking_ind = 0
	park_wp = 0
	
	mode='global'
	
	#mode_msg.data = 'global'
	while not rospy.is_shutdown():

		rospy.Subscriber("/optimal_frenet_path_global", PathArray, global_path_callback)
		rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
		rospy.Subscriber("/link_direction", StringArray, link_callback)
		rospy.Subscriber("/obstacles", ObjectArray, obstacle_callback)
		rospy.Subscriber("/mission_status", String, end_callback)
		rospy.Subscriber("/objects/car_1/gps", Object, state_callback, queue_size=1)
		mode_pub = rospy.Publisher("/mode_selector", String, queue_size=10)
		path_pub = rospy.Publisher("/final_path", PathArray, queue_size=1)
		park_pub = rospy.Publisher("/park_ind_wp", Int32MultiArray, queue_size=1)

		path_msg = PathArray()
		mode_msg = String()
		park_msg = Int32MultiArray()
		#print(use_map.glo_to_park_start)
		######## mode select based waypoint #######
		if (not use_map.diagonal_parking_map_num==0) and (global_wp <= use_map.glo_to_diagonal_park_finish and global_wp >=use_map.glo_to_diagonal_park_start):  # parking mode
			mode = 'parking'
		elif (not use_map.delivery_map_num==0) and (global_wp <= use_map.glo_to_del_finish[0] and global_wp >= use_map.glo_to_del_start[0]):  # delivery mode A
			mode = 'delivery_A'
		elif (not use_map.delivery_map_num==0) and (global_wp <= use_map.glo_to_del_finish[1] and global_wp >= use_map.glo_to_del_start[1]):  # delivery mode B
			mode = 'delivery_B'
		elif (global_wp <= use_map.glo_to_dynamic_finish and global_wp >= use_map.glo_to_dynamic_start):  # delivery mode B
			mode = 'dynamic_object'
		

        ### 미션이 끝나면 end flag를 받아 global path 로 복귀 ##
		mode_status = rospy.get_param('mission_status')
		if mode_status == 'end':
			print('global start')
			mode = 'global'
			mode_status = 'going'
			rospy.set_param('mission_status', mode_status)
		else:
			pass
		if (mode == 'delivery_A' and mode == 'delivery_B'):
			mode_msg.data = 'delivery'
		else:
			mode_msg.data = mode
			mode_pub.publish(mode_msg)
		

		if mode == 'parking':

			for park_i in range(use_map.diagonal_parking_map_num):
				
				fp = MakingPath()
				fp.x=use_map.diagonal_parking_path[park_i][0]
				fp.y=use_map.diagonal_parking_path[park_i][1]
				fp.yaw=use_map.diagonal_parking_path[park_i][2]
				print("위치" + str(park_i) + "번 차량존재유무 : " + str(collision_check(fp,obs_info,0,0,0)))

				if collision_check(fp,obs_info,0,0,0)==False:
					parking_ind=park_i
					print("parking_choose: "+str(park_i))
					break

			if collision_check(fp,obs_info,0,0,0)==True:
				for park_i in range(parking_ind,use_map.diagonal_parking_map_num,1):
					fp_1=MakingPath()
					fp_1.x=use_map.diagonal_parking_path[park_i][0]
					fp_1.y=use_map.diagonal_parking_path[park_i][1]
					fp_1.yaw=use_map.diagonal_parking_path[park_i][2]
					print("위치" + str(park_i) + "번 차량존재유무 : " + str(collision_check(fp_1,obs_info,0,0,0)))

					if collision_check(fp_1,obs_info,0,0,0)==False:
						parking_ind=park_i
						print("parking_choose: "+str(park_i))
						fp=fp_1
						break
			parking_ind=2
			state_x=962802.5118152874
			state_y=1959347.0844059486
			park_wp = get_closest_waypoints(state_x, state_y, use_map.waypoints['diagonal_parking'][parking_ind*2]['x'][:use_map.link_len['diagonal_parking'][parking_ind*2]], use_map.waypoints['diagonal_parking'][parking_ind*2]['y'][:use_map.link_len['diagonal_parking'][parking_ind*2]],park_wp)
			print(park_wp)
			park_msg.data = [parking_ind, park_wp] #현재 이동하는 parking index, wp보내줌
			path_msg.x.data = fp.x  # parking final path
			path_msg.y.data = fp.y
			path_msg.yaw.data = fp.yaw
			park_pub.publish(park_msg)
  
		elif mode == 'delivery_A':
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
		
		rospy.sleep(0.1)