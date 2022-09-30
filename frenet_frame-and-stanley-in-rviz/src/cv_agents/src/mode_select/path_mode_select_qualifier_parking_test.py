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

# diagonal parking mission
# dynamic object mission

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
	park_slow_pub = rospy.Publisher("/park_slow", String, queue_size=1)
	park_pub = rospy.Publisher("/park_ind_wp", Int32MultiArray, queue_size=1)

	parking_ind = 0
	parking = False
	park_wp = 0
	mode='global'
	dist = 0
	flag=0

	r = rospy.Rate(20)
	while not rospy.is_shutdown():

		path_msg = PathArray()
		mode_msg = String()
		park_msg = Int32MultiArray()
		park_slow_msg = String()

        ### 미션이 끝나면 end flag를 받아 global path 로 복귀 ##
		if mode_status == 'end':
			print('global start')
			mode = 'global'
			mode_status = 'going'
		else:
			pass

		if (not use_map.diagonal_parking_map_num==0) and (global_wp <= use_map.diagonal_park_object_finish and global_wp >= use_map.diagonal_park_object_start): # 주차 칸 인식을 위한 flag
			if (flag == 0):
				for park_i in range(use_map.diagonal_parking_map_num):
					print(str(park_i)+"번 주차 공간 인식 중")
					if (collision_check_for_parking(use_map.diagonal_parking_object[park_i],obs_info)==False):
						parking_ind=park_i
						parking = True
						print("parking_choose: "+str(park_i))
						flag=1
						break
			else:
				''' use_map.glo_to_diagonal_park_finish[parking_ind]는 
				주차 가능으로 인식한 구역에 장애물이 없는지 
				마지막까지 확인하기 위한 waypoint로
				테스트 하며 조절해야할 것 같습니다 '''
				if (global_wp <= use_map.glo_to_diagonal_park_finish[parking_ind]) and (collision_check_for_parking(use_map.diagonal_parking_object[park_i],obs_info)==True):
					parking = False
					for park_i in range(parking_ind, use_map.diagonal_parking_map_num, 1):
						print(str(park_i)+"번 주차 공간 인식 중")
						if collision_check_for_parking(use_map.diagonal_parking_object[park_i],obs_info)==False:
							parking_ind=park_i
							parking = True
							print("parking_choose: "+str(park_i))
							break
		
		if (global_wp <= use_map.diagonal_park_object_finish) and (global_wp >= use_map.diagonal_park_object_start):
			park_slow_msg.data = 'slow'
		else:
			park_slow_msg.data = 'no'

		######## mode select based waypoint #######
		if (not use_map.diagonal_parking_map_num==0) and (global_wp <= use_map.glo_to_diagonal_park_finish[parking_ind] and global_wp >=use_map.glo_to_diagonal_park_start[parking_ind])and (parking==True):#and (parking == False):
			mode = 'diagonal_parking'
		elif (global_wp <= use_map.glo_to_dynamic_finish and global_wp >= use_map.glo_to_dynamic_start):  # dynamic_object
			mode = 'dynamic_object'
			if global_wp >= (use_map.glo_to_dynamic_finish-5):
				mode = 'global'
		elif (global_wp >= use_map.glo_to_static_finish and global_wp >= use_map.glo_to_static_start):
			mode = 'static_object'
			if (global_wp > use_map.glo_to_static_finish):
				mode = 'global'
		else:
			pass

		mode_msg.data = mode
		mode_pub.publish(mode_msg)
		
		if mode == 'diagonal_parking':

			# parking_ind=2

			fp=MakingPath()
			fp.x=use_map.diagonal_parking_path[parking_ind][0]
			fp.y=use_map.diagonal_parking_path[parking_ind][1]
			fp.yaw=use_map.diagonal_parking_path[parking_ind][2]

			park_wp = get_closest_waypoints(state_x, state_y, use_map.waypoints['diagonal_parking'][parking_ind*2]['x'][:use_map.link_len['diagonal_parking'][parking_ind*2]], use_map.waypoints['diagonal_parking'][parking_ind*2]['y'][:use_map.link_len['diagonal_parking'][parking_ind*2]],park_wp)
			print("현재 주차할 위치 : " + str(parking_ind) + "차량 위치 :" + str(park_wp))
			park_msg.data = [parking_ind, park_wp] #현재 이동하는 parking index, wp보내줌
			path_msg.x.data = fp.x  # parking final path
			path_msg.y.data = fp.y
			path_msg.yaw.data = fp.yaw
			
		else: # mode = 'global' or 'dynamic_object'
			path_msg.x.data = global_path_x
			path_msg.y.data = global_path_y
			path_msg.yaw.data = global_path_yaw

			park_msg.data=[0,0]

		park_pub.publish(park_msg)
		path_pub.publish(path_msg)
		park_slow_pub.publish(park_slow_msg)
		r.sleep()