#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray
import rospkg
import sys
rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *

global_wp = 0
def waypoint_callback(msg):
	global  global_wp
	global_wp = msg.data[1]  #global waypoint

if __name__ == "__main__":
	rospy.init_node("print_traffic_mode")
	rospy.Subscriber("/waypoint_1", Int32MultiArray, waypoint_callback)
	traffic_mode = 'no'
	r=rospy.Rate(10)
	mode='global'
	while not rospy.is_shutdown():
		
		# for number in range(len(use_map.trafficlight_list)):
		# 	if (global_wp <= use_map.trafficlight_list[number]) and (global_wp >= use_map.trafficlight_list[number]-3):
		# 		traffic_mode = 'traffic'
		# 		break
		# 	else:
		# 		traffic_mode = 'no'

		# for number in range(len(use_map.notrafficlight_list)):
		# 	if (global_wp <= use_map.notrafficlight_list[number]) and (global_wp >= use_map.notrafficlight_list[number]-3):
		# 		traffic_mode = 'notraffic'
		# 		break
		# 	else:
		# 		traffic_mode = 'no'
		for number in range(len(use_map.trafficlight_list)):
			if (global_wp <= use_map.trafficlight_list[number]) and (global_wp >= use_map.trafficlight_list[number]-10):
				traffic_mode = 'traffic'
				break
			else:
				for number1 in range(len(use_map.notrafficlight_list)):
					if (global_wp <= use_map.notrafficlight_list[number1]) and (global_wp >= use_map.notrafficlight_list[number1]-10):
						traffic_mode = 'notraffic'
						# print("IN")
						# print(traffic_mode)
						break
					else:
						traffic_mode = 'no'
				# print(traffic_mode)
				# traffic_mode = 'no'
				# print(traffic_mode)

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

		print(traffic_mode, mode)