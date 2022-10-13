#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import rospkg
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float64, String, Int32, Float32
from rocon_std_msgs.msg import StringArray 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from object_msgs.msg import Object
from darknet_ros_msgs.msg import BoundingBoxes

import numpy as np

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *

global parking_yaw

parking_flag = 'forward'
notraffic_status =  False
waypoint = 0
w = 0
z = 0
yaw = 0

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z

# parking 시작하기전에 수정해야할 파라미터 값들 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
parking_finish_wp=[51,51,51] # 각 parking index마다의 finish waypoint임
parking_cmd_wp = [11,27,32,46]#48]
###########################################################################

car_mode = 'global'
def mode_callback(msg):
	global car_mode
	car_mode = msg.data

current_dir = 'straight'
next_dir = 'straight'
def link_callback(msg):
	global current_dir, next_dir
	current_dir = msg.strings[0]
	next_dir = msg.strings[1]

traffic_mode = 'no'
def traffic_callback(msg):
	global traffic_mode
	traffic_mode = msg.data

frenet_speed = 0
frenet_angle = 0 
frenet_gear = 0
def frenet_callback(msg):
    global frenet_speed, frenet_angle, frenet_gear
    frenet_speed = msg.drive.speed
    frenet_angle = msg.drive.steering_angle  
    frenet_gear = 0

park_ind_wp = [0,0]
def parking_callback(msg):
	global park_ind_wp
	park_ind_wp = msg.data

velocity=0
def callback2(msg):
	global velocity
	obj_msg=msg
	velocity=obj_msg.v * 3.6

link_ind = 0
global_wp = 0
def waypoint_callback(msg):
	global  link_ind, global_wp
	link_ind = msg.data[0]   #link index
	global_wp = msg.data[1]  #global waypoint

traffic_light = -1
def forward_callback(msg):
	global  traffic_light
	traffic_light = msg.data[0]


def odometry_callback(msg):
	global yaw
	x = msg.pose.pose.orientation.x
	y = msg.pose.pose.orientation.y
	z = msg.pose.pose.orientation.z
	w = msg.pose.pose.orientation.w
	yaw = euler_from_quaternion(x, y, z, w)

A_number = 0
A_x = [0,0,0]
B_x = [0,0,0]
def delivery_sign_callback(msg):
	global A_number, A_x, B_x
	A_number = msg.data[0]
	A_x = [msg.data[1], msg.data[2], msg.data[3]]
	B_x = [msg.data[4], msg.data[5], msg.data[6]]

traffic_slow = 'no'
def slow_callback(msg):
    global traffic_slow
    traffic_slow = msg.data

park_slow = 'no'
def park_slow_callback(msg):
	global park_slow
	park_slow = msg.data

uturn = 'no'
def uturn_callback(msg):
	global uturn
	uturn = msg.data

def traffic_decision():
	global next_dir

	if next_dir == 'left':
		if traffic_light == 0 or traffic_light == 2 or traffic_light == 4:  # -1 : none   0 : green   1 : left   2 : red   3 : straightleft   4 : yellow
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 100
			print("traffic mode : stop")
		elif traffic_light == -1:
			traffic_speed = frenet_speed/2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 40
			print("traffic mode : none")
		else :
			traffic_speed = frenet_speed
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 20
			print("traffic mode : go")
    
	elif next_dir == 'straight':
		if traffic_light == 1 or traffic_light == 2 or traffic_light == 4:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 100
			print("traffic mode : stop")
		elif traffic_light == -1:
			traffic_speed = frenet_speed/2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 40
			print("traffic mode : none")
		else :
			traffic_speed = frenet_speed
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 20
			print("traffic mode : go")
	elif next_dir == 'right':
		if traffic_light == 1 or traffic_light == 2 or traffic_light == 4:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 100
			print("traffic mode : stop")
		elif traffic_light == -1:
			traffic_speed = frenet_speed/2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 40
			print("traffic mode : none")
		else :
			traffic_speed = frenet_speed
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 20
			print("traffic mode : go")
	return traffic_speed, traffic_angle, traffic_gear, traffic_brake

delivery_ind =0 
delivery_flag = 'going'
def delivery_decision():
	global delivery_ind, delivery_flag
	
	#print(A_number)
	if A_number == 0:  # A1
		delivery_ind = 0
	elif A_number == 1: # A2
		delivery_ind = 1
	elif A_number == 2: # A3
		delivery_ind = 2
	else:
		pass

	if car_mode == 'delivery_A':
		if A_x[delivery_ind] > 315:   #parameter
			delivery_flag = 'end'
		else:
			delivery_flag = 'going'
	elif car_mode == 'delivery_B':
		if B_x[delivery_ind] > 315:   #parameter
			delivery_flag = 'end'
		else:
			delivery_flag = 'going'
	return delivery_flag

if __name__=='__main__':

	rospy.init_node('core_control')

	rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
	rospy.Subscriber("/forward_sign", Int32MultiArray, forward_callback)
	rospy.Subscriber('/side_sign',Int32MultiArray, delivery_sign_callback)
	rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
	rospy.Subscriber("/odom_imu", Odometry, odometry_callback)
	rospy.Subscriber("/mode_selector",String,mode_callback,queue_size=10)
	rospy.Subscriber("/link_direction", StringArray, link_callback)
	rospy.Subscriber("/park_ind_wp", Int32MultiArray, parking_callback)
	rospy.Subscriber("/traffic_mode", String, traffic_callback)
	rospy.Subscriber("/traffic_slow", String, slow_callback)
	rospy.Subscriber("/objects/car_1", Object, callback2)
	rospy.Subscriber("/park_slow", String, park_slow_callback)
	rospy.Subscriber("/uturn", String, uturn_callback)
	final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)

	mode_status = 'going'
	parking_start = False
	parking_flag = False
	notraffic_status = False
	park_yaw= -2.05735833
	j = 0
	i = 0
	r=rospy.Rate(20)
	while not rospy.is_shutdown():
		status_Pub = rospy.Publisher('/mission_status', String, queue_size=10)
		status_msg = String()
		cmd=AckermannDriveStamped()
		cmd.header.stamp=rospy.Time.now()
		if car_mode == 'global':
			if traffic_mode == 'traffic':
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
				print('traffic mode')
			elif traffic_mode == 'notraffic':
				if notraffic_status == False:
					cmd.drive.speed = 0
					cmd.drive.steering_angle = 0
					cmd.drive.acceleration = 0
					cmd.drive.jerk = 100
					notraffic_status = True
					final_cmd_Pub.publish(cmd)
					print('no traffic mode')
					rospy.sleep(3) # 4sec
				elif notraffic_status == True:
					if abs(frenet_angle) > 0.05: #각도 파라미터
						if j<15:  #감속
							cmd.drive.speed = frenet_speed/2#7/3.6
							cmd.drive.steering_angle = frenet_angle#steer
							cmd.drive.acceleration = frenet_gear
							cmd.drive.jerk = int(5 * velocity) if velocity >= 5 else 0
							j=j+1
						else:
							cmd.drive.speed = frenet_speed#15/3.6
							cmd.drive.steering_angle = frenet_angle#steer
							cmd.drive.acceleration = frenet_gear
							cmd.drive.jerk = 0	
					else:	
						cmd.drive.speed = frenet_speed#15/3.6
						cmd.drive.steering_angle = frenet_angle#steer
						cmd.drive.acceleration = frenet_gear
						cmd.drive.jerk = 0
						j = 0
			else:
				if traffic_slow == 'slow':
					cmd.drive.speed = frenet_speed#15/3.6
					cmd.drive.steering_angle = frenet_angle#steer
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 60
				else:
					if uturn == 'slow':
						if traffic_light == 1:
							if abs(frenet_angle) > 0.05:
								if j<15:  #감속
									cmd.drive.speed = frenet_speed/2
									cmd.drive.steering_angle = frenet_angle
									cmd.drive.acceleration = frenet_gear
									cmd.drive.jerk = int(5.5 * velocity) if velocity >= 5 else 0
									j=j+1
								else:
									cmd.drive.speed = frenet_speed
									cmd.drive.steering_angle = frenet_angle
									cmd.drive.acceleration = frenet_gear
									cmd.drive.jerk = 0
							else:
								cmd.drive.speed = frenet_speed
								cmd.drive.steering_angle = frenet_angle
								cmd.drive.acceleration = frenet_gear
								cmd.drive.jerk = 0
								j = 0
						else:
							cmd.drive.speed = 0
							cmd.drive.steering_angle = 0
							cmd.drive.acceleration = 0
							cmd.drive.jerk = 100
					else:
						if park_slow == 'no':
							if abs(frenet_angle) > 0.05: #각도 파라미터
								if j<15:  #감속
									cmd.drive.speed = frenet_speed/2
									cmd.drive.steering_angle = frenet_angle
									cmd.drive.acceleration = frenet_gear
									cmd.drive.jerk = int(5 * velocity) if velocity >= 5 else 0
									j=j+1
								else:
									cmd.drive.speed = frenet_speed
									cmd.drive.steering_angle = frenet_angle
									cmd.drive.acceleration = frenet_gear
									cmd.drive.jerk = 0	
							else:	
								cmd.drive.speed = frenet_speed
								cmd.drive.steering_angle = frenet_angle
								cmd.drive.acceleration = frenet_gear
								cmd.drive.jerk = 0
								j = 0
						else:
							if i < 30:
								cmd.drive.speed = 7/3.6
								cmd.drive.steering_angle = frenet_angle
								cmd.drive.acceleration = frenet_gear
								cmd.drive.jerk = int(6.5 * velocity) if velocity >= 5 else 0
								i = i+1
							else:
								if abs(frenet_angle) > 0.05:
									if j<15:  #감속
										cmd.drive.speed = 7/3.6
										cmd.drive.steering_angle = frenet_angle
										cmd.drive.acceleration = frenet_gear
										cmd.drive.jerk = int(5.5 * velocity) if velocity >= 5 else 0
										j=j+1
									else:
										cmd.drive.speed = 7/3.6
										cmd.drive.steering_angle = frenet_angle
										cmd.drive.acceleration = frenet_gear
										cmd.drive.jerk = 0
								else:
									cmd.drive.speed = 7/3.6
									cmd.drive.steering_angle = frenet_angle
									cmd.drive.acceleration = frenet_gear
									cmd.drive.jerk = 0
									j = 0
				notraffic_status = False # notraffic 구간이 여러번 있으니 바꿔줘야함
				print('global mode!!!')
			mode_status = 'going'
			
		elif car_mode == 'horizontal_parking':
			if parking_start == False:
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				print('parking start')
				rospy.sleep(2)
				parking_start = True
			else:
				if parking_flag == False:
					if park_ind_wp[1] <= parking_cmd_wp[0]:
						cmd.drive.speed = 5/3.6
						cmd.drive.steering_angle = 0
						cmd.drive.acceleration = 2
						cmd.drive.jerk = 0
					elif park_ind_wp[1] <= parking_cmd_wp[1]:
						cmd.drive.speed = 5/3.6
						cmd.drive.steering_angle = -28*np.pi/180
						cmd.drive.acceleration = 2
						cmd.drive.jerk = 0
					elif park_ind_wp[1] <= parking_cmd_wp[2]:
						cmd.drive.speed = 5/3.6
						cmd.drive.steering_angle = 0
						cmd.drive.acceleration = 2
						cmd.drive.jerk = 0	
					else:
						if (yaw >= park_yaw-2*np.pi/180) and (yaw <= park_yaw+2*np.pi/180):
							cmd.drive.speed = 5/3.6
							cmd.drive.steering_angle = 0
							cmd.drive.acceleration = 2
							cmd.drive.jerk = 0						
						else:
							cmd.drive.speed = 5/3.6
							cmd.drive.steering_angle = 28*np.pi/180
							cmd.drive.acceleration = 2
							cmd.drive.jerk = 0

					if park_ind_wp[1] >= parking_finish_wp[park_ind_wp[0]]:
						cmd.drive.speed = 0
						cmd.drive.steering_angle = 0
						cmd.drive.acceleration = 0
						cmd.drive.jerk = 200  #full brake
						parking_flag = True
						final_cmd_Pub.publish(cmd)
						print('parking finish!!! stop!!')
						rospy.sleep(10) # 4sec
				else:
					if park_ind_wp[1] <= parking_cmd_wp[3]:
						mode_status = 'end'
						status_msg.data = mode_status
						status_Pub.publish(status_msg)
						rospy.sleep(0.5)
					else:
						cmd.drive.speed = 5/3.6
						cmd.drive.steering_angle = 28*np.pi/180
						cmd.drive.acceleration = 0
						cmd.drive.jerk = 0

		elif car_mode == 'delivery_A' or car_mode == 'delivery_B':
			delivery_flag = delivery_decision()
			if delivery_flag == 'going':
				if abs(frenet_angle) > 0.1: #각도 파라미터
					if j<100:  #감속
						cmd.drive.speed = frenet_speed
						cmd.drive.steering_angle = frenet_angle
						cmd.drive.acceleration = frenet_gear
						cmd.drive.jerk = int(5.5 * velocity) if velocity >= 2 else 0
						j=j+1
					else:
						cmd.drive.speed = frenet_speed
						cmd.drive.steering_angle = frenet_angle
						cmd.drive.acceleration = frenet_gear
						cmd.drive.jerk = 0	
				else:	
					cmd.drive.speed = frenet_speed
					cmd.drive.steering_angle = frenet_angle
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 0
					j = 0
			elif delivery_flag == 'end':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				mode_status = 'end'   # global mode로 바꾸기위한 flag를 파라미터 서버로 전달
				delivery_flag = 'going'
				status_msg.data = mode_status
				status_Pub.publish(status_msg)
				print('delivery finish!!! stop!!')
				rospy.sleep(5) # 5sec
			print('delivery mode')
		elif car_mode == 'static_object':
			cmd.drive.speed = frenet_speed
			cmd.drive.steering_angle = frenet_angle
			cmd.drive.acceleration = frenet_gear
			cmd.drive.jerk = 0

		status_msg.data = mode_status
		status_Pub.publish(status_msg)
		final_cmd_Pub.publish(cmd)

		r.sleep()



