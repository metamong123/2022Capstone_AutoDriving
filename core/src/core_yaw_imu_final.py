#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Int32MultiArray, Float64, String
from rocon_std_msgs.msg import StringArray 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes

import numpy as np


global parking_yaw

parking_flag = 'forward'

waypoint = 0
w = 0
z = 0
yaw = 0
parking_yaw = 0

##########################################################################

# parking 시작하기전에 수정해야할 파라미터 값들 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
parking_finish_wp=[16,16,16,16,16,16] # 각 parking index마다의 finish waypoint임
parking_straight_back_wp=[17,16,16,16,16,16]

###########################################################################

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

car_mode = 'global'
def mode_callback(msg):
	global car_mode
	print(1)
	car_mode = msg.data

current_dir = 'straight'
next_dir = 'straight'
def link_callback(msg):
	global current_dir, next_dir
	current_dir = msg.strings[0]
	next_dir = msg.strings[1]

assist_steer=0
def lanenet_callback(msg):
    global assist_steer
    assist_steer = msg.data

frenet_speed = 0
frenet_angle = 0 
frenet_gear = 0
def frenet_callback(msg):
    global frenet_speed, frenet_angle, frenet_gear
    frenet_speed = msg.drive.speed
    frenet_angle = msg.drive.steering_angle  
    frenet_gear = 0

global_wp = 0
parking_wp = []
def waypoint_callback(msg):
	global global_wp, parking_wp1, parking_wp2, parking_wp3, parking_wp4, parking_wp5, parking_wp6
	global_wp = msg.data[0]  #global waypoint
	parking_wp[0] = msg.data[1] #parking waypoint
	parking_wp[1] = msg.data[2]
	parking_wp[2] = msg.data[3]
	parking_wp[3] = msg.data[4]
	parking_wp[4] = msg.data[5]
	parking_wp[5] = msg.data[6]

traffic_light = 0 
person = 0 
car = 0
def yolo_callback(msg):
	global  traffic_light, person, car
	traffic_light = msg.data[0]
	person = msg.data[1]
	car = msg.data[2]

def odometry_callback(msg):
	global yaw
	x = msg.pose.pose.orientation.x
	y = msg.pose.pose.orientation.y
	z = msg.pose.pose.orientation.z
	w = msg.pose.pose.orientation.w
	yaw = euler_from_quaternion(x, y, z, w)

park_ind_wp = [0,0]
def parking_callback(msg):
	global park_ind_wp
	park_ind_wp = msg.data

A_number = 0
A_x = [0,0,0]
B_x = [0,0,0]
def delivery_sign_callback(msg):
	global A_number, A_x, B_x
	A_number = msg.data[0]
	A_x = [msg.data[1], msg.data[2], msg.data[3]]
	B_x = [msg.data[4], msg.data[5], msg.data[6]]

def parking_decision():
	global parking_flag
	global back_speed, back_angle
	global save_speed,save_angle
	global move_mode, frenet_speed, frenet_angle, frenet_gear  
	global parking_angle, parking_brake, parking_speed, parking_gear, parking_yaw, yaw
 	
	#print(park_ind_wp[1])

	if parking_flag == 'backward':
		print(yaw)
		if abs(yaw - parking_yaw) < 3*np.pi/180: # hyperparameter(degree)
			parking_flag = 'end'
		else:
			if park_ind_wp[1] > parking_straight_back_wp[park_ind_wp[0]]:
				parking_speed = 8/3.6
				parking_angle = 0
				parking_gear = 2
				parking_brake = 0	
			else:
				parking_speed = 8/3.6
				parking_angle = -20*np.pi/180
				parking_gear = 2
				parking_brake = 0
	
	else:
		if park_ind_wp[1] >= parking_finish_wp[park_ind_wp[0]]:
			parking_flag = 'finish'
		else:
			parking_speed = frenet_speed
			parking_angle = frenet_angle
			parking_gear = frenet_gear
			parking_brake = 0

	return parking_speed, parking_angle, parking_gear, parking_brake

def traffic_decision():
	global next_dir

	if next_dir == 'left':
		if traffic_light == 6 or traffic_light == 8 or traffic_light == 10:  # -1 : none   6 : green   7 : left   8 : red   9 : straightleft   10 : yellow
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 50
			print("traffic mode : stop")
		elif traffic_light == -1:
			traffic_speed = frenet_speed/2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 0
			print("traffic mode : none")
		else :
			traffic_speed = frenet_speed
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 0
			print("traffic mode : go")
    
	elif next_dir == 'straight':
		if traffic_light == 7 or traffic_light == 8 or traffic_light == 10:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 50
			print("traffic mode : stop")
		elif traffic_light == -1:
			traffic_speed = frenet_speed/2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 0
			print("traffic mode : none")
		else :
			traffic_speed = frenet_speed
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 0
			print("traffic mode : go")
	elif next_dir == 'right':
		if traffic_light == 7 or traffic_light == 8 or traffic_light == 10:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 50
			print("traffic mode : stop")
		elif traffic_light == -1:
			traffic_speed = frenet_speed/2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 0
			print("traffic mode : none")
		else :
			traffic_speed = frenet_speed
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 0
			print("traffic mode : go")
	return traffic_speed, traffic_angle, traffic_gear, traffic_brake

def delivery_decision():
	if A_number == 0:  # A1
		delivery_ind = 0
	elif A_number == 1: # A2
		delivery_ind = 1
	elif A_number == 2: # A3
		delivery_ind = 2
	else:
		pass

	if car_mode == 'delivery_A':
		if A_x[delivery_ind] > 300:
			delivery_flag = 'end'
		else:
			delivery_flag = 'going'
	elif car_mode == 'delivery_B':
		if B_x[delivery_ind] > 300:
			delivery_flag = 'end'
		else:
			delivery_flag = 'going'
	return delivery_flag


if __name__=='__main__':

	rospy.init_node('core_control')

	mode_status = 'going'
	while not rospy.is_shutdown():
		rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
		rospy.Subscriber("/detect_ID", Int32MultiArray, yolo_callback)
		rospy.Subscriber("/assist_steer", Float64, lanenet_callback)
		rospy.Subscriber("/waypoint", Float64, waypoint_callback)
		rospy.Subscriber("/odom_imu", Odometry, odometry_callback)
		rospy.Subscriber("/park_ind_wp", Int32MultiArray, parking_callback)

		rospy.Subscriber("/mode_selector",String,mode_callback,queue_size=10)
		rospy.Subscriber("/link_direction", StringArray, link_callback)

		#mission_pub = rospy.Publisher('/mission_status', String, queue_size=10)
		final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)

		cmd=AckermannDriveStamped()
		#end_msg=String()

		if car_mode == 'global':
			#if move_mode == 'finish':
			#	cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
			#else:
			if parking_flag == 'backward':  # for parking
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()
			else:
				if current_dir == 'straight':
					if assist_steer == 0:
						cmd.drive.speed = frenet_speed
						cmd.drive.steering_angle = frenet_angle
						cmd.drive.acceleration = frenet_gear
						cmd.drive.jerk = 0
					else:
						cmd.drive.speed = frenet_speed
						cmd.drive.steering_angle = assist_steer
						cmd.drive.acceleration = frenet_gear
						cmd.drive.jerk = 0
				else:
					cmd.drive.speed = frenet_speed
					cmd.drive.steering_angle = frenet_angle
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 0
			#print('global mode!!!')

		elif car_mode == 'diagonal_parking':
			print(parking_yaw)
			if parking_flag == 'finish':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				print('parking finish!!! stop!!')
				rospy.sleep(4) # 4sec
				parking_flag = 'backward'
			elif parking_flag == 'end':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 50
				final_cmd_Pub.publish(cmd)
				mode_status = 'end'   # global mode로 바꾸기위한 flag를 파라미터 서버로 전달
				rospy.set_param('mission_status',mode_status)
				mission_pub.publish(end_msg)
				print('parking mission end!')
				rospy.sleep(1) # 1sec
			else:	
				if parking_yaw == 0:
					parking_yaw = yaw
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()
		
		elif car_mode == 'delivery_A' or car_mode == 'delivery_B':
			delivery_flag = delivery_decision()
			if delivery_flag == 'going':
				cmd.drive.speed = frenet_speed
				cmd.drive.steering_angle = frenet_angle
				cmd.drive.acceleration = frenet_gear
				cmd.drive.jerk = 0
			elif delivery_flag == 'end':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				mode_status = 'end'   # global mode로 바꾸기위한 flag를 파라미터 서버로 전달
				delivery_flag = 'going'
				rospy.set_param('mission_status',mode_status)
				print('delivery finish!!! stop!!')
				rospy.sleep(5) # 4sec
		


		#end_msg.data = mode_status
		final_cmd_Pub.publish(cmd)
		#mission_pub.publish(end_msg)

		rospy.sleep(0.1)



