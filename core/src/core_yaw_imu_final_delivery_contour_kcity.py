#!/usr/bin/env python
#-*- coding: utf-8 -*-

from os import access
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float64, String, Int32
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

link_ind = 0
global_wp = 0
def waypoint_callback(msg):
	global  link_ind, global_wp
	link_ind = msg.data[0]   #link index
	global_wp = msg.data[1]  #global waypoint

traffic_light = 0 
person = 0 
car = 0
def forward_callback(msg):
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

A_number = 0
A_x = [0,0,0]
B_x = [0,0,0]
def delivery_sign_callback(msg):
	global A_number, A_x, B_x
	A_number = msg.data[0]
	A_x = [msg.data[1], msg.data[2], msg.data[3]]
	B_x = [msg.data[4], msg.data[5], msg.data[6]]

def delivery_stopline_callback(msg):
	global stopline
	stopline = msg.data

def traffic_decision():
	global next_dir

	if next_dir == 'left':
		if traffic_light == 0 or traffic_light == 2 or traffic_light == 4:  # -1 : none   0 : green   1 : left   2 : red   3 : straightleft   4 : yellow
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 90
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
		if traffic_light == 1 or traffic_light == 2 or traffic_light == 4:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 90
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
		if traffic_light == 1 or traffic_light == 2 or traffic_light == 4:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 90
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

delivery_ind =0 
A_flag = False
B_flag = False
def delivery_decision():
	global delivery_ind, A_flag, B_flag
	
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
		if A_flag == False:
			if A_x[delivery_ind] > 315:   #parameter
				delivery_flag = 'wait'
			else:
				pass
			if delivery_flag == 'wait' and (stopline > 445 or A_x[delivery_ind] > 315):  #parameter
				delivery_flag = 'end'
				A_flag = True
			else:
				delivery_flag = 'going'
		else:
			delivery_flag = 'going'
	elif car_mode == 'delivery_B':
		if B_flag == False:
			if B_x[delivery_ind] > 315:   #parameter
				delivery_flag = 'wait'
			else:
				pass
			if delivery_flag == 'wait' and (stopline > 445 or B_x[delivery_ind] > 315):  #parameter
				delivery_flag = 'end'
				B_flag = True
			else:
				delivery_flag = 'going'
		else:
			delivery_flag = 'going'
	return delivery_flag


if __name__=='__main__':

	rospy.init_node('core_control')

	rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
	rospy.Subscriber("/forward_sign", Int32MultiArray, forward_callback)
	rospy.Subscriber('/side_sign',Int32MultiArray, delivery_sign_callback)
	rospy.Subscriber("/assist_steer", Float64, lanenet_callback)
	rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
	rospy.Subscriber("/odom_imu", Odometry, odometry_callback)
	rospy.Subscriber("/mode_selector",String,mode_callback,queue_size=10)
	rospy.Subscriber("/link_direction", StringArray, link_callback)
	rospy.Subscriber("/stopline", Int32, delivery_stopline_callback)
	rospy.Subscriber("/traffic_mode", String, traffic_callback)
		
	final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)

	cmd=AckermannDriveStamped()

	mode_status = 'going'
	notraffic_status = False
	r=rospy.Rate(10)
	while not rospy.is_shutdown():


		if car_mode == 'global':
			if traffic_mode == 'traffic':
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
				print('traffic mode')
			elif traffic_mode == 'notraffic':
				if notraffic_status == False:
					cmd.drive.speed = 0
					cmd.drive.steering_angle = 0
					cmd.drive.acceleration = 0
					cmd.drive.jerk = 90
					notraffic_status = True
					final_cmd_Pub.publish(cmd)
					print('no traffic mode')
					rospy.sleep(4) # 4sec
				elif notraffic_status == True:
					cmd.drive.speed = frenet_speed
					cmd.drive.steering_angle = frenet_angle
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 0
			else:
				notraffic_status = False
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
				print('global mode!!!')
			mode_status = 'going'
			rospy.set_param('mission_status', mode_status)  #혹시 안바뀌는걸 방지해 global일때 계속 주기적으로 mission status 바꿔줌
				

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
				rospy.sleep(5) # 5sec
			print('delivery mode')

		final_cmd_Pub.publish(cmd)

		r.sleep()



