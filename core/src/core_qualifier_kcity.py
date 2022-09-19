#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Int32MultiArray, Float64, String, Int32
from rocon_std_msgs.msg import StringArray 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes

import numpy as np


parking_flag = 'forward'

waypoint = 0
x = 0
y = 0
z = 0
w = 0
yaw = 0

##########################################################################

# parking 시작하기전에 수정해야할 파라미터 값들 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
parking_finish_wp=[54,54,54,54,54,54] # 각 parking index마다의 finish waypoint임
parking_straight_back_wp=[50,50,50,50,50,50]

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

link_ind = 0
global_wp = 0
def waypoint_callback(msg):
	global  link_ind, global_wp
	link_ind = msg.data[0]   #link index
	global_wp = msg.data[1]  #global waypoint

traffic_light = 0 
def forward_callback(msg):
	global  traffic_light
	traffic_light = msg.data[0]

def odometry_callback(msg):
	global x, y, z, w, yaw
	x = msg.pose.pose.orientation.x
	y = msg.pose.pose.orientation.y
	z = msg.pose.pose.orientation.z
	w = msg.pose.pose.orientation.w
	yaw = euler_from_quaternion(x, y, z, w)

park_ind_wp = [0,0]
def parking_callback(msg):
	global park_ind_wp
	park_ind_wp = msg.data

col = 0
def col_callback(msg):
	global col
	col = msg.data

traffic_slow = 'no'
def slow_callback(msg):
    global traffic_slow
    traffic_slow = msg.data
    

parking_angle = 0
parking_speed = 0
parking_brake = 0 
parking_yaw = 0
parking_gear = 0
j=0
def parking_decision():
	global parking_flag
	global back_speed, back_angle
	global save_speed,save_angle
	global move_mode, frenet_speed, frenet_angle, frenet_gear  
	global parking_angle, parking_brake, parking_speed, parking_gear, parking_yaw, yaw, j

	if parking_flag == 'backward':
		#print(yaw)
		if abs(yaw - parking_yaw) < 3*np.pi/180: # hyperparameter(degree)
			parking_flag = 'end'
		else:
			if park_ind_wp[1] >= parking_straight_back_wp[park_ind_wp[0]]:
				parking_speed = 8/3.6
				parking_angle = 0
				parking_gear = 2
				parking_brake = 0	
			else:
				parking_speed = 8/3.6
				parking_angle = -28*np.pi/180
				parking_gear = 2
				parking_brake = 0
	
	else:
		if park_ind_wp[1] >= parking_finish_wp[park_ind_wp[0]]:
			parking_flag = 'finish'
		else:
			if abs(frenet_angle) > 0.1: #각도 파라미터
				if j<100: # 감속
					parking_speed = frenet_speed
					parking_angle = frenet_angle
					parking_gear = frenet_angle
					parking_brake = 10
				else:
					parking_speed = frenet_speed
					parking_angle = frenet_angle
					parking_gear = frenet_gear
					parking_brake = 0
			else:
				parking_speed = frenet_speed
				parking_angle = frenet_angle
				parking_gear = frenet_gear
				parking_brake = 0
				j=0
	return parking_speed, parking_angle, parking_gear, parking_brake

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
			traffic_speed = 2 * frenet_speed / 3
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
			traffic_speed = 2 * frenet_speed / 3
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
			traffic_speed = frenet_speed / 2
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 40
			print("traffic mode : none")
		else :
			traffic_speed = 2 * frenet_speed / 3
			traffic_angle = frenet_angle
			traffic_gear = 0
			traffic_brake = 20
			print("traffic mode : go")
	return traffic_speed, traffic_angle, traffic_gear, traffic_brake


if __name__=='__main__':

	rospy.init_node('core_control')

	rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
	rospy.Subscriber("/forward_sign", Int32MultiArray, forward_callback)
	rospy.Subscriber("/waypoint", Int32MultiArray, waypoint_callback)
	rospy.Subscriber("/odom_imu", Odometry, odometry_callback)
	rospy.Subscriber("/park_ind_wp", Int32MultiArray, parking_callback)
	rospy.Subscriber("/mode_selector",String,mode_callback,queue_size=10)
	rospy.Subscriber("/link_direction", StringArray, link_callback)
	rospy.Subscriber("/col", Int32, col_callback)
	rospy.Subscriber("/traffic_mode", String, traffic_callback)	
	rospy.Subscriber("/traffic_slow", String, slow_callback)
 
	final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)
	
	r = rospy.Rate(20)
	j = 0
	mode_status = 'going'
	notraffic_status =  False
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
					rospy.sleep(4) # 4sec
				elif notraffic_status == True:
					if abs(frenet_angle) > 0.1: #각도 파라미터
						if j<100:  #감속
							cmd.drive.speed = frenet_speed/2
							cmd.drive.steering_angle = frenet_angle
							cmd.drive.acceleration = frenet_gear
							cmd.drive.jerk = 20
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
				if traffic_slow == 'slow':
					cmd.drive.speed = frenet_speed/2
					cmd.drive.steering_angle = frenet_angle
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 40
				else:
					if abs(frenet_angle) > 0.1: #각도 파라미터
						if j<100:  #감속
							cmd.drive.speed = frenet_speed/2
							cmd.drive.steering_angle = frenet_angle
							cmd.drive.acceleration = frenet_gear
							cmd.drive.jerk = 20  # 예선은 고정! 바꾸지말기
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
				notraffic_status = False # notraffic 구간이 여러번 있으니 바꿔줘야함
				print('global mode!!!')
			mode_status = 'going'
			
		elif car_mode == 'diagonal_parking':
			#print(parking_yaw)
			if parking_flag == 'finish':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				print('parking finish!!! stop!!')
				rospy.sleep(5) # 10sec
				parking_flag = 'backward'
			elif parking_flag == 'end':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 80
				final_cmd_Pub.publish(cmd)
				mode_status = 'end'   # global mode로 바꾸기위한 flag를 파라미터 서버로 전달
				status_msg.data = mode_status
				status_Pub.publish(status_msg)
				print('parking mission end!')
				rospy.sleep(1) # 1sec
			else:	
				if parking_yaw == 0:
					parking_yaw = yaw
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()
			print('diagonal parking mode')

		elif car_mode == 'dynamic_object':
			if col == 0:
				if abs(frenet_angle) > 0.1: #각도 파라미터
					if j<100:  #감속
						cmd.drive.speed = frenet_speed
						cmd.drive.steering_angle = frenet_angle
						cmd.drive.acceleration = frenet_gear
						cmd.drive.jerk = 20
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
				cmd.drive.jerk = 200  #full brake
				print("Dynamic Obstacle Discovery !!")
		elif car_mode == 'static_object':
			cmd.drive.speed = frenet_speed
			cmd.drive.steering_angle = frenet_angle
			cmd.drive.acceleration = frenet_gear
			cmd.drive.jerk = 0

		status_msg.data = mode_status
		status_Pub.publish(status_msg)
		final_cmd_Pub.publish(cmd)

		r.sleep()