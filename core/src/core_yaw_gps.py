#!/usr/bin/env python

from operator import ne
import rospy
import math
from tokenize import String
from std_msgs.msg import Int32MultiArray, Float64
from rocon_std_msgs.msg import StringArray 
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import numpy as np


global car_mode, move_mode, parking_yaw
car_mode = 'default'
move_mode = 'default'
parking_flag = 'default'
save_speed=[]
save_angle=[]
frenet_speed = 0
frenet_angle = 0 
frenet_gear = 0
backward_speed = 0
backward_angle = 0
backward_gear = 0
backward_brake = 0
deliveryA = 0
deliveryB = 0
traffic_light = 0 
person = 0 
car = 0
uturnsign = 0
kidzonesign = 0
parkingsign = 0
stopline = 0
assist_steer=0
waypoint = 0
w = 0
z = 0
parking_yaw = 0

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

def mode_selector_callback(msg):
	global mode_selector, car_mode, move_mode, cur_dir_mode, next_dir_mode
    
	mode_selector = msg.strings
	car_mode = mode_selector[0]
	move_mode = mode_selector[1]
	cur_dir_mode = mode_selector[2]
	next_dir_mode = mode_selector[3]
	#print("car_mode = ",car_mode, "move_mode = ", move_mode, "cur_dir_mode = ", cur_dir_mode, "next_dir_mode = ", next_dir_mode)

def lanenet_callback(msg):
    global assist_steer
    assist_steer = msg.data

def frenet_callback(msg):
    global frenet_speed, frenet_angle, frenet_gear
    frenet_speed = msg.drive.speed
    frenet_angle = msg.drive.steering_angle  
    frenet_gear = 0

# def parking_callback(msg):
#     global backward_speed, backward_angle, backward_gear, backward_brake
#     backward_speed = msg.drive.speed
#     backward_angle = msg.drive.steering_angle
#     backward_gear = msg.drive.acceleration
#     backward_brake = msg.drive.jerk

def waypoint_callback(msg):
	global waypoint
	waypoint = msg.data

def yolo_callback(msg):
	global deliveryA, deliveryB, traffic_light, person, car, uturnsign, kidzonesign, parkingsign, stopline
	deliveryA = msg.data[0]
	deliveryB = msg.data[1]
	traffic_light = msg.data[2]
	#print(traffic_light)
	person = msg.data[3]
	car = msg.data[4]
	uturnsign = msg.data[5]
	kidzonesign = msg.data[6]
	parkingsign = msg.data[7]
	stopline = msg.data[8]

def odometry_callback(msg):
	global yaw
	x = msg.pose.pose.orientation.x
	y = msg.pose.pose.orientation.y
	z = msg.pose.pose.orientation.z
	w = msg.pose.pose.orientation.w
	yaw = euler_from_quaternion(x, y, z, w)


def parking_decision():
	global parking_flag
	global back_speed, back_angle
	global save_speed,save_angle
	global move_mode, frenet_speed, frenet_angle, frenet_gear, backward_speed, backward_angle, backward_gear, backward_brake   
	global parking_angle, parking_brake, parking_speed, parking_gear, parking_yaw
 	
	if parking_flag == 'backward':
		if waypoint >= 20: # hyperparameter
			parking_speed = 8/3.6
			parking_angle = 0
			parking_gear = 2
			parking_brake = 0
		else:
			if abs(yaw - parking_yaw) < 5: # hyperparameter(degree)
				parking_speed = 0
				parking_angle = 0
				parking_gear = 0
				parking_brake = 100
				print('parking end')
				final_cmd_Pub.publish(cmd)
				rospy.sleep(2)
				parking_flag = 'end'
			else:
				parking_speed = 8/3.6
				parking_angle = -20*np.pi/180
				parking_gear = 2
				parking_brake = 0
	
	else:
		parking_speed = frenet_speed
		parking_angle = frenet_angle
		parking_gear = frenet_gear
		parking_brake = 0

	return parking_speed, parking_angle, parking_gear, parking_brake

def traffic_decision():
	global next_dir_mode

	if next_dir_mode == 'left':
		if traffic_light == 6 or traffic_light == 8 or traffic_light == 10:  # -1 : none   6 : green   7 : left   8 : red   9 : straightleft   10 : yellow
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 100
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
    
	elif next_dir_mode == 'straight':
		if traffic_light == 7 or traffic_light == 8 or traffic_light == 10:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 100
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
	elif next_dir_mode == 'right':
		if traffic_light == 7 or traffic_light == 8 or traffic_light == 10:
			traffic_speed = 0
			traffic_angle = 0
			traffic_gear = 0
			traffic_brake = 100
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

if __name__=='__main__':

	rospy.init_node('core_control')
	rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
	rospy.Subscriber("/mode_selector",StringArray,mode_selector_callback,queue_size=10)
	rospy.Subscriber("/detect_ID", Int32MultiArray, yolo_callback)
	rospy.Subscriber("/assist_steer", Float64, lanenet_callback)
	rospy.Subscriber("/waypoint", Float64, waypoint_callback)
	rospy.Subscriber("/odom", Odometry, odometry_callback)
	cmd=AckermannDriveStamped()
	final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)
	while not rospy.is_shutdown():
		if car_mode == 'global':
			if move_mode == 'finish':
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
			else:
				if parking_flag == 'backward':  # for parking
					cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()
				else:
					if cur_dir_mode == 'straight':
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

		elif car_mode == 'parking':
			if move_mode == 'forward': # parking forward -> frenet
				if parking_yaw == 0:
					parking_yaw = yaw
					if parking_yaw+np.pi <= np.pi:
						parking_yaw = parking_yaw + np.pi
					else:
						parking_yaw = parking_yaw - np.pi
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()
			elif move_mode == 'finish':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				print('parking finish!!! stop!!')
				rospy.sleep(5) # 5sec
				parking_flag = 'backward'
		rospy.sleep(0.1)
		final_cmd_Pub.publish(cmd)



