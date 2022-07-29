#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import math
import rospkg
import sys
from ackermann_msgs.msg import AckermannDriveStamped

from object_msgs.msg import Object, PathArray
from std_msgs.msg import Float64, Int32MultiArray,Float64MultiArray
from rocon_std_msgs.msg import StringArray

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from global_path import *


def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi

class State:

	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, WB=1.04):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
		self.dt = dt
		self.WB = WB

	def get_ros_msg(self, a, steer, id):
		dt = self.dt
		v = self.v

		c = AckermannDriveStamped()
		c.header.frame_id = "/map"
		c.header.stamp = rospy.Time.now()
		c.drive.steering_angle = steer
		c.drive.acceleration = a
		c.drive.speed = v

		return c

def callback_state(msg):
	global obj_msg
	obj_msg=msg

path_x=[]
path_y=[]
path_yaw=[]
def callback_path(msg):
	global path_x,path_y,path_yaw
	path_x=msg.x.data
	path_y=msg.y.data
	path_yaw=msg.yaw.data
	

mode='global'
def callback_mode(msg):
	global mode
	mode = msg.strings[0]

link_ind=0
def callback_link_ind(msg):
	global link_ind
	link_ind=msg.data[1]

def acceleration(ai):
	a=Float64()
	a.data=ai

use_map=kcity()
start_index=link_ind
obj_msg=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=0,v=0,L=1.600,W=1.04)

if __name__ == "__main__":
	WB = 1.04

	rospy.init_node("control")

	control_pub = rospy.Publisher("/ackermann_cmd_frenet", AckermannDriveStamped, queue_size=1)
	accel_pub=rospy.Publisher("/acceleration", Float64, queue_size=1)
	state_sub = rospy.Subscriber("/objects/car_1", Object, callback_state, queue_size=1)

	path_sub= rospy.Subscriber("/optimal_frenet_path", PathArray, callback_path, queue_size=10)
	mode_sub= rospy.Subscriber("/mode_selector", StringArray, callback_mode, queue_size=1)
	link_sub= rospy.Subscriber("/waypoint", Int32MultiArray, callback_link_ind, queue_size=1)

	s=0
	d=0
	x=0
	y=0
	road_yaw=0
	no_solution =[]
	error_icte=0
	prev_cte =0
	cte = 0

	v=0
	my_wp={'global':0, 'parking':0,'delivery':0}
	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.1)
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(10)
	a = 0

	while not rospy.is_shutdown():

		if not path_x: ## No solution
			if mode == 'global':
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],my_wp[mode])
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind]])
			else:
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode][link_ind]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode][link_ind]['y'][:use_map.link_len[mode][link_ind]],my_wp[mode])
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode][link_ind]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode][link_ind]['y'][:use_map.link_len[mode][link_ind]],use_map.waypoints[mode][link_ind]['s'][:use_map.link_len[mode][link_ind]])

			steer = road_yaw - state.yaw
			a = 0
		else:
			## PID control
			error_pa = use_map.target_speed[mode] - state.v
			error_da = state.v - prev_v
			error_ia += use_map.target_speed[mode] - state.v
			kp_a = 0.5
			kd_a = 0.7
			ki_a = 0.01
			a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia
			prev_cte = cte
			error_icte += cte
			
			steer, cte, _ = stanley_control(state.x, state.y, state.yaw, state.v, path_x,path_y,path_yaw, WB, error_icte, prev_cte)

		accel_msg=acceleration(a)
		# state.update(a, steer)
		
		msg = state.get_ros_msg(a, steer, id=id)
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer) + ",a = "+str(a))
		prev_v = state.v

		control_pub.publish(msg)
		accel_pub.publish(accel_msg)

		r.sleep()