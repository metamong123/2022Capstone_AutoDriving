#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import math
import rospkg
import sys
from ackermann_msgs.msg import AckermannDriveStamped
import time

from object_msgs.msg import Object, PathArray
from std_msgs.msg import Float64, Int32MultiArray,String

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *


def pi_2_pi(angle):
	return (angle + math.pi) % (2 * math.pi) - math.pi

class State:

	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.05, WB=1.04):
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
		c.drive.speed = v + a*dt

		return c



path_x=[]
path_y=[]
path_yaw=[]
def callback_path(msg):
	global path_x,path_y,path_yaw
	global t1
	path_x=msg.x.data
	path_y=msg.y.data
	path_yaw=msg.yaw.data
	t1 = time.time()
	

mode='global'
def callback_mode(msg):
	global mode
	mode = msg.data

link_ind=2
my_wp=0
def callback_wp_link_ind(msg):
	global link_ind, my_wp
	link_ind=msg.data[0]
	my_wp=msg.data[1]

delivery_ind=0
def callback_delivery_ind(msg):
	global delivery_ind
	delivery_ind=msg.data

def acceleration(ai):
	a=Float64()
	a.data=ai


use_map=frontier()
start_index=link_ind
obj_msg=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=0,v=0,L=1.600,W=1.04)

def callback_state(msg):
	global obj_msg
	obj_msg=msg

if __name__ == "__main__":
	WB = 1.04
	# stanley = Stanley(k, speed_gain, w_yaw, w_cte,  cte_thresh = 0.5, p_gain = 1, i_gain = 1, d_gain = 1, WB = 1.04)
	control_gain=0.5
	cte_speed_gain=5
	yaw_weight=0.8
	cte_weight=0.8
	cte_thresh_hold=0.15
	yaw_d_gain=0.7

	stanley = Stanley(k=control_gain, speed_gain=cte_speed_gain, w_yaw=yaw_weight, w_cte=cte_weight,  cte_thresh = cte_thresh_hold, yaw_dgain = yaw_d_gain, WB = 1.04)
	
	f = open("/home/mds/stanley/"+time.strftime('%Y%m%d_%H:%M')+"_k"+str(control_gain)+"_sg"+str(cte_speed_gain)+"_wy"+str(yaw_weight)+"_wc"+str(cte_weight)+"_thresh"+str(cte_thresh_hold)+"_dgain"+str(yaw_d_gain)+".csv", "w")
	f.write('time' +',' + 'x'+ ',' + 'y' + ',' + 'yaw_term(degree)' + ',' + 'cte(cm)' + ',' + 'steering(degree)' + '\n')
	
	t1 = time.time()

	rospy.init_node("control")

	control_pub = rospy.Publisher("/ackermann_cmd_frenet", AckermannDriveStamped, queue_size=1)
	accel_pub=rospy.Publisher("/accel", Float64, queue_size=1)


	state_sub = rospy.Subscriber("/objects/car_1", Object, callback_state, queue_size=1)
	#path_sub= rospy.Subscriber("/final_path", PathArray, callback_path, queue_size=1)
	path_sub= rospy.Subscriber("/optimal_frenet_path_global", PathArray, callback_path, queue_size=1)
	mode_sub= rospy.Subscriber("/mode_selector", String, callback_mode, queue_size=1)
	waypoint_link_sub= rospy.Subscriber("/waypoint", Int32MultiArray, callback_wp_link_ind, queue_size=1)
	
	accel_msg = Float64()
	

	s=0
	d=0
	x=0
	y=0
	road_yaw=0
	park_ind=0
	v=0

	state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=1, dt=0.05)
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(20)
	a = 0
	steer=0
	msg = state.get_ros_msg(a, steer, id=id)

	while not rospy.is_shutdown():
		
		state=State(x=obj_msg.x, y=obj_msg.y, yaw=obj_msg.yaw, v=msg.drive.speed, dt=0.05)
		
		if not path_x: ## No solution
			if mode == 'global':
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],my_wp)
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind]])
				
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
			steer, yaw_term, cte = stanley.stanley_control_pd(state.x, state.y, state.yaw, state.v, path_x, path_y, path_yaw)
			# stanley_control / stanley_control_thresh / stanley_control_pid
			
			# if mode == 'global':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, path_x,path_y,path_yaw)
			# elif mode == 'parking':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, use_map.parking_path[park_ind][0],use_map.parking_path[park_ind][1],use_map.parking_path[park_ind][1])
			# elif mode == 'delivery':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, use_map.delivery_path[delivery_ind][0],use_map.delivery_path[delivery_ind][1],use_map.delivery_path[delivery_ind][1])
			
			f.write(str(t1) + ',' + str(state.x) + ',' + str(state.y) + ',' + str(yaw_term*180/math.pi) + ',' + str(cte*1e2) + ',' + str(steer*180/math.pi) + '\n')

		accel_msg.data = a
		
		msg = state.get_ros_msg(a, steer, id=id)
		
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer) + ",a = "+str(a))
		prev_v = state.v

		control_pub.publish(msg)
		accel_pub.publish(accel_msg)

		r.sleep()
