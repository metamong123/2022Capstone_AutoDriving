#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
import math
import rospkg
import sys
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
import time

from object_msgs.msg import Object, PathArray
from std_msgs.msg import Float64, Int32MultiArray,String
from rocon_std_msgs.msg import StringArray

from frenet import *
from stanley_pid import *

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *


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

	def update(self, a, delta):
		dt = self.dt
		WB = self.WB

		self.v += a * dt

		self.x += self.v * math.cos(self.yaw) * dt
		self.y += self.v * math.sin(self.yaw) * dt
		self.yaw += self.v / WB * math.tan(delta) * dt
		self.yaw = pi_2_pi(self.yaw)
		self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
		self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

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

def ros_msg(x, y, yaw, v):
	quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

	m = Marker()

	m.scale.x = 1.600
	m.scale.y = 1.650
	m.scale.z = 0.110

	o = Object()
	o.header.frame_id = "/map"
	o.header.stamp = rospy.Time.now()
	o.id = 1
	o.classification = o.CLASSIFICATION_CAR
	o.x = x
	o.y = y
	o.yaw = yaw
	o.v = v
	o.L = m.scale.x
	o.W = m.scale.y

	return {
		"quaternion": quat,
		"object_msg": o
	}

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
	if (msg.data == 'delivery_A') or (msg.data == 'delivery_B'):
		mode = 'delivery'
	elif (msg.data=='diagonal_parking') or (msg.data == 'horizontal_parking'):
		mode = 'parking'
	else:
		mode = msg.data

link_ind=0
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

# obj_msg=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=use_map.nodes['global'][start_index]['yaw'][0],v=0,L=1.600,W=1.04)

# def callback_state(msg):
# 	global obj_msg
# 	obj_msg=msg

dir='straight'
def callback_dir(msg):
	global dir
	dir=msg.strings[0]


if __name__ == "__main__":
	WB = 1.04
	# stanley = Stanley(k, speed_gain, w_yaw, w_cte,  cte_thresh = 0.5, p_gain = 1, i_gain = 1, d_gain = 1, WB = 1.04)
	
	control_gain=0.9
	cte_speed_gain=5
	yaw_weight=0.8
	cte_weight=0.9
	cte_thresh_hold=0
	yaw_d_gain=0

	stanley = Stanley(k=control_gain, speed_gain=cte_speed_gain, w_yaw=yaw_weight, w_cte=cte_weight,  cte_thresh = cte_thresh_hold, yaw_dgain = yaw_d_gain, WB = 1.04)
	#stanley = Stanley(0.5, 5, 0.9, 0.9,  cte_thresh = 0.5, p_gain = 1, i_gain = 1, d_gain = 1, WB = 1.04)
	t1 = time.time()

	rospy.init_node("control")
	tf_broadcaster = tf.TransformBroadcaster()

	control_pub = rospy.Publisher("/ackermann_cmd_frenet", AckermannDriveStamped, queue_size=1)
	accel_pub=rospy.Publisher("/accel", Float64, queue_size=1)
	object_pub = rospy.Publisher("/objects/car_1", Object, queue_size=1)


	# state_sub = rospy.Subscriber("/objects/car_1", Object, callback_state, queue_size=1)
	# path_sub= rospy.Subscriber("/final_path", PathArray, callback_path, queue_size=10)
	path_sub= rospy.Subscriber("/optimal_frenet_path_global", PathArray, callback_path, queue_size=1)
	# mode_sub= rospy.Subscriber("/mode_selector", String, callback_mode, queue_size=1)
	waypoint_link_sub= rospy.Subscriber("/waypoint", Int32MultiArray, callback_wp_link_ind, queue_size=1)
	dir_sub=rospy.Subscriber("/link_direction", StringArray, callback_dir, queue_size=1)

	accel_msg = Float64()

	s=0
	d=0
	x=0
	y=0
	road_yaw=0
	park_ind=0
	v=0

	state=State(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=use_map.nodes['global'][start_index]['yaw'][0],v=1,WB=1.04, dt=0.1)
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(10)
	a = 0

	#f = open("/home/mds/stanley/k1.csv", "w")
	if dir == 'right' or dir == 'left':
		dir='curve'
	
	while not rospy.is_shutdown():

		if not path_x: ## No solution
			if mode == 'global':
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],my_wp)
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind]])
				
				steer = road_yaw - state.yaw
				a = 0
		else:
			## PID control
			if dir == 'right' or dir == 'left':
				dir='curve'
			
			if mode == 'global':
				error_pa = use_map.target_speed[mode][dir] - state.v
				error_da = state.v - prev_v
				error_ia += use_map.target_speed[mode][dir] - state.v
			else:
				error_pa = use_map.target_speed[mode] - state.v
				error_da = state.v - prev_v
				error_ia += use_map.target_speed[mode] - state.v
			
			kp_a = 0.5
			kd_a = 0.7
			ki_a = 0.01
			
			a = kp_a * error_pa + kd_a * error_da + ki_a * error_ia
			
			# stanley_control / stanley_control_thresh / stanley_control_pid
			steer, yaw_term, cte, map_yaw = stanley.stanley_control_pd(state.x, state.y, state.yaw, state.v, path_x, path_y, path_yaw)
			# if mode == 'global':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, path_x,path_y,path_yaw)
			# elif mode == 'parking':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, use_map.parking_path[park_ind][0],use_map.parking_path[park_ind][1],use_map.parking_path[park_ind][1])
			# elif mode == 'delivery':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, use_map.delivery_path[delivery_ind][0],use_map.delivery_path[delivery_ind][1],use_map.delivery_path[delivery_ind][1])
			
			# dt, yaw_term, cte, steer
			#f.write(str(t1) + ',' + str(yaw_term*180/math.pi) + ',' + str(cte*1e2) + ',' + str(steer*180/math.pi) + '\n')

		accel_msg.data = a

		state.update(a, steer)
		o_msg = ros_msg(state.x, state.y, state.yaw, state.v)
		msg = state.get_ros_msg(a, steer, id=id)
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer) + ",a = "+str(a))
		prev_v = state.v

		# send tf
		tf_broadcaster.sendTransform(
			(state.x, state.y, 1.5),
			o_msg["quaternion"],
			rospy.Time.now(),
			"/car_" + str(id), "/map"
		)

		object_pub.publish(o_msg["object_msg"])
		control_pub.publish(msg)
		accel_pub.publish(accel_msg)

		r.sleep()
