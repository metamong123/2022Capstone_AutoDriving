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
from rocon_std_msgs.msg import StringArray


from frenet import *
from stanley_pd import *

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
	path_x=msg.x.data
	path_y=msg.y.data
	path_yaw=msg.yaw.data
	

mode='global'
def callback_mode(msg):
	global mode
	if (msg.data == 'delivery_A') or (msg.data == 'delivery_B'):
		mode = 'delivery'
	elif (msg.data == 'diagonal_parking') or (msg.data == 'horizontal_parking'):
		mode = 'parking'
	else:
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


# obj_msg_gps=obj_msg
# obj_msg_imu=obj_msg
# obj_msg_gps=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=use_map.nodes[mode][start_index]['yaw'][0],v=0,L=1.600,W=1.04)
# obj_msg_imu=Object(x=use_map.nodes[mode][start_index]['x'][0],y=use_map.nodes[mode][start_index]['y'][0],yaw=use_map.nodes[mode][start_index]['yaw'][0],v=0,L=1.600,W=1.04)

def callback_state_gps(msg):
	global obj_msg_gps, t1
	obj_msg_gps=msg
	t1 = time.time()

def callback_state_imu(msg):
	global obj_msg_imu, t2
	obj_msg_imu=msg
	t2 = time.time()

dir='straight'
def callback_dir(msg):
	global dir
	dir=msg.strings[0]

if __name__ == "__main__":
	WB = 1.04
	# stanley = Stanley(k, speed_gain, w_yaw, w_cte,  cte_thresh = 0.5, p_gain = 1, i_gain = 1, d_gain = 1, WB = 1.04)
	control_gain={'global':1,'parking':10}
	cte_speed_gain=0
	yaw_weight=1
	cte_weight=1
	cte_thresh_hold=0
	yaw_d_gain={'global':0,'parking':0.5}

	stanley_gps = Stanley(k=control_gain[mode], speed_gain=cte_speed_gain, w_yaw=yaw_weight, w_cte=cte_weight,  cte_thresh = cte_thresh_hold, yaw_dgain = yaw_d_gain[mode], WB = 1.04)
	stanley_imu = Stanley(k=control_gain[mode], speed_gain=cte_speed_gain, w_yaw=yaw_weight, w_cte=cte_weight,  cte_thresh = cte_thresh_hold, yaw_dgain = yaw_d_gain[mode], WB = 1.04)
	
	f_gps = open("/home/mds/stanley/"+"gps_"+time.strftime('%Y%m%d_%H:%M')+"_k"+str(control_gain)+"_sg"+str(cte_speed_gain)+"_wy"+str(yaw_weight)+"_wc"+str(cte_weight)+"_thresh"+str(cte_thresh_hold)+"_dgain"+str(yaw_d_gain)+".csv", "w")
	f_gps.write('time' + ',' + 'x' + ',' + 'y' + ',' + 'map_yaw' + ',' + 'yaw' + ',' + 'yaw_term(degree)' + ',' + 'cte(cm)' + ',' + 'steering(degree)' + '\n')

	f_imu = open("/home/mds/stanley/"+"imu_"+time.strftime('%Y%m%d_%H:%M')+"_k"+str(control_gain)+"_sg"+str(cte_speed_gain)+"_wy"+str(yaw_weight)+"_wc"+str(cte_weight)+"_thresh"+str(cte_thresh_hold)+"_dgain"+str(yaw_d_gain)+".csv", "w")
	f_imu.write('time' + ',' + 'x' + ',' + 'y' + ',' + 'map_yaw' + ',' + 'yaw' + ',' + 'yaw_term(degree)' + ',' + 'cte(cm)' + ',' + 'steering(degree)' + '\n')
	
	t1 = time.time()
	t2 = time.time()

	rospy.init_node("control")

	control_pub = rospy.Publisher("/ackermann_cmd_frenet", AckermannDriveStamped, queue_size=1)
	accel_pub=rospy.Publisher("/accel", Float64, queue_size=1)


	state_gps_sub = rospy.Subscriber("/objects/car_1/gps", Object, callback_state_gps, queue_size=1)
	state_imu_sub = rospy.Subscriber("/objects/car_1/imu", Object, callback_state_imu, queue_size=1)
	path_sub= rospy.Subscriber("/final_path", PathArray, callback_path, queue_size=1)
	#path_sub= rospy.Subscriber("/optimal_frenet_path_global", PathArray, callback_path, queue_size=1)
	mode_sub= rospy.Subscriber("/mode_selector", String, callback_mode, queue_size=1)
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

	state=State(x=obj_msg_gps.x, y=obj_msg_gps.y, yaw=obj_msg_gps.yaw, v=2, dt=0.05)
	prev_v = state.v
	error_ia = 0
	r = rospy.Rate(20)
	a = 0
	steer_gps=0
	msg = state.get_ros_msg(a, steer_gps, id=id)
	if dir == 'right' or dir == 'left':
		dir='curve'

	while not rospy.is_shutdown():
		
		state=State(x=obj_msg_gps.x, y=obj_msg_gps.y, yaw=obj_msg_gps.yaw, v=msg.drive.speed, dt=0.05)
		
		if not path_x: ## No solution
			if mode == 'global':
				s, d = get_frenet(state.x, state.y, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],my_wp)
				x, y, road_yaw = get_cartesian(s, d, use_map.waypoints[mode]['x'][:use_map.link_len[mode][link_ind]], use_map.waypoints[mode]['y'][:use_map.link_len[mode][link_ind]],use_map.waypoints[mode]['s'][:use_map.link_len[mode][link_ind]])
				
				steer_gps = road_yaw - state.yaw
				# steer_gps, yaw_term_gps, cte_gps, map_yaw_gps = stanley_gps.stanley_control_pd(state.x, state.y, state.yaw, state.v, [x], [y], [road_yaw])
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
			steer_gps, yaw_term_gps, cte_gps, map_yaw_gps = stanley_gps.stanley_control_pd(obj_msg_gps.x, obj_msg_gps.y, obj_msg_gps.yaw, state.v, path_x, path_y, path_yaw)
			steer_imu, yaw_term_imu, cte_imu, map_yaw_imu = stanley_imu.stanley_control_pd(obj_msg_imu.x, obj_msg_imu.y, obj_msg_imu.yaw, state.v, path_x, path_y, path_yaw)
			# stanley_control / stanley_control_thresh / stanley_control_pid
			
			# if mode == 'global':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, path_x,path_y,path_yaw)
			# elif mode == 'parking':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, use_map.parking_path[park_ind][0],use_map.parking_path[park_ind][1],use_map.parking_path[park_ind][1])
			# elif mode == 'delivery':
			# 	steer = stanley.stanley_control(state.x, state.y, state.yaw, state.v, use_map.delivery_path[delivery_ind][0],use_map.delivery_path[delivery_ind][1],use_map.delivery_path[delivery_ind][1])
			
			f_gps.write(str(t1) + ',' + str(obj_msg_gps.x) + ',' + str(obj_msg_gps.y) + ',' + str(map_yaw_gps*180/math.pi) + ',' + str(obj_msg_gps.yaw*180/math.pi) + ',' + str(yaw_term_gps*180/math.pi) + ',' + str(cte_gps*1e2) + ',' + str(steer_gps*180/math.pi) + '\n')
			f_imu.write(str(t2) + ',' + str(obj_msg_imu.x) + ',' + str(obj_msg_imu.y) + ',' + str(map_yaw_imu*180/math.pi) + ',' + str(obj_msg_imu.yaw*180/math.pi) + ',' + str(yaw_term_imu*180/math.pi) + ',' + str(cte_imu*1e2) + ',' + str(steer_imu*180/math.pi) + '\n')

		accel_msg.data = a
		
		msg = state.get_ros_msg(a, steer_gps, id=id)
		
		print("현재 speed = " + str(state.v) + "명령 speed = " + str(msg.drive.speed) + ",steer = " + str(steer_gps) + ",a = "+str(a))
		prev_v = state.v

		control_pub.publish(msg)
		accel_pub.publish(accel_msg)

		r.sleep()
