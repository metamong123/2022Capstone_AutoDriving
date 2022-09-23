#!/usr/bin/python
#-*- coding: utf-8 -*-

from operator import le
import rospy, tf
import rospkg
import sys
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
import pickle

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")

from path_map import *

class Converter(object):
	def __init__(self, file_=None, waypoints=None, obj=None,start_id=0, r=255/255.0, g=255/255.0, b=255/255.0, a= 0.5, scale=0.1):
		self.r = r
		self.g = g
		self.b = b
		self.a = a
		self.scale = scale
		self.start_id = start_id
		self.ma = None
		self.mo = None
		if file_ is not None:
			self.file = file_
			with open(file_, "rb") as f:
				self.waypoints = pickle.load(f)
			self.make_marker_array(self.waypoints)
		elif waypoints is not None:
			self.waypoints = waypoints
			self.make_marker_array(self.waypoints)
		elif obj is not None:
			self.obj=obj
		else:
			self.waypoints = None

	def make_marker_array(self, waypoints):
		ma = MarkerArray()
		#draw_step = 1
		#if isinstance(self.waypoints, dict):
			# for A2_LINK_smoothed_near0.pkl
			#waypoints = self.waypoints.values()
			#draw_step = 5

		for j in range(len(waypoints["x"])):
			#if j % draw_step == 0 or j == len(waypoint["x"])-1:
			m = Marker()
			m.id = j
			m.header.frame_id = "/map"
			m.type = m.SPHERE
			m.scale.x = self.scale
			m.scale.y = self.scale
			m.scale.z = self.scale
			m.action = m.ADD

			m.color.r = self.r
			m.color.g = self.g
			m.color.b = self.b
			m.color.a = self.a

			m.pose.orientation.x = 0
			m.pose.orientation.y = 0
			m.pose.orientation.z = 0
			m.pose.orientation.w = 1

			m.pose.position.x = waypoints["x"][j]
			m.pose.position.y = waypoints["y"][j]
			m.pose.position.z = 0

			ma.markers.append(m)

		self.ma = ma

def msg_pub(i, area):
	x=(area['x'][0]+area['x'][2])/2
	y=(area['y'][0]+area['y'][1])/2
	a = np.sqrt((area['x'][0]-area['x'][1])**2+(area['y'][0]-area['y'][1])**2)
	b = np.sqrt((area['x'][2]-area['x'][1])**2+(area['y'][2]-area['y'][1])**2)
	if a > b:
		Le=a
		Wi=b
	else:
		Wi=a
		Le=b
	yaw=use_map.waypoints['horizontal_parking'][0]['yaw'][0]
	yaw_quat = tf.transformations.quaternion_from_euler(0,0,yaw)

	m = Marker()
	m.header.frame_id = "/map"
	m.header.stamp = rospy.Time.now()
	m.id = 100+i
	m.type = m.CUBE

	m.pose.position.x = x
	m.pose.position.y = y
	m.pose.position.z = 0.1
	m.pose.orientation = Quaternion(*yaw_quat)

	m.scale.x = Le
	m.scale.y = Wi
	m.scale.z = 1

	m.color.r = 100 / 255.0
	m.color.g = 250 / 255.0
	m.color.b = 177 / 255.0
	m.color.a = 0.5

	return m

if __name__ == "__main__":
	rospy.init_node("map_rviz_visualizer")
	path = rospack.get_path("map_server")

	# TODO: AS CONFIGURATION FILE
	global_wayp = use_map.waypoints['global']

	if not use_map.horizontal_parking_map_num==0:
		for i in range(use_map.horizontal_parking_map_num):
			globals()["horizontal_parking_wayp{}".format(i)]=use_map.waypoints['horizontal_parking'][i*2]
	if not use_map.diagonal_parking_map_num==0:
		for i in range(use_map.diagonal_parking_map_num):
			globals()["diagonal_parking_wayp{}".format(i)]=use_map.waypoints['diagonal_parking'][i*2]
	if not use_map.delivery_map_num==0:
		for i in range(use_map.delivery_map_num):
			globals()["delivery_wayp{}".format(i)]=use_map.waypoints['delivery'][i]

	if not use_map.horizontal_parking_map_num==0:
		for i in range(use_map.horizontal_parking_map_num):
			globals()["horizontal_parking_obj{}".format(i)]=use_map.horizontal_parking_object[i]
		
	global_cv = Converter(waypoints=global_wayp, r=255/255.0, g=236/255.0, b=139/255.0, a=0.8, scale=0.2)

	if not use_map.horizontal_parking_map_num==0:
		for i in range(use_map.horizontal_parking_map_num):
			globals()["horizontal_parking_cv_{}".format(i)]=Converter(waypoints=globals()["horizontal_parking_wayp{}".format(i)], r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=0.8, scale=0.2)
	if not use_map.diagonal_parking_map_num==0:
		for i in range(use_map.diagonal_parking_map_num):
			globals()["diagonal_parking_cv_{}".format(i)]=Converter(waypoints=globals()["diagonal_parking_wayp{}".format(i)], r=228 / 255.0, g=133 / 255.0, b=137 / 255.0, a=0.8, scale=0.2)
	if not use_map.delivery_map_num==0:
		for i in range(use_map.delivery_map_num):
			globals()["delivery_cv_{}".format(i)]=Converter(waypoints=globals()["delivery_wayp{}".format(i)], r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=0.8, scale=0.2)

	global_pub = rospy.Publisher("/rviz/global_links", MarkerArray, queue_size=1.2, latch=True)
	
	if not use_map.horizontal_parking_map_num==0:
		for i in range(use_map.horizontal_parking_map_num):
			horizontal_parking_topic="/rviz/horizontal_parking_link_"+str(i)
			globals()["horizontal_parking_pub_{}".format(i)]=rospy.Publisher(horizontal_parking_topic, MarkerArray, queue_size=1.2, latch=True)
	if not use_map.diagonal_parking_map_num==0:
		for i in range(use_map.diagonal_parking_map_num):
			diagonal_parking_topic="/rviz/diagonal_parking_link_"+str(i)
			globals()["diagonal_parking_pub_{}".format(i)]=rospy.Publisher(diagonal_parking_topic, MarkerArray, queue_size=1.2, latch=True)
	if not use_map.delivery_map_num==0:
		for i in range(use_map.delivery_map_num):
			delivery_topic="/rviz/delivery_link_"+str(i)
			globals()["delivery_pub_{}".format(i)]=rospy.Publisher(delivery_topic, MarkerArray, queue_size=1.2, latch=True)
	if not use_map.horizontal_parking_map_num==0:
		for i in range(use_map.horizontal_parking_map_num):
			obj_topic="/rviz/horizontal_object_"+str(i)
			globals()["horizontal_parking_obj_pub_{}".format(i)]=rospy.Publisher(obj_topic, Marker,queue_size=1)

	while not rospy.is_shutdown():
		global_pub.publish(global_cv.ma)
		if not use_map.horizontal_parking_map_num==0:
			for i in range(use_map.horizontal_parking_map_num):
				globals()["horizontal_parking_pub_{}".format(i)].publish(globals()["horizontal_parking_cv_{}".format(i)].ma)
		if not use_map.diagonal_parking_map_num==0:
			for i in range(use_map.diagonal_parking_map_num):
				globals()["diagonal_parking_pub_{}".format(i)].publish(globals()["diagonal_parking_cv_{}".format(i)].ma)
		if not use_map.delivery_map_num==0:
			for i in range(use_map.delivery_map_num):
				globals()["delivery_pub_{}".format(i)].publish(globals()["delivery_cv_{}".format(i)].ma)
		if not use_map.horizontal_parking_map_num==0:
			for i in range(use_map.horizontal_parking_map_num):
				m=msg_pub(i, use_map.horizontal_parking_object[i])
				globals()["horizontal_parking_obj_pub_{}".format(i)].publish(m)
		
		rospy.sleep(1)
