#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import pickle

rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")

from path_map import *

class Converter(object):
	def __init__(self, file_=None, waypoints=None, start_id=0, r=255/255.0, g=255/255.0, b=255/255.0, a= 0.5, scale=0.1):
		self.r = r
		self.g = g
		self.b = b
		self.a = a
		self.scale = scale
		self.start_id = start_id
		self.ma = None
		if file_ is not None:
			self.file = file_
			with open(file_, "rb") as f:
				self.waypoints = pickle.load(f)
			self.make_marker_array(self.waypoints)
		elif waypoints is not None:
			self.waypoints = waypoints
			self.make_marker_array(self.waypoints)
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

	global_cv = Converter(waypoints=global_wayp, r=255/255.0, g=236/255.0, b=139/255.0, a=0.8, scale=0.1)

	if not use_map.horizontal_parking_map_num==0:
		for i in range(use_map.horizontal_parking_map_num):
			globals()["horizontal_parking_cv_{}".format(i)]=Converter(waypoints=globals()["horizontal_parking_wayp{}".format(i)], r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=0.8, scale=0.1)
	if not use_map.diagonal_parking_map_num==0:
		for i in range(use_map.diagonal_parking_map_num):
			globals()["diagonal_parking_cv_{}".format(i)]=Converter(waypoints=globals()["diagonal_parking_wayp{}".format(i)], r=228 / 255.0, g=133 / 255.0, b=137 / 255.0, a=0.8, scale=0.1)
	if not use_map.delivery_map_num==0:
		for i in range(use_map.delivery_map_num):
			globals()["delivery_cv_{}".format(i)]=Converter(waypoints=globals()["delivery_wayp{}".format(i)], r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=0.8, scale=0.1)

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

	rospy.sleep(1)
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
		
		rospy.sleep(1)
