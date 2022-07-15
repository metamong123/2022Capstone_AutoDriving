#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import pickle


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
		draw_step = 1
		if isinstance(self.waypoints, dict):
			# for A2_LINK_smoothed_near0.pkl
			waypoints = self.waypoints.values()
			draw_step = 5
		
		for i, waypoint in enumerate(waypoints):
			m = Marker()
			m.id = self.start_id + i
			m.header.frame_id = "/map"
			m.type = m.LINE_STRIP
			m.action = m.ADD

			m.scale.x = self.scale

			m.color.r = self.r
			m.color.g = self.g
			m.color.b = self.b
			m.color.a = self.a

			m.pose.orientation.x = 0
			m.pose.orientation.y = 0
			m.pose.orientation.z = 0
			m.pose.orientation.w = 1

			m.points = []
			for j in range(len(waypoint["x"])):
				if j % draw_step == 0 or j == len(waypoint["x"])-1:
					p = Point()
					p.x = waypoint["x"][j]
					p.y = waypoint["y"][j]
					m.points.append(p)                
			ma.markers.append(m)
		self.ma = ma
		


if __name__ == "__main__":
	rospy.init_node("map_rviz_visualizer")
	rospack = rospkg.RosPack()
	path = rospack.get_path("map_server")

	# TODO: AS CONFIGURATION FILE
	link_file = path + "/src/kcity/route.pkl"
	global_file = path + "/src/kcity/global.pkl"
	parking1_v3_file = path + "/src/kcity/parking1_v3.pkl"
	parking2_v3_file = path + "/src/kcity/parking2_v3.pkl"
	parking3_v3_file = path + "/src/kcity/parking3_v3.pkl"
	parking4_v3_file = path + "/src/kcity/parking4_v3.pkl"
	parking5_v3_file = path + "/src/kcity/parking5_v3.pkl"
	parking6_v3_file = path + "/src/kcity/parking6_v3.pkl"
	parking1_v1_file = path + "/src/kcity/parking1.pkl"
	parking2_v1_file = path + "/src/kcity/parking2.pkl"
	parking3_v1_file = path + "/src/kcity/parking3.pkl"
	parking4_v1_file = path + "/src/kcity/parking4.pkl"
	parking5_v1_file = path + "/src/kcity/parking5.pkl"
	# parking6_file = path + "/src/kcity/parking6_v2.pkl"

	link_cv = Converter(link_file, 2000, r=255/255.0, g=236/255.0, b=139/255.0, a=0.8, scale=0.5)
	global_cv=Converter(global_file, 2000, r=255/255.0, g=0/255.0, b=139/255.0, a=0.8, scale=0.5)
	# parking1_v1_cv = Converter(parking1_v1_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	# parking2_v1_cv = Converter(parking2_v1_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	# parking3_v1_cv = Converter(parking3_v1_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	# parking4_v1_cv = Converter(parking4_v1_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	# parking5_v1_cv = Converter(parking5_v1_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	parking1_v3_cv = Converter(parking1_v3_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	parking2_v3_cv = Converter(parking2_v3_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	parking3_v3_cv = Converter(parking3_v3_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	parking4_v3_cv = Converter(parking4_v3_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	parking5_v3_cv = Converter(parking5_v3_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)
	parking6_v3_cv = Converter(parking6_v3_file, 3000, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=1.2, scale=1.2)

	link_pub = rospy.Publisher("/rviz/lane_links", MarkerArray, queue_size=1.2, latch=True)
	global_pub = rospy.Publisher("/rviz/global_links", MarkerArray, queue_size=1.2, latch=True)
	# parking_link1_v1_pub = rospy.Publisher("/rviz/parking_link_1_v1", MarkerArray, queue_size=1, latch=True)
	# parking_link2_v1_pub = rospy.Publisher("/rviz/parking_link_2_v1", MarkerArray, queue_size=1, latch=True)
	# parking_link3_v1_pub = rospy.Publisher("/rviz/parking_link_3_v1", MarkerArray, queue_size=1, latch=True)
	# parking_link4_v1_pub = rospy.Publisher("/rviz/parking_link_4_v1", MarkerArray, queue_size=1, latch=True)
	# parking_link5_v1_pub = rospy.Publisher("/rviz/parking_link_5_v1", MarkerArray, queue_size=1, latch=True)
	parking_link1_v3_pub = rospy.Publisher("/rviz/parking_link_1", MarkerArray, queue_size=1, latch=True)
	parking_link2_v3_pub = rospy.Publisher("/rviz/parking_link_2", MarkerArray, queue_size=1, latch=True)
	parking_link3_v3_pub = rospy.Publisher("/rviz/parking_link_3", MarkerArray, queue_size=1, latch=True)
	parking_link4_v3_pub = rospy.Publisher("/rviz/parking_link_4", MarkerArray, queue_size=1, latch=True)
	parking_link5_v3_pub = rospy.Publisher("/rviz/parking_link_5", MarkerArray, queue_size=1, latch=True)
	parking_link6_v3_pub = rospy.Publisher("/rviz/parking_link_6", MarkerArray, queue_size=1, latch=True)

	rospy.sleep(1)
	while not rospy.is_shutdown():
		link_pub.publish(link_cv.ma)
		global_pub.publish(global_cv.ma)
		# parking_link1_v1_pub.publish(parking1_v1_cv.ma)
		# parking_link2_v1_pub.publish(parking2_v1_cv.ma)
		# parking_link3_v1_pub.publish(parking3_v1_cv.ma)
		# parking_link4_v1_pub.publish(parking4_v1_cv.ma)
		# parking_link5_v1_pub.publish(parking5_v1_cv.ma)
		parking_link1_v3_pub.publish(parking1_v3_cv.ma)
		parking_link2_v3_pub.publish(parking2_v3_cv.ma)
		parking_link3_v3_pub.publish(parking3_v3_cv.ma)
		parking_link4_v3_pub.publish(parking4_v3_cv.ma)
		parking_link5_v3_pub.publish(parking5_v3_cv.ma)
		parking_link6_v3_pub.publish(parking6_v3_cv.ma)
		rospy.sleep(1)
