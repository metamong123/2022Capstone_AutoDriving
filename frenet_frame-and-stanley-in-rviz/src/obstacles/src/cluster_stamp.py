#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy

# message import
from visualization_msgs.msg import MarkerArray
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#node class
class ClusterTFPub():
	def __init__(self):
		#create pub&sub
		#pub = rospy.Publisher('topic_name', message_type, queue_size)
		self.pub = rospy.Publisher('adaptive_clustering_v1', MarkerArray, queue_size=1)
		#sub = rospy.Subscriber('topic_name', message_type, callback)
		#self.sub = rospy.Subscriber('adaptive_clustering_v1/markers', MarkerArray, self.get_msg)
		self.sub = rospy.Subscriber('adaptive_clustering_v1/markers', MarkerArray, self.get_msg)
		self.msg = MarkerArray()

    #callback function def
	def get_msg(self, data):
		self.msg = data
		'''
		self.msg = MarkerArray()
		
		idx = 0
		for i in range(len(data.markers)):
			if data.markers[i].points[0].x > 0 and (1 < abs(data.markers[i].points[4].y - data.markers[i].points[3].y) < 4):
				self.msg.markers.append(data.markers[i])
				self.msg.markers[idx].header.frame_id = "car_1"
				self.msg.markers[idx].header.stamp = rospy.Time.now()
				idx += 1'''
		

	def msg_pub(self):
		#self.pub.publish(self.msg)
		
		msg = self.msg
		if msg:
			'''
			idx = 0
			for i in range(len(self.msg.markers)):
				if self.msg.markers[i].points[0].x > 0 and (abs(self.msg.markers[i].points[4].y - self.msg.markers[i].points[3].y) < 5) and (abs(self.msg.markers[i].points[0].x - self.msg.markers[i].points[1].x) < 5):
					msg.markers.append(self.msg.markers[i])
					msg.markers[idx].header.frame_id = "/car_1"
					msg.markers[idx].header.stamp = rospy.Time.now()
					idx += 1
			self.pub.publish(msg)
			'''
			
			for i in range(len(msg.markers)):
				#msg.markers[i].header.frame_id = "/car_1"
				msg.markers[i].header.stamp = rospy.Time.now()
			self.pub.publish(msg)
			

if __name__ == '__main__':
	#create node
	#rospy.init_node('node_name')
	rospy.init_node('cluster_tf')
	r = rospy.Rate(10)
	node = ClusterTFPub()

	while not rospy.is_shutdown():
		node.msg_pub()
		r.sleep()