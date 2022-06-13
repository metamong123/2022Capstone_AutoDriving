#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
import numpy as np

from object_msgs.msg import Object, ObjectArray
from visualization_msgs.msg import Marker, MarkerArray

class ObstaclePub():
	def __init__(self):
		self.world_frame = "map"
		self.detection_frame = "car_1"
		self.msg = MarkerArray()
		self.pub = rospy.Publisher('obstacles', ObjectArray, queue_size=10)
		self.sub_cluster = rospy.Subscriber('/adaptive_clustering_v1/markers', MarkerArray, self.cluster_callback)
		self.listener = tf.TransformListener()

		#self.tf_broadcaster = tf.TransformBroadcaster()
		#loop_rate = rospy.Rate(10) #10Hz

	#callback function def
	def msg_pub(self):
		markers = self.msg
		msg = ObjectArray()

		if markers:
			for id in range(len(markers.markers)):

				o = Object()

				o.L = abs(markers.markers[id].points[0].x - markers.markers[id].points[1].x)
				o.W = abs(markers.markers[id].points[4].y - markers.markers[id].points[3].y)

				o.header.frame_id = "map"
				o.id = id
				o.classification = o.CLASSIFICATION_CAR

				x = (markers.markers[id].points[0].x + markers.markers[id].points[1].x)/2
				y = (markers.markers[id].points[4].y + markers.markers[id].points[3].y)/2
				yaw = 0

				pose = tf.transformations.euler_matrix(0, 0, yaw)
				pose[0, 3] = x
				pose[1, 3] = y
				pose[2, 3] = 0

				self.listener.waitForTransform(self.world_frame,self.detection_frame, rospy.Time(),rospy.Duration(10))
				t, r = self.listener.lookupTransform(self.world_frame,self.detection_frame, rospy.Time(0))

				tf_matrix = np.matrix(tf.transformations.quaternion_matrix(r))
				tf_matrix[0, 3] = t[0]
				tf_matrix[1, 3] = t[1]
				tf_matrix[2, 3] = t[2]
        
				result = np.array(np.dot(tf_matrix, pose))

				o.x = result[0, 3]
				o.y = result[1, 3]

				euler = tf.transformations.euler_from_matrix(result)
				o.yaw = euler[2]

				o.header.stamp = rospy.Time.now()
				msg.object_list.append(o)

		self.pub.publish(msg)
	
	def cluster_callback(self, data):
		self.msg = data


if __name__ == "__main__":
	#create node
	rospy.init_node('parking_car')
	r = rospy.Rate(10)
	node = ObstaclePub()

	while not rospy.is_shutdown():
		node.msg_pub()
		r.sleep()