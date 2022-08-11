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

	
	def change_frame(self, x, y, yaw, world_frame, detection_frame):
		pose = tf.transformations.euler_matrix(0, 0, yaw)
		pose[:2, 3] = np.matrix([x, y])

		self.listener.waitForTransform(world_frame, detection_frame, rospy.Time(),rospy.Duration(10))
		t, r = self.listener.lookupTransform(world_frame, detection_frame, rospy.Time(0))
		
		tf_matrix = np.matrix(tf.transformations.quaternion_matrix(r))
		tf_matrix[0, 3] = t[0]
		tf_matrix[1, 3] = t[1]
		tf_matrix[2, 3] = t[2]

		result = np.array(np.dot(tf_matrix, pose))
		euler = tf.transformations.euler_from_matrix(result)

		return result[0, 3], result[1, 3], euler[2]


	def msg_pub(self):
		markers = self.msg
		msg = ObjectArray()

		if markers:
			world_frame = self.world_frame
			detection_frame = self.detection_frame
			
			for id in range(len(markers.markers)):

				o = Object()

				o.header.frame_id = "map"
				o.id = id
				o.classification = o.CLASSIFICATION_CAR

				# object length, width
				o.L = abs(markers.markers[id].points[0].x - markers.markers[id].points[1].x)
				o.W = abs(markers.markers[id].points[4].y - markers.markers[id].points[3].y)

				# object x, y, yaw
				# center of bounding box
				x = (markers.markers[id].points[0].x + markers.markers[id].points[1].x)/2
				y = (markers.markers[id].points[4].y + markers.markers[id].points[3].y)/2
				yaw = 0

				# transformation (car_1 -> map)
				o.x, o.y, o.yaw = self.change_frame(x, y, yaw, world_frame, detection_frame)

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
