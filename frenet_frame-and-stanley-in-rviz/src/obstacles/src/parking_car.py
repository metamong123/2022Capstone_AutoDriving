#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy, tf
import numpy as np

from object_msgs.msg import Object, ObjectArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

class ObstaclePub():
	def __init__(self, width_thresh, len_thresh):
		self.world_frame = "map"
		self.detection_frame = "car_1"
		self.width_thresh = width_thresh
		self.len_thresh = len_thresh
		self.mode = 'global'
		self.msg = MarkerArray()

		self.pub = rospy.Publisher('/obstacles', ObjectArray, queue_size=1)
		self.sub_cluster = rospy.Subscriber('/adaptive_clustering_v1/markers', MarkerArray, self.cluster_callback)
		self.sub_mode = rospy.Subscriber("/mode_selector", String, self.mode_callback)
		self.listener = tf.TransformListener()

	
	def change_frame(self, x, y, yaw, world_frame, detection_frame):
		pose = tf.transformations.euler_matrix(0, 0, yaw)
		pose[:2, 3] = np.matrix([x, y])

		now = rospy.Time()
		self.listener.waitForTransform(world_frame, detection_frame, now, rospy.Duration(1.0))
		t, r = self.listener.lookupTransform(world_frame, detection_frame, now)
		
		tf_matrix = tf.transformations.quaternion_matrix(r)
		tf_matrix[:3, 3] = t[:3]

		result = np.array(np.dot(tf_matrix, pose))
		euler = tf.transformations.euler_from_matrix(result)
		
		return float(result[0, 3]), float(result[1, 3]), euler[2]


	def get_object(self, id, obs):
		o = Object()

		o.header.frame_id = "map"
		o.id = id
		o.classification = o.CLASSIFICATION_CAR

		# object length, width
		o.L = abs(obs.points[0].x - obs.points[1].x)
		o.W = abs(obs.points[4].y - obs.points[3].y)

		# object x, y, yaw
		x = (obs.points[0].x + obs.points[1].x)/2
		y = (obs.points[4].y + obs.points[3].y)/2
		yaw = 0

		# transformation (car_1 -> map)
		o.x, o.y, o.yaw = self.change_frame(x, y, yaw, self.world_frame, self.detection_frame)

		o.header.stamp = rospy.Time.now()
		return o


	def msg_pub(self):
		markers = self.msg
		msg = ObjectArray()

		if markers.markers:
			width_thresh = self.width_thresh
			if self.mode == 'dynamic_object':
				for id, obs in enumerate(markers.markers):
					# points[0]: upper/back/left point of box
					# points[1]: upper/front/left point of box
					# points[3]: upper/back/right point of box
					if (((-width_thresh < obs.points[3].y < width_thresh) or (-width_thresh < obs.points[1].y < width_thresh)) and ((obs.points[1].x < self.len_thresh))):
						o = self.get_object(id, obs)
						msg.object_list.append(o)
			else:
				for id, obs in enumerate(markers.markers):
					o = self.get_object(id, obs)
					msg.object_list.append(o)

		self.pub.publish(msg)


	def cluster_callback(self, data):
		self.msg = data

	
	def mode_callback(self, data):
		self.mode = data.data


if __name__ == "__main__":
	#create node

	WIDTH_THRESH = 1.5
	LEN_THRESH = 5

	rospy.init_node('parking_car')
	r = rospy.Rate(10)
	node = ObstaclePub(WIDTH_THRESH, LEN_THRESH)

	while not rospy.is_shutdown():
		node.msg_pub()
		r.sleep()
