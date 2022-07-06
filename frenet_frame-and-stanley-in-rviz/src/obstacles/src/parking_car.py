#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import math
import numpy as np
import pickle
import tf
from object_msgs.msg import Object
from geometry_msgs.msg import Quaternion


class CarParked(object):
	def __init__(self, x=0.0, y=0.0, yaw=0.0, id=1):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = 0.0
		self.accel = 0.0
		self.delta = 0.0

		self.tf_broadcaster = tf.TransformBroadcaster()
		self.id = id
		self.object_pub = rospy.Publisher("/objects/car_" + str(id), Object, queue_size=1)
		while self.object_pub.get_num_connections() == 0:
			continue

	def to_ros(self):
		quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
		self.tf_broadcaster.sendTransform(
			(self.x, self.y, 1.5),
			quat,
			rospy.Time.now(),
			"/car_" + str(self.id), "/map"
		)

		o = Object()
		o.header.frame_id = "/map"
		o.header.stamp = rospy.Time.now()
		o.id = self.id
		o.classification = o.CLASSIFICATION_CAR

		o.x = self.x
		o.y = self.y
		o.yaw = self.yaw
		o.v = self.v

		# input u
		o.a = self.accel
		o.delta = self.delta
		o.L = 4.475
		o.W = 1.150

		self.object_pub.publish(o)


if __name__ == "__main__":
	rospy.init_node("parking_car")
	r = rospy.Rate(1)
	cp1 = CarParked(x=962575.68525061, y=1959246.37897911, yaw=2.84853436, id=2)
	cp2 = CarParked(x=962577.02808145, y=1959251.08300435, yaw=2.84853436, id=3)

	while not rospy.is_shutdown():
		cp1.to_ros()
		cp2.to_ros()
		r.sleep()
