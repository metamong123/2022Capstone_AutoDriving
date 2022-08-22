#! /usr/bin/python
#-*- coding: utf-8 -*-

import time
import rospy
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

class TFPub:
	def __init__(self):
		# [x, y, quat(x, y, z, w)]
		self.odom_dict = {'gps':(), 'imu':()}
		self.gps_marker_array = MarkerArray()
		self.imu_marker_array = MarkerArray()
		self.id = 0
		self.t1 = time.time()

		self.odom_gps_sub=rospy.Subscriber("/odom_gps", Odometry,self.odometry_gps_callback)
		self.odom_imu_sub=rospy.Subscriber("/odom_imu", Odometry,self.odometry_imu_callback)
		self.gps_tf = tf.TransformBroadcaster()
		self.imu_tf = tf.TransformBroadcaster()
		self.gps_marker_publisher = rospy.Publisher("/object_markers/gps", MarkerArray, queue_size=1)
		self.imu_marker_publisher = rospy.Publisher("/object_markers/imu", MarkerArray, queue_size=1)
		

	def odometry_gps_callback(self, data):
		self.odom_dict['gps'] = (data.pose.pose.position.x, data.pose.pose.position.y, (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))


	def odometry_imu_callback(self, data):
		self.odom_dict['imu'] = (data.pose.pose.position.x, data.pose.pose.position.y, (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))


	def tf_broad(self):
		if self.odom_dict['gps'] and self.odom_dict['imu']:
			self.gps_tf.sendTransform((self.odom_dict['gps'][0], self.odom_dict['gps'][1], 1.5), self.odom_dict['gps'][2], rospy.Time.now(), "/car_1/gps", "/map")
			self.imu_tf.sendTransform((self.odom_dict['imu'][0], self.odom_dict['imu'][1], 1.5), self.odom_dict['imu'][2], rospy.Time.now(), "/car_1/imu", "/map")


	def marker_pub(self):
		if self.odom_dict['gps'] and self.odom_dict['imu']:
			m1 = Marker()
			m1.header.frame_id = "/map"
			m1.header.stamp = rospy.Time.now()
			m1.type = Marker.ARROW
			m1.id = self.id
			m1.pose.position.x = self.odom_dict['gps'][0]
			m1.pose.position.y = self.odom_dict['gps'][1]
			m1.pose.position.z = 0.1
			m1.pose.orientation.x = self.odom_dict['gps'][2][0]
			m1.pose.orientation.y = self.odom_dict['gps'][2][1]
			m1.pose.orientation.z = self.odom_dict['gps'][2][2]
			m1.pose.orientation.w = self.odom_dict['gps'][2][3]
			m1.scale.x = 1
			m1.scale.y = 0.1
			m1.scale.z = 0.1

			m1.color.a = 1.0
			m1.color.r = 1.0
			m1.color.g = 0.0
			m1.color.b = 0.0

			self.gps_marker_array.markers.append(m1)

			m2 = Marker()
			m2.header.frame_id = "/map"
			m2.header.stamp = rospy.Time.now()
			m2.type = Marker.ARROW
			m2.id = self.id
			m2.pose.position.x = self.odom_dict['imu'][0]
			m2.pose.position.y = self.odom_dict['imu'][1]
			m2.pose.position.z = 0.1
			m2.pose.orientation.x = self.odom_dict['imu'][2][0]
			m2.pose.orientation.y = self.odom_dict['imu'][2][1]
			m2.pose.orientation.z = self.odom_dict['imu'][2][2]
			m2.pose.orientation.w = self.odom_dict['imu'][2][3]
			m2.scale.x = 1
			m2.scale.y = 0.1
			m2.scale.z = 0.1

			m2.color.a = 1.0
			m2.color.r = 0.0
			m2.color.g = 0.8
			m2.color.b = 0.0

			self.imu_marker_array.markers.append(m2)

			self.id += 1
			self.gps_marker_publisher.publish(self.gps_marker_array)
			self.imu_marker_publisher.publish(self.imu_marker_array)
			self.t1 = time.time()

if __name__ == "__main__":
    
	rospy.init_node("tf_pub")
	node = TFPub()
	r = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		node.tf_broad()
		if time.time() - node.t1 > 0.5:
			node.marker_pub()
		r.sleep()