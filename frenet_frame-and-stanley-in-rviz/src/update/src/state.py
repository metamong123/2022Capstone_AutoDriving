#! /usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object
from nav_msgs.msg import Odometry

class ImuPub:
	def __init__(self):
		#self.object_gps_pub=rospy.Publisher("/objects/car_1/gps", Object,queue_size=1)
		self.object_imu_pub=rospy.Publisher("/objects/car_1", Object,queue_size=1)
		self.marker_pub=rospy.Publisher("/objects/marker/car_1", Marker,queue_size=1)
		self.odom_imu_sub=rospy.Subscriber("/odom_imu", Odometry,  self.odometry_imu_callback)
		self.tf_broadcaster = tf.TransformBroadcaster()

		#self.x_gps = 0
		#self.y_gps = 0
		#self.v_gps = 0
		#self.yaw_gps = 0

		self.x_imu = 0
		self.y_imu = 0
		self.v_imu = 0
		self.yaw_imu = 0

	
	def msg_pub(self):
		quat_imu = tf.transformations.quaternion_from_euler(0, 0, self.yaw_imu)
		self.tf_broadcaster.sendTransform((self.x_imu, self.y_imu, 1.5), quat_imu,rospy.Time.now(),"/car_1", "/map")
	
		#o_gps = Object()
		#o_gps.header.frame_id = "/map"
		#o_gps.header.stamp = rospy.Time.now()
		#o_gps.id = 1
		#o_gps.classification = o_gps.CLASSIFICATION_CAR
		#o_gps.x = self.x_gps
		#o_gps.y = self.y_gps
		#o_gps.yaw = self.yaw_gps
		##o.yaw=1.28713
		#########
		#o_gps.v = self.v_gps
		########
		#o_gps.L = 1.600
		#o_gps.W = 1.160
		##o.WB = 1.06
		#self.object_gps_pub.publish(o_gps)

		o_imu = Object()
		o_imu.header.frame_id = "/map"
		o_imu.header.stamp = rospy.Time.now()
		o_imu.id = 1
		o_imu.classification = o_imu.CLASSIFICATION_CAR
		o_imu.x = self.x_imu
		o_imu.y = self.y_imu
		o_imu.yaw = self.yaw_imu
		#o.yaw=1.28713
		########
		o_imu.v = self.v_imu
		#######
		o_imu.L = 1.600
		o_imu.W = 1.160
		#o.WB = 1.06
		self.object_imu_pub.publish(o_imu)


		m = Marker()
		m.header.frame_id = "/map"
		m.header.stamp = rospy.Time.now()
		m.id = 1
		m.type = m.CUBE

		#m.pose.position.x = x + 1.3 * math.cos(yaw)
		#m.pose.position.y = y + 1.3 * math.sin(yaw)

		m.pose.position.x = self.x_imu
		m.pose.position.y = self.y_imu
		m.pose.position.z = 0.3
		m.pose.orientation = Quaternion(*quat_imu)

		m.scale.x = 1.600
		m.scale.y = 1.160
		m.scale.z = 1.645

		m.color.r = 93 / 255.0
		m.color.g = 122 / 255.0
		m.color.b = 177 / 255.0
		m.color.a = 0.97

		self.marker_pub.publish(m)


	def odometry_gps_callback(self, data):
		# sensor_msgs/Imu.msg 
		self.x_gps = data.pose.pose.position.x
		self.y_gps = data.pose.pose.position.y
		self.v_gps = data.twist.twist.linear.x
		# vy = data.twist.twist.linear.y
		# vz = data.twist.twist.linear.z
		# v = np.sqrt(vx**2+vy**2+vz**2)

		orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
		roll, pitch, self.yaw_gps = euler_from_quaternion (orientation_list)


	def odometry_imu_callback(self, data):
		# sensor_msgs/Imu.msg 
		self.x_imu = data.pose.pose.position.x 
		self.y_imu = data.pose.pose.position.y 
		self.v_imu = data.twist.twist.linear.x

		orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
		roll, pitch, self.yaw_imu = euler_from_quaternion(orientation_list) 


if __name__ == "__main__":
    
	rospy.init_node("state")
	node = ImuPub()
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		node.msg_pub()
		r.sleep()