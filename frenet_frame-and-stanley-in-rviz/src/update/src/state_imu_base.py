#! /usr/bin/python
#-*- coding: utf-8 -*-

import time
import math
import rospy
#from shapely.geometry import LineString, Point, Polygon
#import geopandas as gpd
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt32, Bool

x_gps=0.0
y_gps=0.0
yaw_gps=0.0
v_gps=0.0

x_imu=0.0
y_imu=0.0
yaw_imu=0.0
v_imu=0.0

id=1

class TopicReciver:
	def __init__(self):
		self.odom_gps_sub=rospy.Subscriber("/odom_gps", Odometry,self.odometry_gps_callback)
		self.odom_imu_sub=rospy.Subscriber("/odom_imu", Odometry,self.odometry_imu_callback)
	def check_all_connections(self):
		return (self.odom_gps_sub.get_num_connections()+self.odom_imu_sub.get_num_connections())==2
	def odometry_gps_callback(self, data):
		if self.check_all_connections():
			global x_gps, y_gps, v_gps, yaw_gps
			# sensor_msgs/Imu.msg 
			x_gps = data.pose.pose.position.x
			y_gps = data.pose.pose.position.y

			v_gps = data.twist.twist.linear.x
			# vy = data.twist.twist.linear.y
			# vz = data.twist.twist.linear.z
			# v = np.sqrt(vx**2+vy**2+vz**2)

			orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
			roll, pitch, yaw_gps = euler_from_quaternion (orientation_list) 
			# imu_theta=yaw*(180/np.pi) 
			# print("lap: %f" %cur_gps_position[0])

	def odometry_imu_callback(self, data):
		if self.check_all_connections():
			global x_imu, y_imu, v_imu, yaw_imu
			# sensor_msgs/Imu.msg 
			x_imu = data.pose.pose.position.x 
			y_imu = data.pose.pose.position.y 

			v_imu = data.twist.twist.linear.x
			# vy = data.twist.twist.linear.y
			# vz = data.twist.twist.linear.z
			# v = np.sqrt(vx**2+vy**2+vz**2)

			orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
			roll, pitch, yaw_imu = euler_from_quaternion (orientation_list) 
			# imu_theta=yaw*(180/np.pi) 
			# print("lap: %f" %cur_gps_position[0])


# def speed_callback(data): 
# 	global speed 
# 	speed = data.data/36 

if __name__ == "__main__":
    
	rospy.init_node("state")
	#state_pub=rospy.Publisher("/state",Object,queue_size=1)
	topic_receiver=TopicReciver()
	object_gps_pub=rospy.Publisher("/objects/car_1/gps", Object,queue_size=1)
	object_imu_pub=rospy.Publisher("/objects/car_1/imu", Object,queue_size=1)
	marker_pub=rospy.Publisher("/objects/marker/car_1", Marker,queue_size=1)
	
	r = rospy.Rate(10)

	# rospy.Subscriber("/odom", Odometry,odometry_callback)
	# rospy.Subscriber("/ERP42_speed",Float32,speed_callback)
	# imu_theta=0.0
	#cur_gps_position=[126.76780661311217,37.22919729043270]
	
	# speed=0
	while not rospy.is_shutdown():

		# position=[0,0]
		# position[0] = cur_gps_position[0]  # 위도
		# position[1] = cur_gps_position[1]  # 경도

		#gps = [Point((going_gps[0], going_gps[1]))]
		#gps_d = gpd.GeoDataFrame(geometry=gps, crs={'init': 'epsg:4326'})
		#gps_d.to_crs(epsg=5179, inplace=True)
		#x, y = gps_d['geometry'][0].xy
		#x=x[0]
		#y=y[0]
		
		tf_broadcaster = tf.TransformBroadcaster()
        
		quat_imu = tf.transformations.quaternion_from_euler(0, 0, yaw_imu)
		tf_broadcaster.sendTransform((x_imu, y_imu, 1.5),quat_imu,rospy.Time.now(),"/car_1", "/map")
		
		m = Marker()
		m.header.frame_id = "/map"
		m.header.stamp = rospy.Time.now()
		m.id = id
		m.type = m.CUBE

		#m.pose.position.x = x + 1.3 * math.cos(yaw)
		#m.pose.position.y = y + 1.3 * math.sin(yaw)

		m.pose.position.x = x_imu
		m.pose.position.y = y_imu
		m.pose.position.z = 0.3
		m.pose.orientation = Quaternion(*quat_imu)

		m.scale.x = 1.600
		m.scale.y = 1.160
		m.scale.z = 1.645

		m.color.r = 93 / 255.0
		m.color.g = 122 / 255.0
		m.color.b = 177 / 255.0
		m.color.a = 0.97

		marker_pub.publish(m)

		o_gps = Object()
		o_gps.header.frame_id = "/map"
		o_gps.header.stamp = rospy.Time.now()
		o_gps.id = 1
		o_gps.classification = o_gps.CLASSIFICATION_CAR
		o_gps.x = x_gps
		o_gps.y = y_gps
		o_gps.yaw = yaw_gps
		#o.yaw=1.28713
		########
		o_gps.v = v_gps
		#######
		o_gps.L = 1.600
		o_gps.W = 1.160
		#o.WB = 1.06
		object_gps_pub.publish(o_gps)		

		o_imu = Object()
		o_imu.header.frame_id = "/map"
		o_imu.header.stamp = rospy.Time.now()
		o_imu.id = 1
		o_imu.classification = o_imu.CLASSIFICATION_CAR
		o_imu.x = x_imu
		o_imu.y = y_imu
		o_imu.yaw = yaw_imu
		#o.yaw=1.28713
		########
		o_imu.v = v_imu
		#######
		o_imu.L = 1.600
		o_imu.W = 1.160
		#o.WB = 1.06
		object_imu_pub.publish(o_imu)

		r.sleep()