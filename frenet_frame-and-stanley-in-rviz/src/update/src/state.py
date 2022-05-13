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

def odometry_callback(data): 
	global x, y, v, yaw
	# sensor_msgs/Imu.msg 
	x = data.pose.pose.position.x 
	y = data.pose.pose.position.y 

	vx = data.twist.twist.linear.x
	vy = data.twist.twist.linear.y
	v = np.sqrt(vx**2+vy**2)

	orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] 
	roll, pitch, yaw = euler_from_quaternion (orientation_list) 
	# imu_theta=yaw*(180/np.pi) 
    	# print("lap: %f" %cur_gps_position[0])

def speed_callback(data): 
	global speed 
	speed = data.data/36 

if __name__ == "__main__":
    
	rospy.init_node("state")
	#state_pub=rospy.Publisher("/state",Object,queue_size=1)
	object_pub=rospy.Publisher("/objects/car_1", Object,queue_size=1)
	r = rospy.Rate(10)

	rospy.Subscriber("/odom", Odometry,odometry_callback)
	# rospy.Subscriber("/ERP42_speed",Float32,speed_callback)
	# imu_theta=0.0
	#cur_gps_position=[126.76780661311217,37.22919729043270]
	x=0.0
	y=0.0
	yaw=0.0
	v=0.0
	id=1
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
        
		quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
		tf_broadcaster.sendTransform((x, y, 1.5),quat,rospy.Time.now(),"/car_" + str(id), "/map")
		
		m = Marker()
		m.header.frame_id = "/map"
		m.header.stamp = rospy.Time.now()
		m.id = id
		m.type = m.CUBE

		m.pose.position.x = x + 1.3 * math.cos(yaw)
		m.pose.position.y = y + 1.3 * math.sin(yaw)
		m.pose.position.z = 0.75
		m.pose.orientation = Quaternion(*quat)

		m.scale.x = 4.475
		m.scale.y = 1.850
		m.scale.z = 1.645

		m.color.r = 93 / 255.0
		m.color.g = 122 / 255.0
		m.color.b = 177 / 255.0
		m.color.a = 0.97

		o = Object()
		o.header.frame_id = "/map"
		o.header.stamp = rospy.Time.now()
		o.id = 1
		o.classification = o.CLASSIFICATION_CAR
		o.x = x
		o.y = y
		o.yaw = yaw
		#o.yaw=1.28713
		#o.v = 1
		o.L = 1.600
		o.W = 1.160
		#o.WB = 1.06
		object_pub.publish(o)		
		r.sleep()
