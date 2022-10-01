#!/usr/bin/env python

import rospy
import serial
import time
import math
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from pyproj import Proj, transform

import numpy as np

Odom_Pub = rospy.Publisher('/odom_imu', Odometry, queue_size=1)

def callback(gps, imu, vel):
	
	cov1 = gps.position_covariance[0] 
	cov2 = gps.position_covariance[4]
	cov3 = gps.position_covariance[8]  
	kcity=Proj(init='epsg:5179')
	wgs84=Proj(init='epsg:4326')
	a,b=transform(wgs84,kcity,gps.longitude,gps.latitude)

	gpose.pose.pose.position.x=a# +4.0486660078167915#-1.8177898578578606
	gpose.pose.pose.position.y=b# -1.687483238754794#+0.34575470979325473
	gpose.pose.covariance[0]=cov1
	gpose.pose.covariance[7]=cov2
	gpose.pose.covariance[14]=cov3
	gpose.pose.covariance[21]=cov3
	gpose.pose.covariance[28]=cov3
	gpose.pose.covariance[35]=cov3
	
	gpose.pose.pose.orientation.x= imu.quaternion.x
	gpose.pose.pose.orientation.y= imu.quaternion.y
	gpose.pose.pose.orientation.z= imu.quaternion.z
	gpose.pose.pose.orientation.w= imu.quaternion.w
	
	gpose.twist.twist.linear.x = vel.twist.twist.linear.x
	gpose.twist.twist.linear.y = vel.twist.twist.linear.y
	gpose.twist.twist.linear.z = vel.twist.twist.linear.z


	Odom_Pub.publish(gpose)
	r.sleep()
	
if __name__=='__main__':
	
	rospy.init_node('gps_imu_decision')
	gpose=Odometry()
	gpose.header.stamp=rospy.Time.now()
	gpose.header.frame_id="odom"
	r = rospy.Rate(10)
	
	gps_sub = message_filters.Subscriber("/gps/fix",NavSatFix)
	gps_vel_sub = message_filters.Subscriber("/ublox_gps/fix_velocity",TwistWithCovarianceStamped)
	imu_sub = message_filters.Subscriber("/filter/quaternion",QuaternionStamped)
	ts = message_filters.ApproximateTimeSynchronizer([gps_sub,imu_sub,gps_vel_sub], 1,10)
	ts.registerCallback(callback)

	rospy.spin()

		
	