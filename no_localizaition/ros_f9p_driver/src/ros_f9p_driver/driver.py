#!/usr/bin/env python

import rospy
import serial
import time
import math

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseStamped
#from pyproj import Proj, transform
from nav_msgs.msg import Odometry


gpsRvizPub=rospy.Publisher('/odom',Odometry,queue_size=1)

def callback(msg):

	#if msg.latitude !=float("inf"):
	#	kcity=Proj(init='epsg:5179')
	#	wgs84=Proj(init='epsg:4326')
	#	a,b=transform(wgs84,kcity,msg.longitude,msg.latitude)

	gpsMsg=NavSatFix()
	gpsMsg.header=msg.header
	gpsMsg.status=msg.status

	#gpsMsg.latitude=a
	#gpsMsg.longitude=b
	gpsMsg.altitude=msg.altitude

	if msg.position_covariance:
		gpsMsg.position_covariance=msg.position_covariance
		#gpsMsg.position_covariance_type=msg.posotion_covariance_type
	
	navSatPub=rospy.Publisher('~fix',NavSatFix,queue_size=1)
	navSatPub.publish(gpsMsg)
	
	
        rpose.pose.pose.position.x=msg.latitude# rpose.pose.pose.position.x=a-962000
        rpose.pose.pose.position.y=msg.longitude# rpose.pose.pose.position.y=b-1959000
	rpose.pose.pose.position.z=0
	
	rpose.pose.covariance[0]=msg.position_covariance[0]
	rpose.pose.covariance[7]=msg.position_covariance[4]
	rpose.pose.covariance[14]=msg.position_covariance[8]
	rpose.pose.covariance[21]=99999
	rpose.pose.covariance[28]=99999
	rpose.pose.covariance[35]=99999

	
	

def callback1(msg):
	qx=msg.orientation.x
	qy=msg.orientation.y
	qz=msg.orientation.z
	qw=msg.orientation.w
	
	rpose.pose.pose.orientation.x=qx	
	rpose.pose.pose.orientation.y=qy	
	rpose.pose.pose.orientation.z=qz	
	rpose.pose.pose.orientation.w=qw	
	
	rpose.twist.twist.linear.x=msg.linear_acceleration.x
	rpose.twist.twist.linear.y=msg.linear_acceleration.y
	rpose.twist.twist.linear.z=msg.linear_acceleration.z

	rpose.twist.twist.angular.x=msg.angular_velocity.x
	rpose.twist.twist.angular.y=msg.angular_velocity.y
	rpose.twist.twist.angular.z=msg.angular_velocity.z
	
	rpose.twist.covariance[0]=0.01
	rpose.twist.covariance[7]=0.01
	rpose.twist.covariance[14]=0.01
	rpose.twist.covariance[21]=0.01
	rpose.twist.covariance[28]=0.01
	rpose.twist.covariance[35]=0.01

	gpsRvizPub.publish(rpose)

if __name__=='__main__':
	
	rospy.init_node('gps_to_xy')

	global rpose
	rpose=Odometry()
	rpose.header.stamp=rospy.Time.now()
	rpose.header.frame_id="base_footprint"

	rospy.Subscriber("/gps/fix",NavSatFix,callback)
	rospy.Subscriber("/imu_data",Imu,callback1)
	
	
	
	
        
	rospy.spin()
