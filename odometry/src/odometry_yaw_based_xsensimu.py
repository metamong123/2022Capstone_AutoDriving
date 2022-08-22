#!/usr/bin/env python

import rospy
import serial
import time
import math
import pickle

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped, QuaternionStamped
from pyproj import Proj, transform
from nav_msgs.msg import Odometry

import numpy as np

vo_Pub=rospy.Publisher('/odom_imu',Odometry,queue_size=1)


def callback(msg):

	global cov1,cov2

	rpose.header.stamp=rospy.Time.now()
	if msg.latitude !=float("inf"):
		kcity=Proj(init='epsg:5179')
		wgs84=Proj(init='epsg:4326')
		a,b=transform(wgs84,kcity,msg.longitude,msg.latitude)


	rpose.pose.pose.position.x=a
	rpose.pose.pose.position.y=b
	rpose.pose.covariance[0]=msg.position_covariance[0]
	rpose.pose.covariance[7]=msg.position_covariance[4]
	rpose.pose.covariance[14]=msg.position_covariance[8]
		
def imu_callback(msg):
	#print(msg.orientation.x)
	rpose.pose.pose.orientation.x=msg.orientation.x
	rpose.pose.pose.orientation.y=msg.orientation.y
	rpose.pose.pose.orientation.z=msg.orientation.z
	rpose.pose.pose.orientation.w=msg.orientation.w
	vo_Pub.publish(rpose)

if __name__=='__main__':
	
	rospy.init_node('odometry')

	global rpose, a, b, heading
	a=0
	b=0
	heading=0
	i=0
	rpose=Odometry()
	rpose.header.frame_id="odom"

	rospy.Subscriber("/gps/fix",NavSatFix,callback)
	rospy.Subscriber("/imu/data",Imu, imu_callback)
	
	rospy.spin()