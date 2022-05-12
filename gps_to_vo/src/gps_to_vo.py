#!/usr/bin/env python

import rospy
import serial
import time
import math
import pickle

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from pyproj import Proj, transform
from nav_msgs.msg import Odometry

import numpy as np

vo_Pub=rospy.Publisher('/odom',Odometry,queue_size=1)

now_gps_x=0
now_gps_y=0

def callback(msg):

	global cov1,cov2

	rpose.header.stamp=rospy.Time.now()
	if msg.latitude !=float("inf"):
		kcity=Proj(init='epsg:5179')
		wgs84=Proj(init='epsg:4326')
		a,b=transform(wgs84,kcity,msg.longitude,msg.latitude)

########### cov check ################
	cov1 = gps.position_covariance[0] 
	cov2 = gps.position_covariance[4]
######################################
	if (cov1 <= 0.0003 or cov2 <= 0.0003 ) :
		rpose.pose.pose.position.x=a #-962614+27.7457989061-0.28
		rpose.pose.pose.position.y=b #-1959199-56.3029978438-4.13
		rpose.pose.covariance[0]=msg.position_covariance[0]
		rpose.pose.covariance[7]=msg.position_covariance[4]
		rpose.pose.covariance[14]=msg.position_covariance[8]
		
	else:
		gps_jumping()


def callback1(msg):
	
	global i, before_qz, before_qw
	heading=np.arctan( msg.twist.twist.linear.y / msg.twist.twist.linear.x )
	if msg.twist.twist.linear.x < 0 :
		heading=heading+np.pi
	qx=0
	qy=0
	if i==0:
		before_qz = 0
		before_qw = 1
	if (abs(msg.twist.twist.linear.x) or abs(msg.twist.twist.linear.y)) > 0.1:
		qz=np.sin(heading / 2)
		qw=np.cos(heading / 2)
		rpose.pose.pose.orientation.x=qx	
		rpose.pose.pose.orientation.y=qy	
		rpose.pose.pose.orientation.z=qz	
		rpose.pose.pose.orientation.w=qw
		before_qz = qz
		before_qw = qw
	else :
		rpose.pose.pose.orientation.x=qx	
		rpose.pose.pose.orientation.y=qy	
		rpose.pose.pose.orientation.z=before_qz	
		rpose.pose.pose.orientation.w=before_qw
  
	rpose.twist.twist.linear.x = msg.twist.twist.linear.x
	rpose.twist.twist.linear.y = msg.twist.twist.linear.y
 	rpose.twist.twist.linear.z = msg.twist.twist.linear.z
	rpose.pose.covariance[21]=99999
	rpose.pose.covariance[28]=99999
	rpose.pose.covariance[35]=msg.twist.covariance[0]
	#rate=rospy.Rate(1)
	#rate.sleep()
	vo_Pub.publish(rpose)
	
	i=i+1
######################## gps covariance increased, jumping#########################
def gps_jumping():
	if 	





#####################################################################################
if __name__=='__main__':
	
	rospy.init_node('gps_to_vo')

	global rpose, a, b, heading, now_gps_x, now_gps_y
	a=0
	b=0
	heading=0
	i=0
	rpose=Odometry()
	rpose.header.frame_id="odom"

	rospy.Subscriber("/gps/fix",NavSatFix,callback)
	rospy.Subscriber("/ublox_gps/fix_velocity",TwistWithCovarianceStamped,callback1)
	
	
	with open(path_map + "/src/route.pkl", "rb") as f:
		nodes = pickle.load(f)
	nodes[0]={}
	nodes[0]={'x':nodes[0]['x'][:], 'y':nodes[0]['y'][:]}
	
        
	rospy.spin()
