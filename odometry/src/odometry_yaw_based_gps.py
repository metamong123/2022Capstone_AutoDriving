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

vo_Pub=rospy.Publisher('/odom_imu',Odometry,queue_size=1)


heading_array = []
filtered_heading = 0

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z


def callback(msg):

	global cov1,cov2

	rpose.header.stamp=rospy.Time.now()
	if msg.latitude !=float("inf"):
		kcity=Proj(init='epsg:5179')
		wgs84=Proj(init='epsg:4326')
		a,b=transform(wgs84,kcity,msg.longitude,msg.latitude)


	rpose.pose.pose.position.x=a #-962614+27.7457989061-0.28
	rpose.pose.pose.position.y=b #-1959199-56.3029978438-4.13
	rpose.pose.covariance[0]=msg.position_covariance[0]
	rpose.pose.covariance[7]=msg.position_covariance[4]
	rpose.pose.covariance[14]=msg.position_covariance[8]
		

def callback1(msg):
	
	global i, before_qz, before_qw
	heading=np.arctan( msg.twist.twist.linear.y / msg.twist.twist.linear.x )
	
	if msg.twist.twist.linear.x < 0 :
		heading=heading+np.pi
	
	#heading=math.atan2(msg.twist.twist.linear.y , msg.twist.twist.linear.x)
	heading_array.insert(0,heading)   # heading value save
	qx=0
	qy=0
	if len(heading_array) == 3:   # save number
		heading_array.pop()
	#filtered_heading=heading
	filtered_heading = (sum(heading_array)/len(heading_array))   # moving average
	# filtered_heading = np.median(heading_array)  #moving median
 
	if i==0:
		before_qz = 0
		before_qw = 1
	if (abs(msg.twist.twist.linear.x) or abs(msg.twist.twist.linear.y)) > 0.1:
		qz=np.sin(filtered_heading / 2)
		qw=np.cos(filtered_heading / 2)
		rpose.pose.pose.orientation.x=qx	
		rpose.pose.pose.orientation.y=qy	
		rpose.pose.pose.orientation.z=qz	
		rpose.pose.pose.orientation.w=qw
		before_qz = qz			
		before_qw = qw

		yaw = euler_from_quaternion(qx, qy, qz, qw)

	else :
		rpose.pose.pose.orientation.x=qx
		rpose.pose.pose.orientation.y=qy
		rpose.pose.pose.orientation.z=before_qz	
		rpose.pose.pose.orientation.w=before_qw

		yaw = euler_from_quaternion(qx, qy, before_qz, before_qw)
	
	#print(yaw)

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


if __name__=='__main__':
	
	rospy.init_node('odometry_gps')

	global rpose, a, b, heading
	a=0
	b=0
	heading=0
	i=0
	rpose=Odometry()
	rpose.header.frame_id="odom"

	rospy.Subscriber("/gps/fix",NavSatFix,callback)
	rospy.Subscriber("/ublox_gps/fix_velocity",TwistWithCovarianceStamped,callback1)

        
	rospy.spin()