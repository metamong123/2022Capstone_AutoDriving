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
def euler_to_quaternion(roll,pitch,yaw):
	
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

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
     
        return yaw_z # in radians
    

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
		



def callback2(msg):
    
    qx=msg.orientation.x	
    qy=msg.orientation.y		
    qz=msg.orientation.z
    qw=msg.orientation.w
    yaw_z = euler_from_quaternion(qx,qy,qz,qw)
    yaw_z=yaw_z + np.pi
    [qx,qy,qz,qw]=euler_to_quaternion(0,0,yaw_z)
    rpose.pose.pose.orientation.x=qx
    rpose.pose.pose.orientation.y=qy
    rpose.pose.pose.orientation.z=qz
    rpose.pose.pose.orientation.w=qw
    
    vo_Pub.publish(rpose)

if __name__=='__main__':
	
	rospy.init_node('gps_to_vo')

	global rpose, a, b, heading
	a=0
	b=0
	heading=0
	i=0
	rpose=Odometry()
	rpose.header.frame_id="odom"

	rospy.Subscriber("/gps/fix",NavSatFix,callback)
	rospy.Subscriber("imu/data",Imu, callback2)
        
	rospy.spin()