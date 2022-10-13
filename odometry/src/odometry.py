#!/usr/bin/env python
#-*- coding: utf-8 -*-

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

def normalize_angle(angle):
	while angle > np.pi:
		angle -= 2.0 * np.pi
	while angle < -np.pi:
		angle += 2.0 * np.pi
	return angle


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

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
   
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
 
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]

heading_array = []
heading = 0
filtered_heading = 0
i=0
yaw_offset = -0.24  #초기 offset
before_qz = 0
before_qw = 1
def callback(gps, imu, vel):
	global i, yaw_offset,before_qw, before_qz, heading
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

    #heading=np.arctan(msg.twist.twist.linear.y /vel.twist.twist.linear.x )
	
    #if msg.twist.twist.linear.x < 0 :
    #    heading=heading+np.pi
	heading=math.atan2(vel.twist.twist.linear.y , vel.twist.twist.linear.x)
	heading_array.insert(0,heading)   

	if len(heading_array) == 3:   # save number
		heading_array.pop()
	#filtered_heading=heading
	filtered_heading = (sum(heading_array)/len(heading_array))   # moving average
	# filtered_heading = np.median(heading_array)  #moving median	
    
	if i==0:
		before_qz = 0
		before_qw = 1
	if (abs(vel.twist.twist.linear.x) or abs(vel.twist.twist.linear.y)) > 0.1:
		now_qz=np.sin(filtered_heading / 2)
		now_qw=np.cos(filtered_heading / 2)
		gps_qz = now_qz
		gps_qw = now_qw
		before_qz = now_qz
		before_qw = now_qw
	else:
		gps_qz = before_qz
		gps_qw = before_qw

	velocity = np.sqrt(vel.twist.twist.linear.x ** 2 + vel.twist.twist.linear.y ** 2)
	if velocity > 3:
		gpose.pose.pose.orientation.x= 0
		gpose.pose.pose.orientation.y= 0
		gpose.pose.pose.orientation.z= gps_qz
		gpose.pose.pose.orientation.w= gps_qw
		gps_yaw = euler_from_quaternion(0,0,gps_qz,gps_qw)
		imu_yaw = euler_from_quaternion(imu.quaternion.x, imu.quaternion.y, imu.quaternion.z, imu.quaternion.w)
		if abs(imu_yaw - gps_yaw) < np.pi/2:
			yaw_offset = imu_yaw - gps_yaw
		else:
			pass
	else:
		imu_yaw = euler_from_quaternion(imu.quaternion.x, imu.quaternion.y, imu.quaternion.z, imu.quaternion.w)
		final_imu_yaw = normalize_angle(imu_yaw - yaw_offset)  # -180 ~ 180 넘어가는부분?어떻게 하더라
		imu_qx, imu_qy, imu_qz, imu_qw = get_quaternion_from_euler(0,0,final_imu_yaw)
		gpose.pose.pose.orientation.x= imu_qx
		gpose.pose.pose.orientation.y= imu_qy
		gpose.pose.pose.orientation.z= imu_qz
		gpose.pose.pose.orientation.w= imu_qw
	
	gpose.twist.twist.linear.x = vel.twist.twist.linear.x
	gpose.twist.twist.linear.y = vel.twist.twist.linear.y
	gpose.twist.twist.linear.z = vel.twist.twist.linear.z

	Odom_Pub.publish(gpose)
	r.sleep()
	i = i+1
	
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