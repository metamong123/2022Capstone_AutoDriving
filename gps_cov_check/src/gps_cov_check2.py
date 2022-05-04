#!/usr/bin/env python

import rospy
import serial
import time
import math
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from pyproj import Proj, transform
from geometry_msgs.msg import TwistWithCovarianceStamped

import numpy as np

Odom_Pub = rospy.Publisher('/final_odom', Odometry, queue_size=1)

time_old_imu=0
time_old_enc=0
time_old=0
x=0
y=0
a=0
b=0
yaw=0
cov1=0
cov2 = 0
heading=0
i=0
filtered_heading = 0
heading_array = []
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

def callback1(gps):

    global a,b,cov1,cov2

    cov1 = gps.position_covariance[0] 
    cov2 = gps.position_covariance[4]
    cov3 = gps.position_covariance[8] 
    
    if (cov1 <= 0.0003 or cov2 <= 0.0003 ) :
        if gps.latitude !=float("inf"):
            kcity=Proj(init='epsg:5179')
            wgs84=Proj(init='epsg:4326')
            a,b=transform(wgs84,kcity,gps.longitude,gps.latitude)
        a -= 962643
        b -= 1959323
	gpose.pose.pose.position.x=a
	gpose.pose.pose.position.y=b
	gpose.pose.covariance[0]=cov1
	gpose.pose.covariance[7]=cov2
       

def callback4(msg):
	
	global i, before_qz, before_qw, filtered_heading

	if (cov1 <= 0.0003 or cov2 <= 0.0003 ) :
		print("gps!!")
		if i==0:
			before_qz = 0
			before_qw = 1
			
		heading=np.arctan( msg.twist.twist.linear.y / msg.twist.twist.linear.x )

		if msg.twist.twist.linear.x < 0 :
			heading=heading+np.pi
		heading_array.insert(0,heading)
		qx=0
		qy=0
		if len(heading_array) == 9:
			heading_array.pop()
		filtered_heading = (sum(heading_array)/len(heading_array))
		#print(heading_array)

		if (abs(msg.twist.twist.linear.x) or abs(msg.twist.twist.linear.y)) > 0.1:
			qz=np.sin(filtered_heading / 2)
			qw=np.cos(filtered_heading / 2)
			gpose.pose.pose.orientation.x=qx	
			gpose.pose.pose.orientation.y=qy	
			gpose.pose.pose.orientation.z=qz	
			gpose.pose.pose.orientation.w=qw
			before_qz = qz
			before_qw = qw
			
		else :
			gpose.pose.pose.orientation.x=qx	
			gpose.pose.pose.orientation.y=qy	
			gpose.pose.pose.orientation.z=before_qz	
			gpose.pose.pose.orientation.w=before_qw
		
		
		i=i+1


		Odom_Pub.publish(gpose)

def callback2(imu):

    global yaw,time_old_imu
    
    t=imu.header.stamp
    time_now=t.to_sec()
    delta_time=time_now-time_old_imu
    time_old_imu=time_now
    angular = imu.angular_velocity.z
    if (cov1 <= 0.0003 or cov2 <= 0.0003 ) :
	yaw = filtered_heading
    else :
        yaw += imu.angular_velocity.z * delta_time
        [qx,qy,qz,qw]=get_quaternion_from_euler(0,0,yaw)

	gpose.pose.pose.orientation.x=qx	
	gpose.pose.pose.orientation.y=qy	
	gpose.pose.pose.orientation.z=qz	
	gpose.pose.pose.orientation.w=qw

def callback3(enc):

    global time_old_enc

    t=enc.header.stamp
    time_now=t.to_sec()
    delta_time=time_now-time_old_enc
    time_old_enc=time_now

    linear_vel = -enc.twist.twist.linear.x
    x_dot = linear_vel * np.cos(yaw)
    y_dot = linear_vel * np.sin(yaw)

    new_odom(yaw,x_dot,y_dot)

def new_odom(yaw,x_dot,y_dot):

    global time_old,x,y
    t=rospy.Time.now()
    time_now=t.to_sec()
    delta_time=time_now-time_old
    time_old=time_now
    if (cov1 <= 0.0006 or cov2 <= 0.0006 ) :
        x = a
        y = b
    else :
	print("imu+enc!!")
        x += x_dot * delta_time
        y += y_dot * delta_time 

        gpose.pose.pose.position.x=x
        gpose.pose.pose.position.y=y
        gpose.pose.covariance[0]=cov1
	gpose.pose.covariance[7]=cov2
        Odom_Pub.publish(gpose)


if __name__=='__main__':
	
    rospy.init_node('gps_imu_decision')
    gpose=Odometry()
    gpose.header.stamp=rospy.Time.now()
    
    gpose.header.frame_id="odom"
    rospy.Subscriber("/gps/fix",NavSatFix,callback1)
    rospy.Subscriber("/imu_data",Imu,callback2)
    rospy.Subscriber("/old_odom",Odometry,callback3)
    rospy.Subscriber("/ublox_gps/fix_velocity",TwistWithCovarianceStamped,callback4)
	
    

    rospy.spin()
