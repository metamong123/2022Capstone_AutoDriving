#!/usr/bin/env python

import rospy
import serial
import time
import math
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from pyproj import Proj, transform

import numpy as np

Odom_Pub = rospy.Publisher('/final_odom', Odometry, queue_size=1)

def callback(gps, imu, enc):
	
    cov1 = gps.position_covariance[0] 
    cov2 = gps.position_covariance[4]
    cov3 = gps.position_covariance[8]  
    print(cov1)
    if (cov1 | cov2 | cov3) <= 0.001 :
        if gps.latitude !=float("inf"):
            kcity=Proj(init='epsg:5179')
            wgs84=Proj(init='epsg:4326')
	    a,b=transform(wgs84,kcity,msg.longitude,msg.latitude)
	print(a)
	print(b)
	gpose.pose.pose.position.x=a
	gpose.pose.pose.position.y=b
	gpose.pose.covariance[0]=cov1
	gpose.pose.covariance[7]=cov2
	gpose.pose.covariance[14]=cov3

    else : 
        t=imu.header.stamp
	time_now=t.to_sec()
	delta_time=time_now-time_old
	time_old=time_now
        yaw = imu.angular_velocity.z * delta_time
        linear_vel = enc.twist.twist.linear.x
        x_dot = linear_vel * cos(yaw)
        y_dot = linear_vel * sin(yaw)
        x += x_dot * delta_time + a
        y += y_dot * delta_time + b
        gpose.pose.pose.position.x = x
        gpose.pose.pose.position.y = y
        gpose.pose.covariance[0] = cov1
	gpose.pose.covariance[7] = cov2
	gpose.pose.covariance[14] = cov3
    Odom_Pub.publish(gpose)


if __name__=='__main__':
	
    rospy.init_node('gps_imu_decision')
    gpose=Odometry()
    gpose.header.stamp=rospy.Time.now()
    gpose.header.frame_id="odom"

    gps_sub = message_filters.Subscriber("/gps/fix",NavSatFix)
    imu_sub = message_filters.Subscriber("/imu_data",Imu)
    enc_sub = message_filters.Subscriber("/old_odom",Odometry)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub,imu_sub,enc_sub], 1,10)
    ts.registerCallback(callback)
    rospy.spin()
