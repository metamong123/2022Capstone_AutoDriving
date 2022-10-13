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
	imu_yaw = euler_from_quaternion(imu.quaternion.x, imu.quaternion.y, imu.quaternion.z, imu.quaternion.w) + 0.27
	imu_qx, imu_qy, imu_qz, imu_qw = get_quaternion_from_euler(0,0,imu_yaw)
	gpose.pose.pose.orientation.x= imu_qx
	gpose.pose.pose.orientation.y= imu_qy
	gpose.pose.pose.orientation.z= imu_qz
	gpose.pose.pose.orientation.w= imu_qw
	
	#gpose.pose.pose.orientation.x= imu.quaternion.x
	#gpose.pose.pose.orientation.y= imu.quaternion.y
	#gpose.pose.pose.orientation.z= imu.quaternion.z
	#gpose.pose.pose.orientation.w= imu.quaternion.w
	
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

		
	