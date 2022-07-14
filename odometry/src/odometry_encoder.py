#!/usr/bin/env python

import rospy
import serial
import time
import math
import pickle

from std_msgs.msg import Header, Float64, Int8
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from pyproj import Proj, transform
from nav_msgs.msg import Odometry

import numpy as np

vo_Pub=rospy.Publisher('/odom',Odometry,queue_size=1)

begin_encoder = 0
a=0
b=0
heading=0
i=0
begin_meter=0

rpose=Odometry()
rpose.header.frame_id="odom"

class TopicReciver:
	def __init__(self):
		self.gps_sub=rospy.Subscriber("/gps/fix",NavSatFix,self.callback)
		self.heading_sub=rospy.Subscriber("/ublox_gps/fix_velocity",TwistWithCovarianceStamped,self.callback1)
		self.velocity_sub=rospy.Subscriber("ERP42_encoder",Float64, self.callback2)
	def check_all_connections(self):
		return (self.gps_sub.get_num_connections()+self.heading_sub.get_num_connections()+self.velocity_sub.get_num_connections())==3
	def callback(self, msg):
		global a, b, rpose
		if self.check_all_connections():

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

	def callback1(self, msg):
		if self.check_all_connections():
			global i, before_qz, before_qw, heading, rpose
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
			print('1')
			#rpose.twist.twist.linear.x = msg.twist.twist.linear.x
			#rpose.twist.twist.linear.y = msg.twist.twist.linear.y
			#rpose.twist.twist.linear.z = msg.twist.twist.linear.z
			rpose.pose.covariance[21]=99999
			rpose.pose.covariance[28]=99999
			rpose.pose.covariance[35]=msg.twist.covariance[0]
			#rate=rospy.Rate(1)
			#rate.sleep()
			
			
			i=i+1

	def callback2(self, msg):
		if self.check_all_connections():
			global j, begin_encoder, begin_meter, time_last, rpose
			time_current = rospy.Time.now()
			time_current = time_current.to_sec()
			delta_time = time_current - time_last
			#print(delta_time)
			time_last = time_current
			last_encoder = msg.data
			
			meter = last_encoder-begin_encoder
			if meter <= -4000000000:
				meter = meter + 4294967295
			elif meter >= 4000000000:
				meter = meter - 4294967295
			if (meter == 0):
				fixed_meter = begin_meter
			else:
				fixed_meter = meter
			fixed_meter = fixed_meter/59.6  # 1m --> 59.6tic
			meter_per_sec = - (fixed_meter / delta_time)
			#print(meter_per_sec)
			begin_meter = meter  # meter zero prevent
			begin_encoder = last_encoder
			rpose.twist.twist.linear.x = meter_per_sec
			print("rpose"+str(rpose))
			vo_Pub.publish(rpose)



if __name__=='__main__':
	rospy.init_node('gps_to_vo')
	# global rpose
	j=0
	
	if j==0:
		begin_encoder=0
		time_last=0
	
	
	topic_receiver=TopicReciver()
	# print(rpose)
	rospy.spin()
	j=j+1