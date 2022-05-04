#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, Float64
from nav_msgs.msg import Odometry



def callback(msg):

	new_odom.header.stamp=rospy.Time.now()
	new_odom.header.frame_id="odom"
	global i, first_odom_x, first_odom_y
	
	if i==0:
           first_odom_x = msg.pose.pose.position.x - 0.3581
	   first_odom_y = msg.pose.pose.position.y - 1.0637
	new_odom.pose.pose.position.x=msg.pose.pose.position.x - first_odom_x
	

 #x offset / x<->y trans??
	new_odom.pose.pose.position.y=msg.pose.pose.position.y -first_odom_y
	new_odom.pose.pose.position.z=msg.pose.pose.position.z

	new_odom.pose.pose.orientation.x=msg.pose.pose.orientation.x
	new_odom.pose.pose.orientation.y=msg.pose.pose.orientation.y
	new_odom.pose.pose.orientation.z=msg.pose.pose.orientation.z
	new_odom.pose.pose.orientation.w=msg.pose.pose.orientation.w

	new_odom.pose.covariance[0]=0.00008
	new_odom.pose.covariance[7]=0.00008
	new_odom.pose.covariance[14]=1000
	new_odom.pose.covariance[21]=1000
	new_odom.pose.covariance[28]=1000
	new_odom.pose.covariance[35]=0.01
	new_odom.twist.twist.linear.x=msg.twist.twist.linear.x
	new_odom.twist.twist.linear.y=msg.twist.twist.linear.y
	new_odom.twist.twist.linear.z=msg.twist.twist.linear.z
	
	new_odom.twist.twist.angular.x=msg.twist.twist.angular.x
	new_odom.twist.twist.angular.y=msg.twist.twist.angular.y
	new_odom.twist.twist.angular.z=msg.twist.twist.angular.z

	new_odom.twist.covariance[0]=0.1
	new_odom.twist.covariance[7]=0.1
	new_odom.twist.covariance[14]=1000
	new_odom.twist.covariance[21]=1000
	new_odom.twist.covariance[28]=1000
	new_odom.twist.covariance[35]=1000

	new_odom_pub=rospy.Publisher('/odom',Odometry,queue_size=1)
	#rate=rospy.Rate(1)
	#rate.sleep()
	new_odom_pub.publish(new_odom)
	
	i=i+1

if __name__=='__main__':

	rospy.init_node('odom_with_cov')

	global new_odom
	global i
	i=0

	new_odom=Odometry()	

	rospy.Subscriber("/old_odom",Odometry,callback)

	rospy.spin()
