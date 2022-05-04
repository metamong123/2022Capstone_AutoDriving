#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, Float64
from nav_msgs.msg import Odometry


def callback(msg):

	new_odom.header=msg.header

	new_odom.pose.pose.position.x=msg.pose.pose.position.x - 330222.733428
	new_odom.pose.pose.position.y=msg.pose.pose.position.y - 4166720.57941
	new_odom.pose.pose.position.z=msg.pose.pose.position.z - 58.441

	new_odom.pose.pose.orientation.x=msg.pose.pose.orientation.x
	new_odom.pose.pose.orientation.y=msg.pose.pose.orientation.y
	new_odom.pose.pose.orientation.z=msg.pose.pose.orientation.z
	new_odom.pose.pose.orientation.w=msg.pose.pose.orientation.w

	new_odom.pose.covariance[0]=msg.pose.covariance[0]
	new_odom.pose.covariance[7]=msg.pose.covariance[7]
	new_odom.pose.covariance[14]=msg.pose.covariance[14]
	new_odom.pose.covariance[21]=msg.pose.covariance[21]
	new_odom.pose.covariance[28]=msg.pose.covariance[28]
	new_odom.pose.covariance[35]=msg.pose.covariance[35]

	new_odom.twist.twist.linear.x=msg.twist.twist.linear.x
	new_odom.twist.twist.linear.y=msg.twist.twist.linear.y
	new_odom.twist.twist.linear.z=msg.twist.twist.linear.z
	
	new_odom.twist.twist.angular.x=msg.twist.twist.angular.x
	new_odom.twist.twist.angular.y=msg.twist.twist.angular.y
	new_odom.twist.twist.angular.z=msg.twist.twist.angular.z

	new_odom.twist.covariance[0]=msg.twist.covariance[0]
	new_odom.twist.covariance[7]=msg.twist.covariance[7]
	new_odom.twist.covariance[14]=msg.twist.covariance[14]
	new_odom.twist.covariance[21]=msg.twist.covariance[21]
	new_odom.twist.covariance[28]=msg.twist.covariance[28]
	new_odom.twist.covariance[35]=msg.twist.covariance[35]

	new_odom_pub=rospy.Publisher('/vo',Odometry,queue_size=1)
	new_odom_pub.publish(new_odom)


if __name__=='__main__':

	rospy.init_node('gps_offset')

	global new_odom
	new_odom=Odometry()
	
	rospy.Subscriber("/vo_old",Odometry,callback)

	rospy.spin()
