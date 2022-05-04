#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


def callback(msg):

	new_odom.header=msg.header
	new_odom.header.frame_id="/base_footprint"
	new_odom.pose.position.x=msg.pose.pose.position.x #x offset / x<->y trans??
	new_odom.pose.position.y=msg.pose.pose.position.y
	new_odom.pose.position.z=msg.pose.pose.position.z

	new_odom.pose.orientation.x=msg.pose.pose.orientation.x
	new_odom.pose.orientation.y=msg.pose.pose.orientation.y
	new_odom.pose.orientation.z=msg.pose.pose.orientation.z
	new_odom.pose.orientation.w=msg.pose.pose.orientation.w


	new_odom_pub=rospy.Publisher('/final_pose',PoseStamped,queue_size=1)
	new_odom_pub.publish(new_odom)


if __name__=='__main__':

	rospy.init_node('finalpose')

	global new_odom
	new_odom=PoseStamped()
	
	rospy.Subscriber("/robot_pose_ekf/odom_combined",PoseWithCovarianceStamped,callback)

	rospy.spin()
