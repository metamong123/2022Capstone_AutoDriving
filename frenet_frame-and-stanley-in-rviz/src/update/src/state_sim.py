#! /usr/bin/python
#-*- coding: utf-8 -*-
import rospkg
import sys

import rospy
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object


rospack = rospkg.RosPack()
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/path/")
from path_map import *

x=0.0
y=0.0
yaw=0.0
v=0.0
id=1
use_map=kcity()
start_index=0
obj_msg=Object(x=use_map.nodes['global'][start_index]['x'][0],y=use_map.nodes['global'][start_index]['y'][0],yaw=0,v=0,L=1.600,W=1.04)

class TopicReciver:
	def __init__(self):
		self.odom_sub=rospy.Subscriber("/objects/car_1",  Object,self.odometry_callback)
	def check_all_connections(self):
		return (self.odom_sub.get_num_connections())==1
	def odometry_callback(self, msg): 
		if self.check_all_connections():
			global obj_msg
			obj_msg=msg

if __name__ == "__main__":
    
	rospy.init_node("state")

	topic_receiver=TopicReciver()

	marker_pub=rospy.Publisher("/objects/marker/car_1", Marker,queue_size=1)
	
	r = rospy.Rate(10)

	while not rospy.is_shutdown():

		tf_broadcaster = tf.TransformBroadcaster()
        
		quat = tf.transformations.quaternion_from_euler(0, 0, obj_msg.yaw)
		tf_broadcaster.sendTransform((obj_msg.x, obj_msg.y, 1.5),quat,rospy.Time.now(),"/car_" + str(id), "/map")
		
		m = Marker()
		m.header.frame_id = "/map"
		m.header.stamp = rospy.Time.now()
		m.id = id
		m.type = m.CUBE

		m.pose.position.x = obj_msg.x
		m.pose.position.y = obj_msg.y
		m.pose.position.z = 0.3
		m.pose.orientation = Quaternion(*quat)

		m.scale.x = 1.600
		m.scale.y = 1.160
		m.scale.z = 1.645

		m.color.r = 93 / 255.0
		m.color.g = 122 / 255.0
		m.color.b = 177 / 255.0
		m.color.a = 0.97

		marker_pub.publish(m)
		
		r.sleep()