#! /usr/bin/python
#-*- coding: utf-8 -*-

import time
import math
import rospy
import numpy as np
import pickle
import sys
import rospkg
rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
sys.path.append(path_map + "/src/")
from map_visualizer import Converter
# import tf
# from tf.transformations import euler_from_quaternion
# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Quaternion
from object_msgs.msg import Path
import json

if __name__ == "__main__":
    
	rospy.init_node("path")
	path_pub=rospy.Publisher("/path", Path,queue_size=10)

	r = rospy.Rate(10)

	with open(path_map + "/src/route.pkl", "rb") as f:
		nodes = pickle.load(f)

	encoded_data_string = json.dumps({... your dictionary ...})

	loaded_dictionary = json.loads(encoded_data_string)


	x=0.0
	y=0.0
	# speed=0
	while not rospy.is_shutdown():


		p = Path()
		o.header.frame_id = "/map"
		o.header.stamp = rospy.Time.now()
		o.id = 1
		o.classification = o.CLASSIFICATION_CAR
		o.x = x
		o.y = y
		o.yaw = yaw
		#o.yaw=1.28713
		########
		o.v = v
		#######
		o.L = 1.600
		o.W = 1.160
		#o.WB = 1.06
		path_pub.publish(p)		
		r.sleep()
