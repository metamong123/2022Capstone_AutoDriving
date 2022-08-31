#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy, tf

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from object_msgs.msg import Object, ObjectArray
from separation_axis_theorem import separating_axis_theorem, get_vertice_rect


class ObsMarkerPub:
	def __init__(self):
		self.car_msg = []    # [x, y, yaw, L, W]
		self.obstacle_msg = [] # [(x, y, yaw, L, W), (x, y, yaw, L, W), ... ]

		self.sub_car = rospy.Subscriber('/objects/car_1', Object, self.callback_car)
		self.sub_obstacle = rospy.Subscriber('obstacles', ObjectArray, self.callback_obstacle)
        
		self.obstacle_marker_pub = rospy.Publisher("/objects/marker/obstacles", MarkerArray, queue_size=1)


	def callback_car(self, msg):
		self.car_msg = (msg.x, msg.y, msg.yaw, msg.L, msg.W)


	def callback_obstacle(self, msg):
		self.obstacle_msg = [(o.x, o.y, o.yaw, o.L, o.W) for o in msg.object_list]


	def get_marker_msg(self, obs, id, is_collide):
		m = Marker()
		m.header.frame_id = "/map"
		m.header.stamp = rospy.Time.now()
		m.id = id
		m.type = m.CUBE

		m.pose.position.x = obs[0]
		m.pose.position.y = obs[1]
		m.pose.position.z = 0.75
		m.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, obs[2]))

		m.scale.x = obs[3]
		m.scale.y = obs[4]
		m.scale.z = 1.645

		if is_collide:
			# red
			m.color.r = 192 / 255.0
			m.color.g = 57 / 255.0
			m.color.b = 43 / 255.0
			m.color.a = 0.97
		else:
			# yellow
			m.color.r = 255 / 255.0
			m.color.g = 204 / 255.0
			m.color.b = 102 / 255.0
			m.color.a = 0.97

		return m


	def collision_check_and_publish(self):
		marray = MarkerArray()
		obs_msg = self.obstacle_msg
		car_msg = self.car_msg

		if obs_msg and car_msg:
			for idx, obs in enumerate(obs_msg):
				# collision check
				car_vertices = get_vertice_rect(car_msg)
				obstacle_vertices = get_vertice_rect(obs)
				is_collide = separating_axis_theorem(car_vertices, obstacle_vertices)

				marker = self.get_marker_msg(obs, idx, is_collide)
				marray.markers.append(marker)

		self.obstacle_marker_pub.publish(marray)


if __name__ == "__main__":
    rospy.init_node("collision_checking_marker_node")
    collision_check = ObsMarkerPub()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        collision_check.collision_check_and_publish()
        r.sleep()
