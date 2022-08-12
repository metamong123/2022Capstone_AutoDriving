#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
import itertools
import argparse
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point
from object_msgs.msg import Object, ObjectArray
from separation_axis_theorem import separating_axis_theorem, get_vertice_rect


class collision_check_marker():
    def __init__(self, num_objects = None):
        self.obstacle_msg = [] # [(x, y, yaw, L, W), (x, y, yaw, L, W), ... ]
        self.car_msg = []    # [x, y, yaw, L, W]
        self.marker_msg = {}
        self.collision_check_obs = []
        self.collision_check_car = False
        self.num_objects = num_objects

        #Car, Obstacle Array Subscriber
        self.sub_car = rospy.Subscriber('/objects/car_1/gps', Object, self.callback_car)
        self.sub_obstacle = rospy.Subscriber('obstacles', ObjectArray, self.callback_obstacle)
        
        #self.car_marker_pub = rospy.Publisher("/objects/marker/car_1", Marker, queue_size=1)
        self.obstacle_marker_pub = rospy.Publisher("/objects/marker/obstacles", MarkerArray, queue_size=1)

    def callback_car(self, msg):
        self.car_msg = [msg.x, msg.y, msg.yaw, msg.L, msg.W]

    def callback_obstacle(self, msg):
        self.obstacle_msg = []
        for o in msg.object_list:
            '''
            #####(x, y) 좌표가 중심이 아니라 시작점인지?
            yaw = o.yaw           
            center_x = o.x + 1.3 * math.cos(yaw)
            center_y = o.y + 1.3 * math.sin(yaw)
            '''
            #id(=i)가 문자열이어야 하는지 확인 필요
            self.obstacle_msg.append((o.x, o.y, o.yaw, o.L, o.W))

    def get_marker_msg(self, msg_tuple, id, is_collide):

        x = msg_tuple[0]
        y = msg_tuple[1]
        yaw = msg_tuple[2]
        L = msg_tuple[3]
        W = msg_tuple[4]

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        m = Marker()
        m.header.frame_id = "/map"
        m.header.stamp = rospy.Time.now()
        m.id = int(id)
        m.type = m.CUBE

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.75
        m.pose.orientation = Quaternion(*quat)

        m.scale.x = L
        m.scale.y = W
        m.scale.z = 1.645

        if is_collide:
            m.color.r = 192 / 255.0
            m.color.g = 57 / 255.0
            m.color.b = 43 / 255.0
            m.color.a = 0.97
        else:
            m.color.r = 93 / 255.0
            m.color.g = 122 / 255.0
            m.color.b = 177 / 255.0
            m.color.a = 0.97

        return m

    '''
    def get_sphere_marker_list_msg(self, vertice_all):
        m = Marker()
        m.header.frame_id = "/map"
        m.header.stamp = rospy.Time.now()
        m.id = 5
        m.type = m.POINTS

        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5

        m.color.r = 0
        m.color.g = 1
        m.color.b = 0
        m.color.a = 0.97
        for vertice_subset in vertice_all:
            for vertex in vertice_subset:
                point = Point()
                point.x = vertex[0]
                point.y = vertex[1]
                m.points.append(point)
        return m
    '''

    def collision_check_and_publish(self):
        marray = MarkerArray()
        obs_msg = self.obstacle_msg
        car_msg = self.car_msg
        self.collision_check_obs = []
        self.collision_check_car = False
        idx = len(obs_msg)

        if idx and car_msg:
            self.collision_check_car = False
            for i in range(idx):
                self.collision_check_obs.append(False)

            for i in range(idx):
                car_vertices = get_vertice_rect(car_msg)
                obstacle_vertices = get_vertice_rect(obs_msg[i])
                is_collide = separating_axis_theorem(car_vertices, obstacle_vertices)
                if is_collide:
                    self.collision_check_car = True
                    self.collision_check_obs[i] = True

            for id in range(idx):
                marker = self.get_marker_msg(obs_msg[id], id + 2, self.collision_check_obs[id])
                marray.markers.append(marker)

            self.obstacle_marker_pub.publish(marray)


if __name__ == "__main__":
    rospy.init_node("collision_checking_marker_node")
    collision_check = collision_check_marker()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        collision_check.collision_check_and_publish()
        r.sleep()
