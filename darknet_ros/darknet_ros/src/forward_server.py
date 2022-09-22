#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ossaudiodev import SOUND_MIXER_DIGITAL1
from scipy.stats import mode

import rospy
import serial
import sys
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32MultiArray

class YoloPub():
    def __init__(self, class_map, queue_size, thresh):
        self.queue_size = queue_size
        self.threshold = thresh
        self.callback_flag = False

        self.queue_list = [[0 for i in range(self.queue_size)] for j in range(len(class_map))]

        self.id_to_queue_list = [self.queue_list[i] for i in range(len(class_map)) for _ in range(len(class_map[i]))]

        self.forward_pub = rospy.Publisher('/forward_sign', Int32MultiArray, queue_size=10)
        self.boundingbox_sub = rospy.Subscriber('/camera1/darknet_ros/bounding_boxes', BoundingBoxes, self.BoundingBoxes_callback)


    def hard_vote(self, queue):
        return int(mode(queue)[0])


    def majority_vote(self, queue):
        # Finding majority candidate
        candidate = 0
        votes = 0

        for i in range(self.queue_size):
            if (votes == 0):
                candidate = queue[i]
                votes = 1
            else:
                votes = votes + 1 if (queue[i] == candidate) else votes - 1
        
        count = 0
        # Checking if majority candidate occurs more than n/2
        # times
        for i in range (self.queue_size):
            if (queue[i] == candidate):
                count += 1
        
        return candidate if (count > self.queue_size // 2) else 0


    def msg_pub(self):
        final_check = Int32MultiArray()
        queue_list = self.queue_list

        # queue voting
        for idx in range(len(queue_list)):
            final_check.data.append(self.hard_vote(queue_list[idx]))

        self.forward_pub.publish(final_check)
        
        final_data = final_check.data[0]
        ser_data = chr(final_data)
        ser.write(ser_data)

        self.callback_flag = False


    def BoundingBoxes_callback(self, data):
        queue_size = self.queue_size

        # append new bounding boxes data
        for bounding_box in data.bounding_boxes:
            if bounding_box.probability >= self.threshold:
                self.id_to_queue_list[bounding_box.id].append(bounding_box.id + 1)
        
        for queue in self.queue_list:
            if len(queue) == queue_size: # append 0 to an undetected classes
                queue.append(0)
            while len(queue) != queue_size: # delete first element
                del queue[0]
        self.callback_flag = True


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',19200,timeout=1)

    CLASS_MAP = (
        ("green", "left", "red", "straightleft", "yellow",),
	("traffic",)
    )
    QUEUE_SIZE = 13
    ACCURACY_THRESHOLD = 0.7

    rospy.init_node('forwrad_yolo')
    node = YoloPub(CLASS_MAP, QUEUE_SIZE, ACCURACY_THRESHOLD)

    while not rospy.is_shutdown():
        if node.callback_flag:
            node.msg_pub()
