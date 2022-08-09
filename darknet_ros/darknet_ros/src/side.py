#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scipy.stats import mode

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32MultiArray

         
# CLASS ====================================================================================
# 0 : A1  1 : A2  2 : A3  3 : B1  4 : B2  5: B3                  idx = 0(1 2 3), 1(1 2 3)
# 6 : green  7 : left  8 : red  9 : straightleft  10 : yellew    idx = 2(0 1 2 3 4)
# 11 : person  12 : car  13 : uturn  14 : kidzone
# 15 : stopline                                                  idx = 3 ~ 8 (0 0 0 0 0 0)
# =========================================================================================

# OUTPUT ===========================================================================
# pub = [3, 2, 3, 0, 1, 0, 1, 0, 0] (A, B1_x, B2_x, B3_x, traffic_light, person, car, sign)
# =============================================================================

class YoloPub():
    def __init__(self, class_map, queue_size, thresh):
        self.queue_size = queue_size
        self.threshold = thresh
        self.callback_flag = False

        # [[A queue], [B1 queue], [B2 queue], ...]
        self.queue_list = [[-1 for i in range(self.queue_size)] for j in range(len(class_map))]

        # [[A1 to A queue], [A2 to A queue], [A3 to A queue], [B1 to B1 queue], [B2 to B2 queue], [B3 to B3 queue], ...]
        self.id_to_queue_list = [self.queue_list[i] for i in range(len(class_map)) for _ in range(len(class_map[i]))]

        self.id_pub = rospy.Publisher('/side_sign', Int32MultiArray, queue_size=10)
        self.boundingbox_sub = rospy.Subscriber('/camera2/darknet_ros/bounding_boxes', BoundingBoxes, self.BoundingBoxes_callback)


    def deliveryB_vote(self, queue):
        if queue.count(-1) > int(self.queue_size/2):
            return -1
        else:
            for element in queue: # get latest x_min
                if element != -1:
                    val = element
            return val


    def hard_vote(self, queue):
        return int(mode(queue)[0])


    def majority_vote(self, queue):
        # Finding majority candidate
        candidate = -1
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
        
        return candidate if (count > self.queue_size // 2) else -1


    def msg_pub(self):
        final_check = Int32MultiArray()
        queue_list = self.queue_list

        # queue voting
        for idx in range(len(queue_list)):
            if idx in (1, 2, 3): # (B1, B2, B3)
                final_check.data.append(self.deliveryB_vote(queue_list[idx]))
            else:
                final_check.data.append(self.hard_vote(queue_list[idx]))

        self.id_pub.publish(final_check)
        self.callback_flag = False


    def BoundingBoxes_callback(self, data):
        queue_size = self.queue_size

        # append new bounding boxes data
        for bounding_box in data.bounding_boxes:
            if bounding_box.probability >= self.threshold:
                if bounding_box.id in (3, 4, 5): # (B1, B2, B3)
                    self.id_to_queue_list[bounding_box.id].append(bounding_box.xmin)
                else:
                    self.id_to_queue_list[bounding_box.id].append(bounding_box.id)
        
        for queue in self.queue_list:
            if len(queue) == queue_size: # append -1 to an undetected classes
                queue.append(-1)
            while len(queue) != queue_size: # delete first element
                del queue[0]
        self.callback_flag = True


if __name__ == '__main__':

    CLASS_MAP = (
        ("A1", "A2", "A3"),
        ("B1",),
        ("B2",),
        ("B3",)
    )
    QUEUE_SIZE = 7
    ACCURACY_THRESHOLD = 0.7

    rospy.init_node('side_yolo')
    node = YoloPub(CLASS_MAP, QUEUE_SIZE, ACCURACY_THRESHOLD)

    while not rospy.is_shutdown():
        if node.callback_flag:
            node.msg_pub()
