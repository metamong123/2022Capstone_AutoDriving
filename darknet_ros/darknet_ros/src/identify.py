#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int32MultiArray

         
# CLASS====================================================================================
# 0 : A1  1 : A2  2 : A3  3 : B1  4 : B2  5: B3                  idx = 0(1 2 3), 1(1 2 3)
# 6 : green  7 : left  8 : red  9 : straightleft  10 : yellew    idx = 2(0 1 2 3 4)
# 11 : person  12 : car  13 : uturn  14 : kidzone  
# 15 : parking  16 : stopline                                    idx = 3 ~ 8 (0 0 0 0 0 0)
# =========================================================================================

# Ex===========================================================================
# count = [0,0,10, 10,10,10, 0,0,0,10,0, 0,10,0,10,0,0, 1,20,60,40]
# pub = [3, 2, 3, 0, 1, 0, 1, 0, 0] (A3, B1-B3-B2, straightleft, car, kidzone)
# =============================================================================

def pub_detected(count):
    
    thresh = 20
    
    delivery_A = [count[0], count[1], count[2]]
    delivery_B = [count[3], count[4], count[5]]
    traffic_sign = [count[6], count[7], count[8], count[9], count[10]]
    detect_else = [count[11], count[12], count[13], count[14], count[15], count[16]]
    B_flag = count[17]
    B_ypos = [count[18], count[19], count[20]]
    
    class_list=["A1", "A2", "A3", "B1", "B2", "B3", "green", "left", "red", "straightleft", "yellow", "person", "car", "uturn", "kidzone", "parking", "stopline"]
    
    final_check=Int32MultiArray()
    final_check.data = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    global A_num

    # 배달 미션 - A표지판 판별
    max_A = delivery_A[0]
    for a in range(len(delivery_A)):
        if (delivery_A[a] >= max_A):
            max_A = delivery_A[a]
            A_num = a
    if (max_A >= thresh):
        final_check.data[0] = A_num + 1
        print(class_list[A_num])
    else:
        final_check.data[0] = 0
    detected_A = A_num
    #배달 미션 - B표지판 위치를 통해 경로 설정
    if (B_flag == 1):
        if all(b >= thresh for b in delivery_B):
            min_B = min(B_ypos)
            max_B = max(B_ypos)
            if (B_ypos[A_num] == min_B):
                final_check.data[1] = 1
            elif (B_ypos[A_num] == max_B):
                final_check.data[1] = 3
            else:
                final_check.data[1] = 2
            print(class_list[A_num + 3])

    #신호등 확인
    clearly_sign = traffic_sign[0]
    for t in range(len(traffic_sign)):
        if (traffic_sign[t] >= clearly_sign):
            clearly_sign = traffic_sign[t]
            traffic_num = t
    if (clearly_sign >= thresh):
        final_check.data[2] = traffic_num
        print(class_list[traffic_num + 6])
    else:
        final_check.data[2] = 0
    
    #그 외 객체 판별
    for i in range(len(detect_else)):
        if (detect_else[i] >= thresh):
            final_check.data[i + 3] = 1
            print(class_list[i+11])
        else:
            final_check.data[i + 3] = 0
    
    pub_ID = rospy.Publisher('/detect_ID', Int32MultiArray, queue_size=1)
    pub_ID.publish(final_check)


def BoundingBoxes_callback(data):
    
    now = time.gmtime(time.time())
    sec = now.tm_sec
    
    number = len(data.bounding_boxes)
    for i in range(number):
        class_id = data.bounding_boxes[i].id
        xmin = data.bounding_boxes[i].xmin
        xmax = data.bounding_boxes[i].xmax
        ymin = data.bounding_boxes[i].ymin
        ymax = data.bounding_boxes[i].ymax

        #정확도 판별
        if (data.bounding_boxes[i].probability > 0.5):
            #신호등 인식 범위 지정
            if(class_id >= 6 and class_id <= 10):
                if(xmin >= 100 and xmax <= 540):
                    detected_count[class_id] += 1
                    detected_time[class_id] = sec
            else:
                detected_count[class_id] += 1
                detected_time[class_id] = sec
                #B 표지판 정보 저장
                if (class_id >= 3 and class_id <= 5):
                    detected_count[17] = 1
                    detected_count[class_id + 15] = ymin
    #객체가 더이상 없다고 판단할 시간 설정
    for i in range(6,len(detected_time)):
        if (sec - detected_time[i] >= 3):
            detected_count[i] = 0

    pub_detected(detected_count)


if __name__ == '__main__':
    
    global pub_sign, detected_count, detected_time
    
    detected_count = np.zeros(21, dtype=int)
    detected_time = np.zeros(17)
    for i in range(17):
        detected_time[i] = time.time()
    
    rospy.init_node('obj_ID', anonymous=True)
    rospy.Subscriber("/camera1/darknet_ros/bounding_boxes", BoundingBoxes, BoundingBoxes_callback)
    rate = rospy.Rate(10)
    
# /bounding_boxes==============================================================
# float64 probability
# int64 xmin
# int64 ymin
# int64 xmax
# int64 ymax
# int16 id
# string Class
# =============================================================================
    
    while (True):
        try:
            pass
        except rospy.ROSInterruptException:
            pass
