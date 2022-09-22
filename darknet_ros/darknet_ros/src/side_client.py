#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import sys
from std_msgs.msg import Int32MultiArray

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',19200,timeout=1)
    delivery_sign = Int32MultiArray()
    delivery_sign.data = [0,0,0,0,0,0,0]
    raw_data=[]
    side_pub = rospy.Publisher('/side_sign', Int32MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        rospy.init_node('side_yolo')
        ser_data = ser.read()
        if ser_data == "":
            print("=====404 NOT FOUND=====")
        elif ser_data == "/":
            raw_data = []
        else:
            raw_data.append(ord(ser_data))

        if len(raw_data) == 19:
            delivery_sign.data[0] = raw_data[0]
            for i in range(6):
                xmean = raw_data[3*i + 1]*100 + raw_data[3*i +2]*10 +raw_data[3*i +3]
                delivery_sign.data[i + 1] = xmean
        
        side_pub.publish(delivery_sign)


        