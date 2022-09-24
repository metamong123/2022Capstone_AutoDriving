#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import sys
from std_msgs.msg import Int32MultiArray

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0',19200,timeout=1)
    traffic_sign = Int32MultiArray()
    traffic_sign.data = [0]
    forward_pub = rospy.Publisher('/forward_sign', Int32MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        rospy.init_node('forwrad_yolo')
        ser_data = ser.read()
        if ser_data == "":
            print("=====404 NOT FOUND=====")
        else:
            traffic_sign.data[0]=ord(ser_data)
        forward_pub.publish(traffic_sign)