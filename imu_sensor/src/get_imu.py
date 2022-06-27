#!/usr/bin/env python

import serial
import rospy
import numpy as np
import math



if __name__ == '__main__':
    rospy.init_node("get_imu")
    port = rospy.get_param("~GPS_PORT",port)
    ser = serial.serial_for_url(port,115200, timeout=0)
    
    imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10)
    
    


