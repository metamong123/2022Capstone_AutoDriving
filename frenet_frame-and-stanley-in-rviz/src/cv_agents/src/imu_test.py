#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

if __name__ == "__main__":
    rospy.init_node("get_imu")
    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
    imu=Imu()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        imu.linear_acceleration.x = 0   #gravity eliminate velocity!!!!
        imu.linear_acceleration.y = 0   #gravity eliminate velocity!!!!
        imu.linear_acceleration.z = 0   #gravity eliminate velocity!!!!
        
        imu_pub.publish(imu)
        r.sleep()