#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

global a
a = 'default'

if __name__=='__main__':

   rospy.init_node('core_control')
   rospy.set_param('car_mode', 'global')
   
   rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
   rospy.Subscriber('/ackermann_cmd_parking_backward',AckermannDriveStamped,frenet_callback)
   r= rospy.Rate(1)
   r.sleep()
   rospy.set_param('car_mode', 'parking')
   a = rospy.get_param('car_mode')
   print(a)
   r= rospy.Rate(1)
   r.sleep()
   rospy.spin()