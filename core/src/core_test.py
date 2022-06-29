#!/usr/bin/env python

import rospy

global a
a = 'default'    
if __name__=='__main__':

   rospy.init_node('core')
   rospy.set_param('car_mode', 'global')
   a = rospy.get_param('car_mode')
   print(a)
   r= rospy.Rate(1)
   r.sleep()
   rospy.set_param('car_mode', 'parking')
   a = rospy.get_param('car_mode')
   print(a)
   r= rospy.Rate(1)
   r.sleep()
   rospy.spin()