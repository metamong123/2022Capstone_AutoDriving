#!/usr/bin/env python

import rospy
import numpy as np

from pyproj import Proj, transform
from nav_msgs.msg import Odometry

global i
global parking_flag

i=0
parking_flag = 'notparking'

def calculate_cmd_forward():
   
def calculate_cmd_backward():


def callback(msg):
   global current_x, current_y
   current_x, current_y = msg.pose.pose.position.x, msg.pose.pose.position.y
   qx = msg.pose.pose.orientation.x
   qy = msg.pose.pose.orientation.y
   qz = msg.pose.pose.orientation.z
   qw = msg.pose.pose.orientation.w
   
   if (abs(waypoint[0,0]-current_x) < 0.1) and (abs(waypoint[1,0]-current_y) < 0.1):
       parking_flag = 'forward'
   elif (abs(waypoint[0,30]-current_x) < 0.1) and (abs(waypoint[1,30]-current_y) < 0.1):  #임의의 끝점
       parking_flag = 'backward'
       while(parking_complete 
     
def parking_select():
   if parking_flag == 'forward':
       calculate_cmd_forward()
   elif parking_flag == 'backward':
       calculate_cmd_backward()
   elif parking_flag == 'end':
       rospy.is_shutdown()  # parking end --> node exit
     
    
    
   
if __name__=='__main__':
   rospy.init_node('parking_gps')
   
   
   rospy.Subscriber("/odom",Odometry,callback)
  
   ###########################
   # gps.txt import
   ###########################
    
    
   rospy.spin()
    
