#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

global parking_flag

parking_flag = 'default'

save_speed=[]
save_angle=[]
save_accel=[]


cmd_Pub=rospy.Publisher('/ackermann_cmd_parking_backward',AckermannDriveStamped,queue_size=1)

def odom_callback(msg):
    global current_x, current_y
    current_x, current_y = msg.pose.pose.position.x, msg.pose.pose.position.y
    if (abs(waypoint[0,0]-current_x) < 0.1) and (abs(waypoint[1,0]-current_y) < 0.1):
        parking_flag = 'forward'
	else #########################해슬씨한테 현재 waypoint나오는부분 알려달라하기(거기서 parking_flag쏴주는게 좋을듯)
        parking_flag = 'forward'
        ##### parking_flag 를 forward, 
def cmd_callback(msg):
    global speed, angle, accel
    speed, angle, accel = msg.drive.speed, msg.drive.steering_angle, msg.drive.acceleration
    if parking_flag == 'forward':  #start save_cmd
        save_speed.append(speed)
        save_angle.append(angle)
        #save_accel.append(accel)
    elif parking_flag == 'backward':
        cmd.drive.speed = save_speed.pop()
        cmd.drive.steering_angle = save_angle.pop()
        cmd.drive.acceleration = -1 #backward gear
        if save_speed.len() == 0 and save_angle.len():
            rospy.set_param('car_mode', 'global')   #parking end
    cmd_Pub.publish(cmd)

    
     
if __name__=='__main__':
   global save_speed,save_angle,save_accel
   
   rospy.init_node('parking_backward')
   
   cmd=AckermannDriveStamped()
   
   rospy.Subscriber("/odom",Odometry,odom_callback)
   rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,cmd_callback)  #해슬씨한테 cmd값 토픽이름 바꿔달라하기.
    
   rospy.spin()
    