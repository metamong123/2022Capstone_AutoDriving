#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped

save_speed=[]
save_angle=[]
save_accel=[]

cmd_Pub=rospy.Publisher('/ackermann_cmd_parking_backward',AckermannDriveStamped,queue_size=1)

def cmd_callback(msg):
    global speed, angle, accel
    speed, angle, accel = msg.drive.speed, msg.drive.steering_angle, msg.drive.acceleration
    parking_flag = rospy.get_param('parking_flag')
    car_mode = rospy.get_param('car_mode')
    if parking_flag == 'forward':  #start save_cmd
        save_speed.append(speed)
        save_angle.append(angle)
        #save_accel.append(accel)
    elif parking_flag == 'backward':
        cmd.drive.speed = save_speed.pop()
        cmd.drive.steering_angle = save_angle.pop()
        cmd.drive.acceleration = 2 #backward gear
        cmd.drive.jerk = 0
        if save_speed.len() == 0 and save_angle.len():
            rospy.set_param('car_mode', 'global')   #parking end
    if car_mode == 'parking' and _________ == 'finish':
        cmd.drive.speed = 0
        cmd.drive.steering_angle = 0
        cmd.drive.acceleration = 0
        cmd.drive.jerk = 200  #full brake
        r = rospy.Rate(4)  # 4sec
        r.sleep()
        rospy.set_param('parking_flag','backward')
    cmd_Pub.publish(cmd)

     
if __name__=='__main__':
   global save_speed,save_angle,save_accel
   
   rospy.init_node('parking_backward')
   
   cmd=AckermannDriveStamped()
   
   rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,cmd_callback)

   rospy.spin()