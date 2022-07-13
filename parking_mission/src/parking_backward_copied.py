#!/usr/bin/env python

from shutil import move
from turtle import backward
import rospy
import numpy as np
from rocon_std_msgs.msg import StringArray 
from ackermann_msgs.msg import AckermannDriveStamped


save_speed=[]
save_angle=[]
save_accel=[]
car_mode='default'
move_mode='default'

cmd_Pub=rospy.Publisher('/ackermann_cmd_parking_backward',AckermannDriveStamped,queue_size=1)

def mode_selector_callback(msg):
    global mode_selector, car_mode, move_mode, cur_dir_mode, next_dir_mode

    mode_selector = msg.strings
    car_mode = mode_selector[0]
    move_mode = mode_selector[1]
    cur_dir_mode = mode_selector[2]
    next_dir_mode = mode_selector[3]
    print("car_mode = ",car_mode, "move_mode = ", move_mode, "cur_dir_mode = ", cur_dir_mode, "next_dir_mode = ", next_dir_mode)

def cmd_callback(msg):
    global speed, angle, accel
    global save_speed,save_angle,save_accel
    global move_mode,car_mode
    speed, angle, accel = msg.drive.speed, msg.drive.steering_angle, msg.drive.acceleration
    if car_mode == 'parking' and move_mode == 'forward':  #start save_cmd
        save_speed.append(speed)
        save_angle.append(angle)
        #save_accel.append(accel)
    elif car_mode == 'parking' and move_mode == 'backward':
        cmd.drive.speed = save_speed.pop()
        cmd.drive.steering_angle = save_angle.pop()
        cmd.drive.acceleration = 2 #backward gear
        cmd.drive.jerk = 0
        if (len(save_speed) == 0) and (len(save_angle)==0):
            print(1)

    if car_mode == 'parking' and move_mode == 'finish':
        cmd.drive.speed = 0
        cmd.drive.steering_angle = 0
        cmd.drive.acceleration = 0
        cmd.drive.jerk = 200  #full brake
        cmd_Pub.publish(cmd)
        rospy.sleep(0.2)  # 5sec
        move_mode= 'backward'
    cmd_Pub.publish(cmd)

     
if __name__=='__main__':

   
   rospy.init_node('parking_backward')
   
   cmd=AckermannDriveStamped()
   
   rospy.Subscriber("/mode_selector",StringArray,mode_selector_callback)
   rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,cmd_callback)

   rospy.spin()