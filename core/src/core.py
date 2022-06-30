#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

global frenet_speed, frenet_angle, frenet_gear
global backward_speed, backward_angle, backward_gear, backward_brake

global car_mode, parking_flag, finish
carmode = 'default'
parking_flag = 'default'
finish = 'default'

def frenet_callback(msg):   
   frenet_speed = msg.drive.speed
   frenet_angle = msg.drive.steering_angle  
   frenet_gear = 0

def parking_callback(msg):
   backward_speed = msg.drive.speed
   backward_angle = msg.drive.steering_angle
   backward_gear = msg.drive.acceleration
   backward_brake = msg.drive.jerk
   
def parking_decision():
   if parking_flag == 'forward':    # parking forward -> frenet
      if finish == 'finish':
         parking_speed = backward_speed
         parking_angle = backward_angle
         parking_gear = backward_gear
         parking_brake = backward_brake
         print('parking finish!!! stop!!')
      else:
         parking_speed = frenet_speed
         parking_angle = frenet_angle
         parking_gear = frenet_gear
         parking_brake = 0
         print('parking mode forward!!!')
   elif parking_flag == 'backward':
      parking_speed = backward_speed
      parking_angle = backward_angle
      parking_gear = backward_gear
      parking_brake = backward_brake
      print('parking mode backward!!!')
   return parking_speed, parking_angle, parking_gear, parking_brake

def traffic_decision():
   ########## need traffic light value #########
   #elif car_mode == 'global' and _________ == 'finish':
   #   if 
   ##################################

if __name__=='__main__':
   
   rospy.init_node('core_control')

   rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
   rospy.Subscriber('/ackermann_cmd_parking_backward',AckermannDriveStamped,parking_callback)

   cmd=AckermannDriveStamped()

   final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)

   while not rospy.is_shutdown():
      car_mode = rospy.get_param('car_mode')
      parking_flag = rospy.get_param('parking_flag')
      finish = rospy.get_param('finish')

      if car_mode == 'global':
         if finish == 'not finish':
            cmd.drive.speed = frenet_speed
            cmd.drive.steering_angle = frenet_angle
            cmd.drive.acceleration = frenet_gear
            cmd.drive.jerk = 0
            print('global mode!!!')
         elif finish == 'finish':
            traffic_desicion()

      elif car_mode == 'parking':
         cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()


      final_cmd_Pub.publish(cmd)