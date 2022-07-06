#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32MultiArray, Float64

global car_mode, move_mode, reduce_speed
car_mode = 'default'
move_mode = 'default'
frenet_speed = 0
frenet_angle = 0 
frenet_gear = 0
backward_speed = 0
backward_angle = 0
backward_gear = 0
backward_brake = 0
deliveryA = 0
deliveryB = 0
traffic_light = 0 
person = 0 
car = 0
uturnsign = 0
kidzonesign = 0
parkingsign = 0
stopline = 0
lanenet_steer = 0 
reduce_speed = 0

def frenet_callback(msg):   
   global frenet_speed, frenet_angle, frenet_gear
   frenet_speed = msg.drive.speed
   frenet_angle = msg.drive.steering_angle  
   frenet_gear = 0
   
def lanenet_callback(msg):
   global lanenet_steer
   lanenet_steer = msg.data

def yolo_callback(msg):
   global deliveryA, deliveryB, traffic_light, person, car, uturnsign, kidzonesign, parkingsign, stopline
   deliveryA = msg.data[0] # x
   deliveryB = msg.data[1] # x
   traffic_light = msg.data[2]
   person = msg.data[3] # x
   car = msg.data[4] # x
   uturnsign = msg.data[5] # x
   kidzonesign = msg.data[6]
   parkingsign = msg.data[7] # x 고민된다
   stopline = msg.data[8]  

def traffic_decision():
   if traffic_light == 0: # green or None
       traffic_speed = frenet_speed - reduce_speed
       traffic_angle = frenet_angle
       traffic_gear = frenet_gear
       traffic_brake = 0
   elif traffic_light == 1: # left
       traffic_speed = frenet_speed - reduce_speed
       traffic_angle = frenet_angle
       traffic_gear = frenet_gear
       traffic_brake = 0
   elif traffic_light == 2 or traffic_light == 4: # red and yellow
       traffic_speed = 0
       traffic_angle = 0
       traffic_gear = 0
       traffic_brake = 50  # ê¸‰ë¸Œë ˆì´í¬í• ì§€ ë§ì§€ ê³ ë¯¼..
   elif traffic_light == 3: # straightleft
       traffic_speed = frenet_speed - reduce_speed
       traffic_angle = frenet_angle
       traffic_gear = frenet_gear
       traffic_brake = 0

   return traffic_speed, traffic_angle, traffic_gear, traffic_brake



if __name__=='__main__':

   before = rospy,Time.now() #virtual
   dt = 0

   rospy.init_node('core_control')

   rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
   rospy.Subscriber("/detect_ID", Int32MultiArray, yolo_callback)
   rospy.Subscriber("/assist_steer", Float64, lanenet_callback)

   cmd=AckermannDriveStamped()

   final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)

   while not rospy.is_shutdown():
      car_mode = rospy.get_param('car_mode')
      move_mode = rospy.get_param('move_mode')

      if stopline == 1:
         cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
      else:
         if kidzonesign == 1:
            reduce_speed = 3 # 임의로 줄임

         cmd.drive.speed = frenet_speed - reduce_speed  # school zone
         cmd.drive.steering_angle = frenet_angle + lanenet_steer  # extra lanenet
         cmd.drive.acceleration = frenet_gear
         cmd.drive.jerk = 0
         print('global mode!!!')

      dt = (rospy.Time.now() - before) + dt  # virtual
      before = rospy,Time.now() # virtual
      if dt > 50:
         reduce_speed = 0  #일정시간 지나면 school zone 종료 (virtual하게끔 하기 위해 시간으로 함.)
      
      final_cmd_Pub.publish(cmd)

      