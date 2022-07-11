#!/usr/bin/env python

from operator import ne
import rospy
from tokenize import String
from std_msgs.msg import Int32MultiArray, Float64
from rocon_std_msgs.msg import StringArray 
from ackermann_msgs.msg import AckermannDriveStamped

global car_mode, move_mode
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

def mode_selector_callback(msg):
    global mode_selector, car_mode, move_mode, cur_dir_mode, next_dir_mode

    mode_selector = msg.strings
    car_mode = mode_selector[0]
    move_mode = mode_selector[1]
    cur_dir_mode = mode_selector[2]
    next_dir_mode = mode_selector[3]
    print("car_mode = ",car_mode, "move_mode = ", move_mode, "cur_dir_mode = ", cur_dir_mode, "next_dir_mode = ", next_dir_mode)

def lanenet_callback(msg):
    global assist_steer
    assist_steer = msg.data

def frenet_callback(msg):
    global frenet_speed, frenet_angle, frenet_gear
    frenet_speed = msg.drive.speed
    if assist_steer == 0:
        frenet_angle = msg.drive.steering_angle  
    else:
        frenet_angle = assist_steer
    frenet_gear = 0

def parking_callback(msg):
    global backward_speed, backward_angle, backward_gear, backward_brake
    backward_speed = msg.drive.speed
    backward_angle = msg.drive.steering_angle
    backward_gear = msg.drive.acceleration
    backward_brake = msg.drive.jerk
   
def parking_decision():
    if move_mode == 'forward':    # parking forward -> frenet
        parking_speed = frenet_speed
        parking_angle = frenet_angle
        parking_gear = frenet_gear
        parking_brake = 0
        print('parking mode forward!!!')
    elif move_mode == 'finish':
        parking_speed = backward_speed
        parking_angle = backward_angle
        parking_gear = backward_gear
        parking_brake = 200 #backward_brake
        print('parking finish!!! stop!!')
    elif move_mode == 'backward':
        parking_speed = backward_speed
        parking_angle = backward_angle
        parking_gear = backward_gear
        parking_brake = backward_brake
        print('parking mode backward!!!')
    return parking_speed, parking_angle, parking_gear, parking_brake

def yolo_callback(msg):
    global deliveryA, deliveryB, traffic_light, person, car, uturnsign, kidzonesign, parkingsign, stopline
    deliveryA = msg.data[0]
    deliveryB = msg.data[1]
    traffic_light = msg.data[2]
    person = msg.data[3]
    car = msg.data[4]
    uturnsign = msg.data[5]
    kidzonesign = msg.data[6]
    parkingsign = msg.data[7]
    stopline = msg.data[8]

def traffic_decision():
    
    # if next_dir_mode == 'left':
    #     if traffic_light == 1 or traffic_light == 3:
    #         traffic_speed = frenet_speed
    #         traffic_angle = frenet_angle
    #         traffic_gear = frenet_gear
    #         traffic_brake = 0
    #     else :
    #         traffic_speed = 0
    #         traffic_angle = 0
    #         traffic_gear = 0
    #         traffic_brake = 200
    
    # elif next_dir_mode == 'straight':
    #     if traffic_light == 0 or traffic_light == 3:
    #         traffic_speed = frenet_speed
    #         traffic_angle = frenet_angle
    #         traffic_gear = frenet_gear
    #         traffic_brake = 0
    #     else :
    #         traffic_speed = 0
    #         traffic_angle = 0
    #         traffic_gear = 0
    #         traffic_brake = 200
    
    # elif next_dir_mode == 'right':
    #     if traffic_light == 0 or traffic_light == 3:
    #         traffic_speed = frenet_speed
    #         traffic_angle = frenet_angle
    #         traffic_gear = frenet_gear
    #         traffic_brake = 0
    #     else :
    #         traffic_speed = 0
    #         traffic_angle = 0
    #         traffic_gear = 0
    #         traffic_brake = 200       
    traffic_speed = frenet_speed
    traffic_angle = frenet_angle
    traffic_gear = frenet_gear
    traffic_brake = 0  

    return traffic_speed, traffic_angle, traffic_gear, traffic_brake

if __name__=='__main__':

    rospy.init_node('core_control')

    rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
    rospy.Subscriber("/ackermann_cmd_parking_backward",AckermannDriveStamped,parking_callback)
    rospy.Subscriber("/mode_selector",StringArray,mode_selector_callback,queue_size=5)
    rospy.Subscriber("/detect_ID", Int32MultiArray, yolo_callback)
    rospy.Subscriber("/assist_steer",Float64,lanenet_callback)
    cmd=AckermannDriveStamped()

    final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)

    while not rospy.is_shutdown():

        if car_mode == 'global':
            if move_mode == 'finish':
                cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
            else:
                cmd.drive.speed = frenet_speed
                cmd.drive.steering_angle = frenet_angle
                cmd.drive.acceleration = frenet_gear
                cmd.drive.jerk = 0
                #print('global mode!!!')

        elif car_mode == 'parking':
            cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()


        final_cmd_Pub.publish(cmd)

