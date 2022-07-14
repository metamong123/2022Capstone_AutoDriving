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
parking_flag = 'default'
save_speed=[]
save_angle=[]
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
assist_steer=0

#class TopicReciver:
#	def __init__(self):
#		self.frenet_sub=rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,self.frenet_callback)
#		self.mode_sub=rospy.Subscriber("/mode_selector",StringArray,self.mode_selector_callback,queue_size=10)
#		self.YOLO_sub=rospy.Subscriber("/detect_ID", Int32MultiArray, self.yolo_callback)
#		self.lanenet_sub=rospy.Subscriber("/assist_steer",Float64,self.lanenet_callback)
#
#	def check_all_connections(self):
#		return (self.frenet_sub.get_num_connections()+self.mode_sub.get_num_connections()+self.park_back_sub.get_num_connections())==3
#		# return (self.frenet_sub.get_num_connections()+self.park_back_sub.get_num_connections()+self.mode_sub.get_num_connections()+self.YOLO_sub.get_num_connections()+self.lanenet_sub.get_num_connections())==5
#	
#	def mode_selector_callback(self,msg):
#		if self.check_all_connections():
#			global mode_selector, car_mode, move_mode, cur_dir_mode, next_dir_mode
#
#			mode_selector = msg.strings
#			car_mode = mode_selector[0]
#			move_mode = mode_selector[1]
#			cur_dir_mode = mode_selector[2]
#			next_dir_mode = mode_selector[3]
#			#print("car_mode = ",car_mode, "move_mode = ", move_mode, "cur_dir_mode = ", cur_dir_mode, "next_dir_mode = ", next_dir_mode)
#
#	def lanenet_callback(self,msg):
#		if self.check_all_connections():
#			global assist_steer
#			assist_steer = msg.data
#
#	def frenet_callback(self,msg):
#		if self.check_all_connections():
#			global frenet_speed, frenet_angle, frenet_gear
#			frenet_speed = msg.drive.speed
#			if assist_steer == 0:
#				frenet_angle = msg.drive.steering_angle  
#			else:
#				frenet_angle = assist_steer
#			frenet_gear = 0
#
#	def parking_callback(self,msg):
#		if self.check_all_connections():
#			global backward_speed, backward_angle, backward_gear, backward_brake
#			backward_speed = msg.drive.speed
#			backward_angle = msg.drive.steering_angle
#			backward_gear = msg.drive.acceleration
#			backward_brake = msg.drive.jerk
#
#	def yolo_callback(self,msg):
#		if self.check_all_connections():
#			global deliveryA, deliveryB, traffic_light, person, car, uturnsign, kidzonesign, parkingsign, stopline
#			deliveryA = msg.data[0]
#			deliveryB = msg.data[1]
#			traffic_light = msg.data[2]
#			person = msg.data[3]
#			car = msg.data[4]
#			uturnsign = msg.data[5]
#			kidzonesign = msg.data[6]
#			parkingsign = msg.data[7]
#			stopline = msg.data[8]
#
def mode_selector_callback(msg):
	global mode_selector, car_mode, move_mode, cur_dir_mode, next_dir_mode
    
	mode_selector = msg.strings
	car_mode = mode_selector[0]
	move_mode = mode_selector[1]
	cur_dir_mode = mode_selector[2]
	next_dir_mode = mode_selector[3]
	#print("car_mode = ",car_mode, "move_mode = ", move_mode, "cur_dir_mode = ", cur_dir_mode, "next_dir_mode = ", next_dir_mode)

def lanenet_callback(msg):
    global assist_steer
    assist_steer = msg.data

def frenet_callback(msg):
    global frenet_speed, frenet_angle, frenet_gear
    frenet_speed = msg.drive.speed
    frenet_angle = msg.drive.steering_angle  
    frenet_gear = 0

# def parking_callback(msg):
#     global backward_speed, backward_angle, backward_gear, backward_brake
#     backward_speed = msg.drive.speed
#     backward_angle = msg.drive.steering_angle
#     backward_gear = msg.drive.acceleration
#     backward_brake = msg.drive.jerk
   

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

def parking_decision():
	global parking_flag
	global back_speed, back_angle
	global save_speed,save_angle
	global move_mode, frenet_speed, frenet_angle, frenet_gear, backward_speed, backward_angle, backward_gear, backward_brake   
	global parking_angle, parking_brake, parking_speed, parking_gear
	if parking_flag == 'backward':
		if (len(save_speed) > 0) and (len(save_angle) > 0):
			parking_speed = save_speed.pop()
			parking_angle = save_angle.pop()
			parking_gear = 2 #backward gear
			parking_brake = 0
			#print('parking mode backward!!!')
			print(save_speed)
		if (len(save_speed) == 0) and (len(save_angle)==0):
			
			parking_flag == 'end'
	else:
		parking_speed = frenet_speed
		parking_angle = frenet_angle
		parking_gear = frenet_gear
		parking_brake = 0
		save_speed.append(frenet_speed) #save_cmd
		save_angle.append(frenet_angle)
		print(save_speed)
		#print('parking mode forward!!!')
	return parking_speed, parking_angle, parking_gear, parking_brake

def traffic_decision():
    
    if next_dir_mode == 'left':
        if traffic_light == 1 or traffic_light == 3:
            traffic_speed = frenet_speed
            traffic_angle = frenet_angle
            traffic_gear = frenet_gear
            traffic_brake = 0
        else :
            traffic_speed = 0
            traffic_angle = 0
            traffic_gear = 0
            traffic_brake = 200
    
    elif next_dir_mode == 'straight':
        if traffic_light == 0 or traffic_light == 3:
            traffic_speed = frenet_speed
            traffic_angle = frenet_angle
            traffic_gear = frenet_gear
            traffic_brake = 0
        else :
            traffic_speed = 0
            traffic_angle = 0
            traffic_gear = 0
            traffic_brake = 200
    
    elif next_dir_mode == 'right':
        if traffic_light == 0 or traffic_light == 3:
            traffic_speed = frenet_speed
            traffic_angle = frenet_angle
            traffic_gear = frenet_gear
            traffic_brake = 0
        else :
            traffic_speed = 0
            traffic_angle = 0
            traffic_gear = 0
            traffic_brake = 200   
                
    traffic_speed = frenet_speed
    traffic_angle = frenet_angle
    traffic_gear = frenet_gear
    traffic_brake = 0  

    return traffic_speed, traffic_angle, traffic_gear, traffic_brake

if __name__=='__main__':

	rospy.init_node('core_control')
	#topic_receiver=TopicReciver()
	rospy.Subscriber("/ackermann_cmd_frenet",AckermannDriveStamped,frenet_callback)
	#rospy.Subscriber("/ackermann_cmd_parking_backward",AckermannDriveStamped,parking_callback)
	rospy.Subscriber("/mode_selector",StringArray,mode_selector_callback,queue_size=10)
	rospy.Subscriber("/detect_ID", Int32MultiArray, yolo_callback)
	rospy.Subscriber("/assist_steer",Float64,lanenet_callback)
	cmd=AckermannDriveStamped()
	final_cmd_Pub = rospy.Publisher('/ackermann_cmd',AckermannDriveStamped,queue_size=1)
	while not rospy.is_shutdown():
		if car_mode == 'global':
			if move_mode == 'finish':
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = traffic_decision()
			else:
				if assist_steer == 0:
					cmd.drive.speed = frenet_speed
					cmd.drive.steering_angle = frenet_angle
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 0
				else:
					cmd.drive.speed = frenet_speed
					cmd.drive.steering_angle = assist_steer
					cmd.drive.acceleration = frenet_gear
					cmd.drive.jerk = 0
				#print('global mode!!!')

		elif car_mode == 'parking':
			if move_mode == 'forward': # parking forward -> frenet
				cmd.drive.speed, cmd.drive.steering_angle, cmd.drive.acceleration, cmd.drive.jerk = parking_decision()
			elif move_mode == 'finish':
				cmd.drive.speed = 0
				cmd.drive.steering_angle = 0
				cmd.drive.acceleration = 0
				cmd.drive.jerk = 200  #full brake
				final_cmd_Pub.publish(cmd)
				print('parking finish!!! stop!!')
				rospy.sleep(5) # 5sec
				parking_flag = 'backward'
			final_cmd_Pub.publish(cmd)
		#print(parking_flag)
		rospy.sleep(0.1)
		final_cmd_Pub.publish(cmd)



