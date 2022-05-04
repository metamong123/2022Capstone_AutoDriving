#!/usr/bin/env python

import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def callback(msg):

	######rosimage topic to cv_image for image processing#######
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
	height,width,co = cv_image.shape
	#print(cv_image.shape)

	gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
	triangle = np.array([[(0,height),(200,225),(500,225),(width,height)]])
	gray_mask = np.zeros_like(gray)
	gray_mask = cv2.fillPoly(gray_mask,triangle,(255,255,255))
	gray_mask = cv2.bitwise_and(gray,gray_mask)	

	blur = cv2.GaussianBlur(gray,(5,5),0)

	edge = cv2.Canny(cv_image,10,50)


	######masking for ROI#############
	
	triangle = np.array([[(0,height),(200,225),(500,225),(width,height)]])
	mask = np.zeros_like(cv_image)
	mask = cv2.fillPoly(mask,triangle,(255,255,255))
	mask = cv2.bitwise_and(cv_image,mask)

	
	#####color filtering for yellow & white lane#############
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	lower_yellow = np.array([20,100,100],dtype="uint8")
	upper_yellow = np.array([30,255,255],dtype="uint8")

	mask_yellow = cv2.inRange(hsv,lower_yellow,upper_yellow)
	mask_white = cv2.inRange(gray_mask,200,255)

	mask_yw = cv2.bitwise_or(mask_yellow,mask_white)
	mask_yw_image = cv2.bitwise_and(gray_mask,mask_yw)
 
	print(mask_white)

	ros_image_p = bridge.cv2_to_imgmsg(gray,"mono8")
	processing_img_pub=rospy.Publisher("/processing_image",Image,queue_size=1)
	processing_img_pub.publish(ros_image_p)


	#####lane detection for straight lane#######
	lines = cv2.HoughLinesP(edge, 1, np.pi/180, 10, None, 80, 1)
	if lines is not None:
			
		for line in lines:
			x1,y1,x2,y2=line[0]
			line_on_img  = cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),3)

			######cv_image to rosimage for topic publish#########	
			ros_image = bridge.cv2_to_imgmsg(line_on_img,"bgr8")
			lane_img_pub=rospy.Publisher("/lane_image",Image,queue_size=1)
			lane_img_pub.publish(ros_image)

	#####circle detection for curved lane######
	circles = cv2.HoughCircles(edge,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=100,maxRadius=120)
	if circles is not None:

		#print(circles)
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			circle_on_img = cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),3)


			######cv_image to rosimage for topic publish#########	
			ros_image_c = bridge.cv2_to_imgmsg(circle_on_img,"bgr8")
			circle_img_pub=rospy.Publisher("/circle_image",Image,queue_size=1)
			circle_img_pub.publish(ros_image_c)

		
if __name__=='__main__':

	rospy.init_node('lane_detector')


	#####rosrun usb_cam usb_cam_node########
	rospy.Subscriber("/usb_cam/image_raw",Image,callback)

	rospy.spin()
