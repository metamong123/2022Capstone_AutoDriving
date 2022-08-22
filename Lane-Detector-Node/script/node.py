#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
VERBOSE = True


class LaneDetectionNode:
    def __init__(self, sub, detector):
        self.image_pub = rospy.Publisher("/output/lane_detection/compressed", CompressedImage, queue_size=1)
        self.steer_pub = rospy.Publisher("/assist_steer",Float64, queue_size=1)
        self.subscriber = rospy.Subscriber(sub, CompressedImage, self.callback, queue_size=1)
        self.detector = detector

    def callback(self, ros_data):
        # direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        lanes, shape = self.detector.get_lanes(image_np)
        if len(lanes) == 0:
                s_angle=0
                self.steer_pub.publish(s_angle)
	#print(lanes[0][:,0])
	#print(lanes[1])
	#print(len(lanes))

        lanes=np.sort(lanes,axis=0)[::-1]

        dist_cen=np.zeros(shape=(len(lanes),1))
        for i in range(len(lanes)):
                dist_cen[i]=np.average(lanes[i][:,0])-256
        flatten=dist_cen[:,0]
	#print(flatten)
        under_dist=flatten[np.where(dist_cen[:,0]<0)]
        upper_dist=flatten[np.where(dist_cen[:,0]>0)]
	#print(under_dist)
	#print(upper_dist)
	#print(np.where(dist_cen[:,0]==np.max(under_dist)))
	#print(np.where(dist_cen[:,0]==np.min(upper_dist)))
	#a=np.argpartition(dist_cen[:,0],-2)
        index0=np.where(dist_cen[:,0]==np.max(under_dist))
        index1=np.where(dist_cen[:,0]==np.min(upper_dist))

        my_lanes_0=lanes[index0]
        my_lanes_1=lanes[index1]
	#print(my_lanes_0[0][:,0])
	#print(1)

        center_x=np.float32((my_lanes_0[0][:,0]+my_lanes_1[0][:,0])/2)
        center_y=np.float32(lanes[0][:,1])	
        center_lane=np.array([center_x,center_y])
        s_angle=0.03*(center_x[0]-256)

	#print(center_lane[:,0])
	#print(len(center_lane[0,:]))
                
        mask_image = self.detector.draw_lanes(lanes, shape)
        mask_image = cv2.resize(mask_image,(1280,720),interpolation=cv2.INTER_LINEAR)
        mask_image = cv2.bitwise_or(mask_image,image_np)
        mask_image = cv2.line(mask_image,(640,300),(640,720),(255,255,255),thickness=2)
        mask_image = cv2.putText(mask_image, 'steering_angle : %0.1f'%(s_angle), (80,80),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
	
        #for i in range(len(center_lane[0,:])):
                #mask_image=cv2.circle(mask_image,(np.float32(center_lane[:,i][0]*1280/512),np.float32(center_lane[:,i][1]*720/256)),5,(0,0,0),-1)

        # Create CompressedIamge
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', mask_image)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        self.steer_pub.publish(s_angle)
