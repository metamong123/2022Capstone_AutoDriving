#!/usr/bin/env python

import argparse
import os
import sys
import numpy as np

# Set Python Path and Current Working Directory for Python
DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(DIR)
os.chdir(DIR)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Lane Detection")
    parser.add_argument("--detector", dest="detector", help="lanenet/scnn", default="lanenet", type=str)
    parser.add_argument("--subscriber", dest="subscriber", help="video/node", default='node', type=str)
    parser.add_argument("--name", dest="name", help="path to video or the node to be subscribed", default="/usb_cam/image_raw/compressed",type=str)
    parsed_args = parser.parse_args()
    assert parsed_args.detector.lower() in ["lanenet", "scnn"]
    assert parsed_args.subscriber.lower() in ["node", "video"]

    if parsed_args.detector.lower() == "lanenet":
        from detectors.lanenet_detector import LanenetLaneDetector
        detector = LanenetLaneDetector(y_range=[200, 256])
    else:
        from detectors.scnn_detector import SCNNDetector
        detector = SCNNDetector(y_range=[100, 288])

    if parsed_args.subscriber.lower() == "video":
        import cv2
        cap = cv2.VideoCapture(parsed_args.name)
        while cap.isOpened():
            ret, image = cap.read()
            image = cv2.resize(image,(512,256),interpolation=cv2.INTER_AREA) #800 288
            lanes, shape = detector.get_lanes(image)
            lanes=np.sort(lanes,axis=0)[::-1]
            mask_image = detector.draw_lanes(lanes, shape)
            mask_image = cv2.bitwise_or(mask_image,image)
            mask_image = cv2.resize(mask_image,(1280,720),interpolation=cv2.INTER_AREA)
            cv2.imshow('frame', mask_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                exit()
        cap.release()

    else:
        import rospy
        from node import LaneDetectionNode
        lane_detector_node = LaneDetectionNode(parsed_args.name, detector)
        rospy.init_node('Lane_Detector', anonymous=True)
	
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down ROS Lane Detector Module")
