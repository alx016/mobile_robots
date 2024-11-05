#!/usr/bin/env python

import rospy
import cv2 as cv
import sys
import signal
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# img = cv.imread('/home/alex/catkin_ws/src/camera/scripts/linea.jpeg')
cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'

def handler (signal_recieved, frame):
    print('EXIT')
    sys.exit(0)

def main():
    rospy.init_node('imageCapture')
    image_publisher = rospy.Publisher('/img', Image, queue_size=10)

    cap = cv.VideoCapture(cam_port)
    if not cap.isOpened():
        rospy.logerr("Failed to open the camera")
        return
    
    while(1):
        # Take each frame
        _, img = cap.read()

        # cv.imshow('re',img)
        # cv.waitKey(3)

        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        
        # rospy.loginfo(img_msg)
        image_publisher.publish(img_msg)

        signal.signal(signal.SIGTSTP, handler)  # Detects when CTRL + Z is displayed and finishes the process
        signal.signal(signal.SIGINT, handler)   # Detects when CTRL + C is displayed and finishes the process
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
