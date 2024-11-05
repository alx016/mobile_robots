#!/usr/bin/env python

import rospy
import cv2 as cv
import os
import numpy as np
import sys
import signal
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

img2 = Image()
bridge = CvBridge()
errorPub = rospy.Publisher('/lineFollowerError', Float32, queue_size=5)

def searcher(data, height, width):
    # This function allows the user to search where the black line begins and aux1 ==0 and aux2 ==0
    # and where it finishes 
    aux1 = 0
    aux2 = 0
    pt1 = 0
    pt2 = width-1
    height = height - 1
    for j in range (width):
        if(data[int(height/2)][j-1] == 0 and aux1 == 0):
            pt1 = j
            aux1 += 1
        if (data[int(height/2)][j-1] == 0):
            if(data[int(height/2)][j] > 127):
                pt2 = j
                aux2 += 1
    return pt1, pt2

def videoProcessing():   
    global errorPub                     
    while(1):
        try:
            img = bridge.imgmsg_to_cv2(img2, desired_encoding='passthrough')
            height, width = img.shape[:2]
            #print(f"{height} , {width}")

            #Initial and final desired height and width
            idh = int(height/4)*3
            idw = int(width/5)
            fdh = height
            fdw = int(idw*4)
             
            #Image cropping, first goes the height then the width
            img = img[idh:fdh, idw:fdw]
            # cv.imshow('result1',img)

            # Obtaining new values
            height, width = img.shape[:2]
            # print(f"{height} , {width}")

            # Convert BGR to BIN
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            blur = cv.blur(gray,(30,30),0)
            _ , bin = cv.threshold(blur,110,255, cv.THRESH_BINARY)
            #cv.imshow('bin',bin)
                    
            data = list(bin)

           # Detection of where the black part starts & ends
            pt1, pt2 = searcher(data, height, width)
            img = cv.line(img, (int(pt1), int(height/2)), (int(pt1), height), (255,0,0),2) # BLUE LINE
            img = cv.line(img, (int(pt2), int(height/2)), (int(pt2), height), (255,0,0),2) # BLUE LINE
            aux1 = (pt1 + pt2)/2
            # aux2 = (pt3 + pt4)/2
            img = cv.line(img, (int(aux1), int(height/2)), (int(width/2), height), (0,0,255),2) # RED
            img = cv.line(img, (int(width/2), int(height/2)), (int(width/2), height), (0,255,0),2) # GREEN

            # Calculating error
            ca = math.sqrt(((width/2)-(width/2))**2 + (height-0)**2)
            hipo = math.sqrt(((width/2)-int(aux1))**2 + (height-0)**2)
            theta = math.acos(ca/hipo)
            # theta = (math.acos(ca/hipo)*180)/math.pi # Radians to Degrees
            
            if (aux1 < (width/2)):
                theta *= -1
                # print("O =", theta)
            elif (aux1 > (width/2)):
                theta = theta
                # print("O =", theta)

            errorPub.publish(theta)

            # Show images
            #cv.imshow('result',img)
            #cv.waitKey(30)

            # Allows us to destroy the created windows with esc 
            signal.signal(signal.SIGTSTP, handler) # Detects when CTRL + Z is displayed and finishes the process
            signal.signal(signal.SIGINT, handler) # Detects when CTRL + C is displayed and finishes the process
        
        except CvBridgeError as e:
            print(e)

    return theta

def handler (signal_recieved, frame):
    print('EXIT')
    sys.exit(0)

def callback(msg):
	global img2
	img2 = msg


def main():
    rospy.init_node('lineFollower')
    rospy.Subscriber("/img", Image, callback)
    videoProcessing()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
