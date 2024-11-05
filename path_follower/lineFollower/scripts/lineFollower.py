#!/usr/bin/env python

import rospy
import cv2 as cv
import os
import numpy as np
from std_msgs.msg import Float32

def searcher(data, height, width):
    # This function allows the user to search where the black line begins
    # and where it finishes 
    aux1 = 0
    pt1 = 0  # point 1
    pt2 = 0  # point 2
    height = height - 1
    for j in range (width):
            if(data[0][j] == 0 and aux1 == 0):
                pt1 = j
                aux1 += 1
            if(data[height][j-1] == 0):
                if (data[height][j]>127):
                    pt2 = j
    return pt1, pt2

def videoProcessing():
    cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'

    cap = cv.VideoCapture(cam_port)
    # Take each frame
    _, img = cap.read()
    height, width = img.shape[:2]
    #print(f"{height} , {width}")

    #Initial and final desired height and width
    idh = int(height/4)*3
    idw = int(width/4)
    fdh = height
    fdw = int(idw*3)

    while(1):
        # Take each frame
        _, img = cap.read()
        cv.imshow('org',img)
        #Image cropping, first goes the height then the width
        img = img[idh:fdh, idw:fdw]
        # cv.imshow('result1',img)

        # Obtaining new values
        height, width = img.shape[:2]
        # print(f"{height} , {width}")

        # Convert BGR to HSV
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        _ , bin = cv.threshold(gray, 120 ,255, cv.THRESH_BINARY)
        cv.imshow('bin',bin)        
        data = list(bin)

        # Detection of where the black part starts
        detectR, detectL = searcher(data, height, width)
        img = cv.line(img, (int(detectR), 0), (int(detectR), height), (255,0,0),2)
        img = cv.line(img, (int(detectL), 0), (int(detectL), height), (255,0,0),2)
        detect3 = (detectL + detectR)/2
        img = cv.line(img, (int(detect3), 0), (int(detect3), height), (0,0,255),2)
        img = cv.line(img, (int(width/2), 0), (int(width/2), height), (0,255,0),2) # GREEN

        # Calculating error
        error = detect3 - (width/2)
        # print(error)

        # Show images
        # cv.imshow('result',img)

        # Allows us to destroy the created windows with esc 
        if cv.waitKey(5) & 0xFF == ord('q'):
            break 
    cv.destroyAllWindows()
    return error


def main():
    rospy.init_node('errorTesting')
    errorPub = rospy.Publisher('/lineFollowerError', Float32, queue_size=5)
    error =  videoProcessing()
    errorPub.publish(error)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
