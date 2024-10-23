#!/usr/bin/env python

import cv2 as cv

class CSIFT():
    def __init__(self):
        self.__sift = cv.xfeatures2d.SIFT_create()
        pass

    def apply(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        keypoints = self.__sift.detect(gray, None)
        img = cv.drawKeypoints(gray, keypoints, img)

        return img