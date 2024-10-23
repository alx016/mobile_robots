#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class DepthNode:
    def __init__(self):
        self.__bridge = CvBridge()
        self.__img_sub = rospy.Subscriber("/camera/depth/image", Image, self.img_callback)
        self.__cam_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cam_info_callback)
        self.__matrix_cam = None

    def cam_info_callback(self, msg):
        self.__matrix_cam = np.asarray(msg.K).reshape(3, 3)

    def img_callback(self, msg):
        try:
            cv_img = self.__bridge.imgmsg_to_cv2(msg, "uint16")
            if self.__matrix_cam != None:
                self.reproject(cv_img, self.__matrix_cam)
        
        except CvBridgeError as error:
            print(error)

    def reproject(self, img, cam_matrix):
        height, width = img.shape

        for row in range(height):
            for col in range(width):
                depth = img[row, col]

    def valid_depth(self, depth):
        meters = depth / 1000

        if meters < 0.3 or meters > 30:
            return None
        return meters

if __name__ == '__main__':
    rospy.init_node("depth", anonymous=True)
    node_handler = DepthNode()
    rospy.spin()