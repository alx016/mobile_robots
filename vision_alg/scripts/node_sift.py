#!/usr/bin/env python

import rospy
from sensor_msgs import Image
from cv_bridge import CvBridge, CvBridgeError
from sift import CSIFT

class NodeSIFT:
    def __init__(self):
        self.__bridge = CvBridge()
        self.__img_sub = rospy.Subscriber("/usb_cam/image_color", Image, self.img_callback)
        self.__img_pub = rospy.Publisher('/img_sift', Image, queue_size=1)

    def img_callback(self, data):
        try: 
            # cv_img = self.__bridge.compressed_imgmsg_to_cv2()
            cv_img = self.__bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as error:
            print(error)

        #algo proc!
        img_result = self.__sift.apply(cv_img)
        #self.__img_pub.publish(self.__bridge.cv2_to_compressed_imgmsg)
        self.__img_pub.publish(self.__bridge.cv2_to_imgmsg(img_result, 'bgr8'))

if __name__ == '__main__':
    rospy.init_node('node_sift', anonymous=True)
    node_handler = NodeSIFT()
    rospy.spin()