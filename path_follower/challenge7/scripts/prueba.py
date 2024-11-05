#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

wr = 0
wl = 0

#Callback functions
def callback_left(msg):
	global wl
	wl = msg.data

def callback_right(msg):
    global wr
    wr = msg.data

r = 0.05    #m. wheel radius
l = 0.19    #m. Distance between robots wheels

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("control")
    rate = rospy.Rate(100) #HZ
    #Setup Publishers and subscribers here
    rospy.Subscriber("/wl", Float32, callback_left)
    rospy.Subscriber("/wr", Float32, callback_right)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

	#Run the node
    while not rospy.is_shutdown():
        twist = Twist()
        dt = 0.01
        t = 0
        while (t < 3):
            v = 0.2
            twist.linear.x = v
            twist.angular.z = 0
            rospy.loginfo("v")
            rospy.loginfo(v)
            pub.publish(twist)
            rospy.loginfo("wr")
            rospy.loginfo(wr)
            rospy.loginfo("wl")
            rospy.loginfo(wl)
            v2 = r * ((wr + wl)/2)
            rospy.loginfo("v2")
            rospy.loginfo(v2)
            t = t + dt
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        
        w = r * ((wr - wl)/l)
        rospy.loginfo("FINISH")
        rate.sleep()
        exit()

