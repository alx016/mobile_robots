#!/usr/bin/env python
# coding=utf-8
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time
import math


#Callback functions
def callback_left(msg):
	global wl
	wl = msg.data

def callback_right(msg):
    global wr
    wr = msg.data

# Constants & Variables
r = 0.05    # m. wheel radius
l = 0.19    # m. Distance between robots wheels

wr = 0  # right angular speed
wl = 0  # left angular speed

x = 0       # Initial position
y = 0       # Initial position
theta = 0   # Angular position 

#Target positions
xt = [2, 2, 0, 0]      # Wanted position on x
yt = [0, 2, 2, 0]      # Wanted position on y
d = math.sqrt((xt[0] - x)**2 + (yt[0] - y)**2)       # Distance between target and actual position 

init_time = 0   # Time initialization
dt = 0          # Time differential

vmax = 0.4
kv = 1
kw = 1

if __name__=='__main__':

    # Initialise and Setup node
    rospy.init_node("control")
    rate = rospy.Rate(100) #HZ
    # Setup Publishers and subscribers here
    rospy.Subscriber("/wl", Float32, callback_left)
    rospy.Subscriber("/wr", Float32, callback_right)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
	# Run the node
    while not rospy.is_shutdown():
        twist = Twist()
        for i in range(len(xt)):
            rospy.loginfo("Round " +str(i+1))
            d = math.sqrt((xt[i] - x)**2 + (yt[i] - y)**2)       # Distance between target and actual position 
            while d > 0.2: 
                init_time = time.time()
                time.sleep(0.01)
                #rate.sleep()    # Short delay to let settle
                dt = time.time() - init_time

                d = math.sqrt((xt[i] - x)**2 + (yt[i] - y)**2)     # Distance between actual and desired position 
                thetae = theta - np.arctan2((yt[i] - y), xt[i] - x) # Angle between actual and desired angle
                if(thetae > math.pi):
                    thetae = thetae - 2*math.pi
                elif(thetae < -math.pi):
                    thetae = thetae + 2*math.pi
                #rospy.loginfo("differential time")
                #rospy.loginfo(dt)

                vR = r * ((wr + wl)/2)  # Linear velocity rad/seg
                wR = r * ((wr - wl)/l)  # Angular velocity 

                # rospy.loginfo("velocity")
                # rospy.loginfo(vR)   # Calculated velocity using encoders
                # rospy.loginfo("angular velocity")
                # rospy.loginfo(wR)   # Calculated angular velocity using encoders

                theta += wR * dt            # Calculated theta
                x += vR * math.cos(theta) * dt  # Calculated position in x
                y += vR * math.sin(theta) * dt  # Calculated position in y


                v = vmax * math.tanh(kv * d)   # Calculated velocity
                w = -kw *math.tanh(thetae)          # Calculated angular velocity
                if v > 0.2:
                    v = 0.2
                if w > 2:
                    w = 2

                #rospy.loginfo("distance")
                #rospy.loginfo(d)
                # rospy.loginfo("input velocity")
                # rospy.loginfo(v)
                # rospy.loginfo("input angular velocity")
                # rospy.loginfo(w)
                #rospy.loginfo("VR "+ str(vR))

                #Upload information
                twist.linear.x = v
                twist.angular.z = w
                pub.publish(twist)     
                # Podría necesitar spinOnce
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        rospy.loginfo("FINISH")
        init_time = rospy.get_time()
        exit()

