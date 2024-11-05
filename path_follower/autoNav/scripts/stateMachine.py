#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32 
import time
import sys
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import signal

parametro=10
thetae = 0
wl = 0
wr = 0
v = 0.1
w = 0
kp = 0.65

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

def callback(msg):
    global parametro 
    parametro = msg.data

def callback_error(error):
	global thetae
	thetae = error.data
        
def lineFollowerCntrl():
    global w, v, pub
    time.sleep(0.1)
    w = 0
    w = thetae * -kp
    twist.linear.x = v
    twist.angular.z = w
    pub.publish(twist)

def handler (signal_recieved, frame):
    global pub
    print('EXIT')
    #Upload information
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)
    sys.exit(0)


if __name__ == '__main__':
   
    rospy.init_node('stateMachine')
    rate=rospy.Rate(10)
    rospy.Subscriber("/lineFollowerError", Float32, callback_error)
    rospy.Subscriber("/signals", Int32, callback)
    twist = Twist()
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    
    while True: 
        if parametro == 0:
            #Construccion
            twist.linear.x = 0.01
            #twist.angular.z= 0
            pub.publish(twist)
            #print ("aca ando")
            rospy.sleep(5)
        
        elif parametro == 1: 
            #Forward
            twist.linear.x = 0.02
            #twist.angular.z= 0
            pub.publish(twist)

        elif parametro ==2:
            #Green
            twist.linear.x = 0.02
            #twist.angular.z= 0
            pub.publish(twist)
           
        elif parametro ==3:
            #Giveaway
            twist.linear.x = 0.0
            twist.angular.z= 0.0
            pub.publish(twist)
            rospy.sleep(5)
            
        elif parametro ==4:
            #Left
            twist.linear.x = 0.02
            twist.angular.z= 0.0
            pub.publish(twist)
            rospy.sleep(8)
            twist.linear.x = 0.02
            twist.angular.z= 0.1
            pub.publish(twist)
            rospy.sleep(15)
            
        elif parametro ==5:
            #Red
            twist.linear.x = 0.0
            twist.angular.z= 0.0
            pub.publish(twist)

        elif parametro ==6:
            #Right
            twist.linear.x = 0.02
            twist.angular.z= 0.0
            pub.publish(twist)
            rospy.sleep(12)
            twist.linear.x = 0.02
            twist.angular.z= -0.1
            pub.publish(twist)
            rospy.sleep(15)
		
        elif parametro ==7:
            #Round
            twist.linear.x = 0.0
            twist.angular.z= 0.0
            pub.publish(twist)

        elif parametro == 8:
            #print("stop")
            twist.linear.x = 0.0
            twist.angular.z= 0.0
            pub.publish(twist)
            rospy.sleep(5)
           

        elif parametro == 9:
            #Yellow
            twist.linear.x = 0.01
            twist.angular.z= 0.0
            pub.publish(twist)

        else:
            #Nada
            print("lineFollower in action")
            lineFollowerCntrl()
        
        rospy.sleep(0.01)
        # Allows us to destroy the created windows with esc 
        signal.signal(signal.SIGTSTP, handler) # Detects when CTRL + Z is displayed and finishes the process
        signal.signal(signal.SIGINT, handler) # Detects when CTRL + C is displayed and finishes the process
    # rospy.spin()
    
