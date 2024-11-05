#!/usr/bin/env python
import rospy
import numpy as np
from challenge1.msg import signal_msg
from std_msgs.msg import Float32


#Global variable to store the data from the message in the /signal topic
signal_data = 0
time_data = 0
t = 0

# Example Callback Function (Hint)
def callback(msg):
    	global signal_data, time_data
    	signal_data = msg.signal_y
	time_data = msg.time_x

def callbacktime(time_msg):
    	global t
	time_data = time_msg.data


if __name__=='__main__':

   	rospy.init_node("process")
	
    	rospy.Subscriber(rospy.get_namespace() + "signal", signal_msg, callback)
	rospy.Subscriber("time", signal_msg, callbacktime)
    	
	signal2_pub = rospy.Publisher("proc_signal",Float32, queue_size = 10)
	A = 5
	F = 2
	rate = rospy.Rate(10)


    	while not rospy.is_shutdown():

		y = (signal_data - 5) * 0.5 + 5 + 0.5 * np.pi
		#y = (signal_data + A) * (np.sin(np.pi*t+F))
		#y = A * signal_data + F
		#y = A * np.sin(t*np.pi) + F
		rospy.loginfo(y) 
		signal2_pub.publish(y)
        	rate.sleep()
	
