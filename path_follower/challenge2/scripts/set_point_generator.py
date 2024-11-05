#!/usr/bin/env python
import rospy
import numpy as np
from challenge2.msg import set_point

# Setup Variables, parameters and messages to be used (if required)

x = rospy.get_param("/SetPoint", 1.0)

#Stop Condition
def stop():
	#Setup the stop message (can be the same as the control message)
	print("Stopping")


if __name__=='__main__':
	#Initialise and Setup node
	rospy.init_node("Set_Point_Generator")
	rate = rospy.Rate(100)
	rospy.on_shutdown(stop)

	#Setup Publishers and subscribers here
	setpoint_pub = rospy.Publisher("/set_point",set_point, queue_size=1)
	msg = set_point()
	print("The Set Point Genertor is Running")
	init_time = rospy.get_time()
	#Run the node
	while not rospy.is_shutdown():
		t = rospy.get_time() - init_time	
		y = np.sin(t) * x * np.pi
		msg.time = t
		msg.set_point = y
		setpoint_pub.publish(msg)
		rospy.loginfo("The Set Point is: %f at a time %f", y, t)
		rate.sleep()
