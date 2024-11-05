#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist


t = 0
dt = 0.01

if __name__=='__main__':

	#Initialise and Setup node
	rospy.init_node("puzzlebot_simulation")
	rate = rospy.Rate(100) #HZ
	#Setup Publishers and subscribers here
	pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

	#Run the node
	while not rospy.is_shutdown():
		twist = Twist()
		for i in range (4):
			t = 0
			while t < 30:
				while t < 8:
					twist.linear.x = 0.2
					pub.publish(twist)
					rospy.loginfo("FORWARD")
					t = t + dt
				while t < 9:
					twist.linear.x = 0
					pub.publish(twist)
					rospy.loginfo("STOP")
					t = t + dt	
				while t < 13:
					twist.angular.z = 0.2
					pub.publish(twist)
					rospy.loginfo("RIGHT")
					t = t + dt	
				while t < 21:
					twist.angular.z = 0
					pub.publish(twist)
					rospy.loginfo("STOP")
					t = t + dt
				t = t + dt	
			
		rospy.loginfo("FINISH")
		rate.sleep()
		exit()
