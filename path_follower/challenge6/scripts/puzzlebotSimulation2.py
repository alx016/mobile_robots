#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist

if __name__=='__main__':

	#Initialise and Setup node
	rospy.init_node("puzzlebot_simulation2")
	rate = rospy.Rate(100) #HZ
	#Setup Publishers and subscribers here
	pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

	#Run the node
	while not rospy.is_shutdown():
		twist = Twist()
		for i in range (4):
			t = 0.01		#time
			dt = 0.01		#diferential time
			d = 2.00		#distance
			O = np.pi / 3 	#theta
			while (t < 5):
				twist.linear.x = 0
				twist.angular.z = 0
				pub.publish(twist)
				t = t + dt
			while (t > 5 and t < 60):
				v = (d/t) * 0.15 #velocidad lineal
				if v > 0.2 :
					v = 0.2
				elif  v < 0.09:
					v = 0.12
				rospy.loginfo(v)
				# rospy.loginfo(t)
				twist.linear.x = v
				twist.angular.z = 0
				pub.publish(twist)
				t = t + dt
			twist.linear.x = 0
			twist.angular.z = 0
			t = 0.01
			while (t < 25):
				w = (O/t) * 0.2 #velocidad angular
				if w > 0.2 :
					w = 0.2
				elif w < 0.09:
					w = 0.19
				rospy.loginfo(w)
				# rospy.loginfo(t)
				twist.linear.x = 0
				twist.angular.z = w
				pub.publish(twist)
				t = t + dt
			twist.linear.x = 0
			twist.angular.z = 0
		pub.publish(twist)
		rospy.loginfo("FINISH")
		rate.sleep()
		exit()

