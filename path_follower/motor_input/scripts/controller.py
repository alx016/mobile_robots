#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from motor_input.msg import set_point

#Variables Globales
e  = 0
e1 = 0
Kp = 0.05
Ti = 0.002
Td = 0.0
T  = 0.02

motor_input	=  0 #dutyCycle
motor_output	= 0 #radians per second
s_point      	= 5  # radians per second (0:15) // desired speed

#Stop Condition
def stop():
	#Setup the stop message (can be the same as the control message)
	print("Stopping")

def motorOutput_callback(msg):
	global motor_output
	motor_output = msg.data

def setPoint_callback(setPoint_msg):
	global s_point
	s_point = setPoint_msg.set_point

def controller():
	global motor_input, e, e1
	e = s_point - motor_output
	motor_input = Kp*e + Ti*e1
	e1 += e
	if (motor_input > 1 ):
		motor_input = 1   

	elif (motor_input < 0):
		motor_input = 0
	return motor_input

if __name__=='__main__':

	#Initialise and Setup node
	rospy.init_node("controller")
	rate = rospy.Rate(100) #HZ
	#Setup Publishers and subscribers here
	motorInput_pub = rospy.Publisher("/motor_input", Float32, queue_size=1)
	rospy.Subscriber("/motor_output", Float32, motorOutput_callback)
	rospy.Subscriber("/set_point", set_point, setPoint_callback)

	#Run the node
	while not rospy.is_shutdown():
		motor_in = controller()
		motorInput_pub.publish(motor_in*255)
		rospy.loginfo("The input is: %f ",motor_in)
		rospy.loginfo("The output is: %f ",motor_output)
		rate.sleep()
