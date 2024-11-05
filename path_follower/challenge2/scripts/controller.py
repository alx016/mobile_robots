#!/usr/bin/env python
import rospy
import numpy as np
from challenge2.msg import motor_output
from challenge2.msg import motor_input
from challenge2.msg import set_point

#Variables Globales
e = 0
e_prev = 0
Kp = rospy.get_param("/Kp",0)
Ki = rospy.get_param("/Ki",0)
Kd = rospy.get_param("/Kd",0)
k = rospy.get_param("/k",2)
Ts = rospy.get_param("/Ts",2) 

output_data = 0
set_point_v = 0

#Setup parameters, vriables and callback functions here (if required)
def output_callback(msg):
	global output_data, time, status
	output_data = msg.output
	time = msg.time
	status = msg.status

def set_point_callback(set_point_msg):
	global set_point_v, t_set_point
	set_point_v = set_point_msg.set_point
	t_set_point = set_point_msg.time

#Stop Condition
def stop():
	#Setup the stop message (can be the same as the control message)
	print("Stopping")

def controller():
	global e, e_prev, motor_input
	e = set_point_v - output_data
	motor_input = Kp*e + Ki*Ts*(e + Kd*((e-e_prev)/Ts))
	e_prev = e



if __name__=='__main__':
	#Initialise and Setup node
	rospy.init_node("controller")
	rate = rospy.Rate(100)
	rospy.on_shutdown(stop)

	#Setup Publishers and subscribers here
	rospy.Subscriber("/motor_output", motor_output, output_callback)
	rospy.Subscriber("/set_point", set_point, set_point_callback)
	motorinput_pub = rospy.Publisher("/motor_input", motor_input, queue_size=1)
	
	msg = motor_input()
	init_time = rospy.get_time()
	print("The Controller is Running")
	
	#Run the node
	while not rospy.is_shutdown():
		controller()
		t = rospy.get_time() - init_time
		msg.input = motor_input
		msg.time = t
		motorinput_pub.publish(msg)
		rospy.loginfo("The input is: %f at a time %f",motor_input, t)		

		rate.sleep()
