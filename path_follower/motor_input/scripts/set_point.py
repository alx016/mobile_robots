#!/usr/bin/env python

import rospy
import numpy as np
from motor_input.msg import set_point


choice = rospy.get_param("/choice",0)

class SignalGenerator:
    def __init__(self):
        self.set_point = rospy.Publisher('/set_point', set_point, queue_size=10)
        #self.pub_step = rospy.Publisher('step_signal', Float64, queue_size=10)
        #self.pub_square = rospy.Publisher('square_signal', Float64, queue_size=10)
        
    def sin_signal(self, frequency=1.0):
        rate = rospy.Rate(10) # Hz
        t = 0.0
        while not rospy.is_shutdown():
            amplitude = rospy.get_param("/amplitude",0)
            sin_value = 5 + amplitude * np.sin(2 * np.pi * frequency * t)
            self.set_point.publish(sin_value)
            t += 0.1
            rate.sleep()

    def step_signal(self):
        rate = rospy.Rate(10)
        step_time = 5  # seconds
        amplitude = rospy.get_param("/amplitude",0)
        signal_value = amplitude  # amplitude
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            elapsed_time = current_time - start_time
            if elapsed_time <= step_time:
                signal_value = 0
            else:
                signal_value = 10

            self.set_point.publish(signal_value)

            rate.sleep()

    def square_signal(self, frequency=0.25):
        rate = rospy.Rate(10) # Hz
        t = 0.0
        while not rospy.is_shutdown():
            amplitude = rospy.get_param("/amplitude",0)
            square_value = 10 * amplitude if (t % (1.0 / frequency)) < (0.5 / frequency) else amplitude*2
            self.set_point.publish(square_value)
            t += 0.1
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('signal_generator')
    signal_generator = SignalGenerator()
    menu = """
    Please select a signal to plot:
    1. Sin signal
    2. Step signal
    3. Square signal
    """
    choice = raw_input(menu)
    if choice == '1':
        signal_generator.sin_signal()
        rospy.spin()
    elif choice == '2':
        signal_generator.step_signal()
        rospy.spin()
    elif choice == '3':
        signal_generator.square_signal()
        rospy.spin()
    else:
        print("Invalid choice")
