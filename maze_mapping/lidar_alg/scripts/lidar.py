#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NodeLIDAR:
    def __init__(self):
        # rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/key', Bool, self.key_callback)
        self.__vel_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
        
        self.a_vel  =    rospy.get_param("a_vel/val", default = 0.2)              #rotational velocity
        self.l_vel  =    rospy.get_param("l_vel/val", default = 0.1)              #rotational velocity
        self.kp     =    rospy.get_param("control/kp", default = 0.1)
        self.min_dist =  rospy.get_param("distance/min_dist", default = 0.4)          #minimum detected distance

        self.msg = Twist()
        self.key = True

        self.rate = rospy.Rate(30)
    
    def adjust_velocity(self, x_vel, y_vel, angular_vel):
        # Apply speed limit to linear velocity
        self.msg.linear.x = x_vel
        self.msg.linear.y = y_vel
        self.msg.angular.z = angular_vel
        self.__vel_pub.publish(self.msg)
        self.rate.sleep()

    def move_forward(self, forward_distance, y_vel):
        if forward_distance > self.min_dist:
            self.l_vel
            self.adjust_velocity(self.l_vel, y_vel, 0)

    def turn_left(self):
        self.adjust_velocity(0, 0, self.a_vel)

    def turn_right(self):
        x_vel = 0.12   
        w = x_vel/0.30
        self.adjust_velocity(x_vel, 0, -w)
    
    def stop(self):
        self.adjust_velocity(0, 0, 0)

    def key_callback(self, data):
        self.key = data

    def lidar_callback(self, data):
        forward_distance =  min(data.ranges[0:144] + data.ranges[1004:1147])
        left_distance =     min(data.ranges[144:287])
        right_distance =    min(data.ranges[861:1004])

        if (self.key == True):
            if right_distance < self.min_dist:
                self.turn_left()
            elif right_distance > self.min_dist + 0.1:
                self.turn_right()
            else:
                y_vel = -self.kp*(right_distance-0.40)
                if (y_vel > 0.15):
                    y_vel = 0.15
                if (y_vel < -0.15):
                    y_vel = -0.15
                self.move_forward(forward_distance, y_vel)
        else:
            print("Stop maze exploration")
            self.stop()

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    node_handler = NodeLIDAR()
    rospy.spin()