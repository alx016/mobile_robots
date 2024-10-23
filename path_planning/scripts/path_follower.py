#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)

        # Publishers N Subscribers
        rospy.Subscriber('/rrt_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Rate at which to publish velocity commands
        self.rate = rospy.Rate(10) 
        
        self.optimal_path = None            # Variable to store the optimal path
        self.opt_path_receive = False       # Key variable, so that it only receives path once
        self.actual_pose = [0.0, 0.0, 0.0]  # Variable to store the actual position (x, y, theta)

        # PID Traslational Control constants  
        self.translational_kp = 4.0         # Proportional gain
        self.translational_ki = 0.015       # Integral gain
        self.translational_kd = 1.0         # Derivative gain

        # # PID Rotational Control constants
        self.rotational_kp = 0.1            #constante proporcional rotacional

        # Initialize PID controller state
        self.prev_error = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0]

    def path_callback(self, data):
        self.optimal_path = data
        
        if (not self.opt_path_receive):
            # Call the function to follow the optimal path
            self.follow_optimal_path()
            self.opt_path_receive = True

    def odom_callback(self, data):
        # Update the current pose of the robot
        self.actual_pose = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z] #Obtains the actual postion of the bot

    def follow_optimal_path(self):

        threshold_linear_error = 0.1
        threshold_rotational_error = 0.2    #radianes
        error = [0.0, 0.0, 0.0]

        if self.optimal_path is None:
            rospy.logwarn("Optimal path data is missing.")
            return

        # Loop through the optimal path and send velocity commands
        for pose_stamped in self.optimal_path.poses:
            xd = pose_stamped.pose.position.x           #desired x
            yd = pose_stamped.pose.position.y           #desired y
            thetad = self.estimate_theta(xd, yd)        #desired theta
            error = self.calculate_error(xd, yd, thetad)

            while (abs(error[0]) > threshold_linear_error and abs(error[1]) > threshold_linear_error):

                while (abs(error[2]) > threshold_rotational_error):
                    angular_velocity_command = self.calculate_rotational_velocity(error)
                    self.velocity_pub.publish(angular_velocity_command)
                    error = self.calculate_error(xd, yd, thetad)
                linear_velocity_command = self.calculate_linear_velocity(error)
                # Publish the velocity command
                self.velocity_pub.publish(linear_velocity_command)

                error = self.calculate_error(xd, yd, thetad)

                # Sleep to control the rate of publishing
                self.rate.sleep()

        # Stop the robot after reaching the end of the path
        self.velocity_pub.publish(Twist())

    def estimate_theta(self, xd, yd):
        # Estimate desired theta
        y = yd - self.actual_pose[1]
        x = xd - self.actual_pose[0]
        thetad = np.arctan2(y, x)
        return thetad

    def calculate_error(self, xd, yd, thetad):
        # Calculate error in the current pose 
        error_x = xd - self.actual_pose[0]
        error_y = yd - self.actual_pose[1]
        error_theta = self.actual_pose[2] - thetad
        if error_theta > np.pi:
            error_theta = error_theta - 2*np.pi
        elif error_theta < np.pi:
            error_theta = error_theta + 2*np.pi

        error = [error_x, error_y, error_theta]

        return error

    def calculate_linear_velocity(self, error):
        #Pasar del marco inercial al marco del body con la matriz de rotación en z
        # Calculate velocity with PID control
        velocity_command = Twist()
        self.integral[0] += error[0] * self.rate.sleep_dur.to_sec()
        self.integral[1] += error[1] * self.rate.sleep_dur.to_sec()

        velocity_command.linear.x = self.translational_kp * error[0] + self.translational_ki * self.integral[0] + self.translational_kd * (error[0] - self.prev_error[0])
        velocity_command.linear.y = self.translational_kp * error[1] + self.translational_ki * self.integral[1] + self.translational_kd * (error[1] - self.prev_error[1])      

        # Update previous error for the next iteration
        self.prev_error = error

        return velocity_command
    
    def calculate_rotational_velocity(self, error):
        v_max_angular = 0.3
        velocity_command = Twist()
        velocity_command.angular.z =  -v_max_angular * np.tanh(self.rotational_kp*error[2]/v_max_angular)
        return velocity_command

if __name__ == '__main__':
    path_follower = PathFollower()
    rospy.spin()