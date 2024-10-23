#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
import numpy as np

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)

        # Variable to store the optimal path
        self.optimal_path = None

        rospy.Subscriber('/rrt_path', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Rate at which to publish velocity commands
        self.rate = rospy.Rate(20)  # adjust as needed

        # PID controller parameters
        self.kp = 0.05  # proportional gain
        # self.ki = 0.01  # integral gain
        # self.kd = 0.1  # derivative gain

        # Variables for PID control
        self.prev_error = 0
        self.integral = 0

        # Target pose
        self.target_pose = PoseWithCovarianceStamped()

    def path_callback(self, data):
        self.optimal_path = data

        # Reset PID variables when a new path is received
        self.prev_error = 0
        self.integral = 0

        # Call the function to follow the optimal path
        self.follow_optimal_path()

    def odom_callback(self, data):
        # Update the current pose of the robot
        self.actual_pose = [data.pose.pose.position.x, data.pose.pose.position.y] #Obtains the actual postion of the bot

    def follow_optimal_path(self):
        velocity_buffer = [Twist()] * 5
        buffer_counter = 0
        if self.optimal_path is None:
            rospy.logwarn("Optimal path data is missing.")
            return

        # Loop through the optimal path and send velocity commands
        for pose_stamped in self.optimal_path.poses:
            self.target_pose = pose_stamped

            # Calculate error in the current pose
            error = self.calculate_error()

            # Update PID variables
            # self.integral += error
            # derivative = error - self.prev_error

            # Calculate velocity command using PID formula
            velocity_command = self.calculate_pid_command(error)
            # Add the velocity command to the buffer
            velocity_buffer[buffer_counter] = velocity_command
            buffer_counter = (buffer_counter + 1) % len(velocity_buffer)

            # Publish the average velocity command
            average_velocity = self.calculate_average_twist(velocity_buffer)
            # Publish the velocity command
            self.velocity_pub.publish(average_velocity)

            # # Update previous error for the next iteration
            # self.prev_error = error

            # Sleep to control the rate of publishing
            self.rate.sleep()

        # Stop the robot after reaching the end of the path
        self.velocity_pub.publish(Twist())

    def calculate_error(self):
        # Calculate error in the current pose using Euclidean distance
        error_x = self.target_pose.pose.position.x - self.actual_pose[0]
        error_y = self.target_pose.pose.position.y - self.actual_pose[1]
        # error = np.sqrt(error_x**2 + error_y**2)
        error = [error_x, error_y]
        return error
    
    def calculate_average_twist(self, twist_list):
        # Calculate the average of linear and angular components separately
        avg_linear_x = sum(twist.linear.x for twist in twist_list) / len(twist_list)
        avg_linear_y = sum(twist.linear.y for twist in twist_list) / len(twist_list)
        
        avg_angular_z = sum(twist.angular.z for twist in twist_list) / len(twist_list)

        # Create a new Twist message with the average values
        average_twist = Twist()
        average_twist.linear.x = avg_linear_x
        average_twist.linear.y = avg_linear_y
        average_twist.angular.z = avg_angular_z

        return average_twist

    def calculate_pid_command(self, error):
        # Calculate velocity command using PID formula
        velocity_command = Twist()
        velocity_command.linear.x = -self.kp * error[0] #+ self.ki * self.integral + self.kd * derivative
        velocity_command.linear.y = -self.kp * error[1]  # Adjust as needed
        
        return velocity_command

if __name__ == '__main__':
    path_follower = PathFollower()
    rospy.spin()
