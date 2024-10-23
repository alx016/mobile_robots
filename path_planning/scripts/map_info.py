#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    # Accessing map information
    map_info = msg.info
    resolution = map_info.resolution
    width = map_info.width
    height = map_info.height
    origin = map_info.origin

    # Accessing map data
    data = msg.data

    rospy.loginfo("Received OccupancyGrid message:")
    rospy.loginfo(f"  Resolution: {resolution}")
    rospy.loginfo(f"  Width: {width}, Height: {height}")
    rospy.loginfo(f"  Origin: {origin}")
    rospy.loginfo(f"  First 10 data values: {data[:10]}")

def map_listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    map_listener()