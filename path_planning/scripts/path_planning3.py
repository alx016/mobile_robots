#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from heapq import heappop, heappush

# Add the RRT* algorithm
def rrt_star(start, goal, map_array, iterations= 1500, delta_q=0.7, goal_tolerance=0.5):
    path = [start]
    parents = {start: None}

    for _ in range(iterations):
        q_rand = (np.random.rand() * map_array.shape[1], np.random.rand() * map_array.shape[0])

        q_near = nearest_neighbor(path, q_rand)

        q_new = new_point(q_near, q_rand, delta_q)

        if is_collision_free(q_new, map_array):
            path.append(q_new)
            parents[q_new] = q_near

        # Check if the goal is reached with tolerance
        distance_to_goal = euclidean_distance(path[-1], goal)
        if distance_to_goal < goal_tolerance:
            print("Goal Reached:", path[-1])
            break

    # Connect the last point in the path to the goal
    last_point = path[-1]
    goal_reached = new_point(last_point, goal, delta_q)

    if is_collision_free(goal_reached, map_array):
        path.append(goal_reached)
        parents[goal_reached] = last_point

    return path, parents

def backtrack_to_start(parents, start, goal):
    path = [goal]
    current = goal
    while current != start:
        current = parents[current]
        path.append(current)
    return path[::-1]

class MapSubscriber:
    def __init__(self):
        self.path = None
        self.map_data = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher('/rrt_path', Path, queue_size=10)
        self.path_generated = False

    def map_callback(self, msg):
        self.map_data = msg

    def generate_rrt_star_path(self):
        if self.map_data is not None:
            if not self.path_generated:
                goal_reached = False
                while (not goal_reached):
                    # Convert the 1D occupancy grid data to a 2D numpy array
                    map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

                    # Define start and goal nodes (you can set your own coordinates)
                    start = (20, 20)
                    goal = (60, 40)

                    # Generate RRT* path
                    path, parents = rrt_star(start, goal, map_array)

                    # Print parents dictionary for debugging
                    #print("Parents Dictionary:", parents)
                    distance_goal = euclidean_distance(path[-1], goal)
                    if (distance_goal < 0.7):
                        goal_reached = True

                    # Backtrack to get the shortest path from goal to start
                    shortest_path = backtrack_to_start(parents, start, goal)

                    # Visualize the path and the map
                    self.plot_rrt_star(map_array, path, start, goal, shortest_path)

                    # Set the flag to False to indicate that the path has been generated
                    self.path_generated = True

    def publish_path(self):
        if self.path is not None:
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = '/map'
            for point in self.path:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = '/map'
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0
                pose.pose.orientation.w = 1.0 
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)

    def plot_rrt_star(self, map_array, path, start, goal, shortest_path):
        plt.imshow(map_array, cmap='gray', origin='lower')
        plt.scatter(start[0], start[1], color='red', marker='x')
        plt.scatter(goal[0], goal[1], color='green', marker='x')

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.scatter(path_x, path_y, color='blue', linewidth=2, s=2)  # Adjusted marker size

        # Plot the shortest path in red
        shortest_path_x = [point[0] for point in shortest_path]
        shortest_path_y = [point[1] for point in shortest_path]
        plt.plot(shortest_path_x, shortest_path_y, color='red', linewidth=2)

        plt.title('RRT* Path Planning')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.savefig('/home/jetauto/Pictures/occupancy_grid_map_rrt_star.png')
        rospy.loginfo('Image saved as occupancy_grid_map_rrt_star.png')

def new_point(q_near, q_rand, delta_q):
    distance = euclidean_distance(q_near, q_rand)
    if distance < delta_q:
        return q_rand
    else:
        theta = np.arctan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0])
        new_x = q_near[0] + delta_q * np.cos(theta)
        new_y = q_near[1] + delta_q * np.sin(theta)
        return (new_x, new_y)

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def is_collision_free(point, map_array):
    x, y = int(point[0]), int(point[1])
    return 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0] and map_array[y, x] < 20

def nearest_neighbor(tree, point):
    distances = [euclidean_distance(node, point) for node in tree]
    return tree[np.argmin(distances)]

if __name__ == '__main__':
    rospy.init_node('map_subscriber', anonymous=True)
    map_subscriber = MapSubscriber()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        map_subscriber.generate_rrt_star_path()
        map_subscriber.publish_path()
        rate.sleep()
        print("Publishing")