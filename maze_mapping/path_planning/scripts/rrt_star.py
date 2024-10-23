#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy.ma as ma


class MapSubscriber:
    def __init__(self):
        rospy.init_node('map_subscriber', anonymous=True)
        self.map_data = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher('/rrt_star_path', Path, queue_size=10)
        self.path_generated = False  # Flag to track whether the path has been generated
        self.stop_generation = False  # Flag to stop further path generation if the goal is reached

    def map_callback(self, msg):
        self.map_data = msg
        if not self.path_generated and not self.stop_generation:
            self.generate_rrt_star_path()

    def generate_rrt_star_path(self):
        if self.map_data is not None:
            # Convert the 1D occupancy grid data to a 2D numpy array
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

            # Crop the map
            cropped_map_array = self.crop_map(map_array)

            # Define start and goal nodes (you can set your own coordinates)
            start = (45, 40)
            goal = (8, 20)

            # Generate RRT* path
            path = rrt_star(start, goal, cropped_map_array)

            # Visualize the path and the map
            self.plot_rrt_star(cropped_map_array, path, start, goal)

            # Publish the path as a ROS message
            self.publish_path(path)

            # Check if the goal is reached
            if path[-1] == goal:
                rospy.loginfo('Goal reached. Stopping further path generation.')
                self.stop_generation = True

            # Set the flag to True to indicate that the path has been generated
            self.path_generated = True

    def publish_path(self, path):
        # Create a Path message
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'

        # Populate the Path message with PoseStamped messages
        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # No rotation

            path_msg.poses.append(pose)

        # Publish the Path message
        self.path_pub.publish(path_msg)

    def crop_map(self, map_array):
        threshold = 50
        relevant_indices = np.where(map_array >= threshold)
        min_x, max_x = np.min(relevant_indices[1]), np.max(relevant_indices[1])
        min_y, max_y = np.min(relevant_indices[0]), np.max(relevant_indices[0])
        cropped_map_array = map_array[min_y:max_y + 1, min_x:max_x + 1]

        return cropped_map_array

    def plot_rrt_star(self, map_array, path, start, goal):
        plt.imshow(map_array, cmap='gray', origin='lower')
        plt.scatter(start[0], start[1], color='red', marker='x')
        plt.scatter(goal[0], goal[1], color='green', marker='x')

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.scatter(path_x, path_y, color='blue', linewidth=2, s=2)  # Adjusted marker size
        plt.title('RRT* Path Planning')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.savefig('/home/jetauto/Pictures/occupancy_grid_map.png')
        rospy.loginfo('image saved as occupancy_grid_map.png')

    def run(self):
        rospy.spin()

def rrt_star(start, goal, map_array, iterations=400, delta_q=1.5):
    path = [start]
    goal_threshold = 5

    for _ in range(iterations):
        q_rand = (np.random.rand() * map_array.shape[1], np.random.rand() * map_array.shape[0])

        q_near = nearest_neighbor(path, q_rand)

        q_new = new_point(q_near, q_rand, delta_q)

        if is_collision_free(q_new, map_array):
            path.append(q_new)

        # Check if the goal is reached
        if euclidean_distance(path[-1], goal) < goal_threshold:
            break

    # Connect the last point in the path to the goal
    last_point = path[-1]
    goal_reached = new_point(last_point, goal, delta_q)

    if is_collision_free(goal_reached, map_array):
        path.append(goal_reached)

    return path

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

def nearest_neighbor(tree, point):
    distances = [euclidean_distance(node, point) for node in tree]
    return tree[np.argmin(distances)]

def is_collision_free(point, map_array):
    x, y = int(point[0]), int(point[1])
    return 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0] and map_array[y, x] < 10


if __name__ == '__main__':
    map_subscriber = MapSubscriber()
    map_subscriber.run()
