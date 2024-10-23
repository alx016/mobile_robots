#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from heapq import heappop, heappush

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
            self.generate_a_star_path()  # Call A* path generation

    def generate_rrt_star_path(self):
        if self.map_data is not None:
            # Convert the 1D occupancy grid data to a 2D numpy array
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

            # Crop the map
            cropped_map_array = self.crop_map(map_array)

            # Define start and goal nodes (you can set your own coordinates)
            start = (35, 35)
            goal = (10, 10)

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

    def generate_a_star_path(self):
        if self.map_data is not None:
            # Convert the 1D occupancy grid data to a 2D numpy array
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

            # Crop the map
            cropped_map_array = self.crop_map(map_array)

            # Define start and goal nodes (you can set your own coordinates)
            start = (35, 35)
            goal = (10, 10)

            # Generate A* path
            path = a_star(start, goal, cropped_map_array)

            # Visualize the path and the map
            self.plot_a_star(cropped_map_array, path, start, goal)

            # Publish the path as a ROS message
            self.publish_path(path)

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

        cropped_map_array = np.rot90(cropped_map_array, k=3)

        return cropped_map_array

    def plot_rrt_star(self, map_array, path, start, goal):
        plt.imshow(map_array.T, cmap='gray', origin='lower')
        plt.scatter(start[0], start[1], color='red', marker='x')
        plt.scatter(goal[0], goal[1], color='green', marker='x')

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.scatter(path_x, path_y, color='blue', linewidth=2, s=2)  # Adjusted marker size
        plt.title('RRT* Path Planning')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.savefig('/home/jetauto/Pictures/occupancy_grid_map_rrt_star.png')
        rospy.loginfo('Image saved as occupancy_grid_map_rrt_star.png')

    def plot_a_star(self, map_array, path, start, goal):
        plt.imshow(map_array.T, cmap='gray', origin='lower')
        plt.scatter(start[0], start[1], color='red', marker='x')
        plt.scatter(goal[0], goal[1], color='green', marker='x')

        path_x = [point[0] for point in path]
        path_y = [point[1] for point in path]
        plt.plot(path_x, path_y, color='orange', linewidth=2)  # Adjusted line color
        plt.title('A* Path Planning')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.savefig('/home/jetauto/Pictures/occupancy_grid_map_a_star.png')
        rospy.loginfo('Image saved as occupancy_grid_map_a_star.png')

    def run(self):
        rospy.spin()

# Add the RRT* algorithm
def rrt_star(start, goal, map_array, iterations=1200, delta_q=1):
    path = [start]

    for _ in range(iterations):
        q_rand = (np.random.rand() * map_array.shape[1], np.random.rand() * map_array.shape[0])

        q_near = nearest_neighbor(path, q_rand)

        q_new = new_point(q_near, q_rand, delta_q)

        if is_collision_free(q_new, map_array):
            path.append(q_new)

        # Check if the goal is reached
        if path[-1] == goal:
            break

    # Connect the last point in the path to the goal
    last_point = path[-1]
    goal_reached = new_point(last_point, goal, delta_q)

    if is_collision_free(goal_reached, map_array):
        path.append(goal_reached)

    return path

# Add the A* algorithm
def a_star(start, goal, map_array):
    def heuristic(node):
        return np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

    open_set = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        current_cost, current_node = heappop(open_set)

        if current_node == goal:
            path = reconstruct_path(came_from, start, goal)
            return path

        for neighbor in neighbors(current_node, map_array):
            new_cost = cost_so_far[current_node] + 1  # Assuming a cost of 1 for each step

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor)
                heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current_node

    return None  # No path found

def neighbors(node, map_array):
    x, y = node
    potential_neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    return [neighbor for neighbor in potential_neighbors if is_valid(neighbor, map_array)]

def is_valid(node, map_array):
    x, y = node
    return 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0] and map_array[y, x] < 10

def reconstruct_path(came_from, start, goal):
    current_node = goal
    path = [current_node]

    while current_node != start:
        current_node = came_from[current_node]
        path.append(current_node)

    return path[::-1]

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
    return 0 <= x < map_array.shape[1] and 0 <= y < map_array.shape[0] and map_array[y, x] < 10
def nearest_neighbor(tree, point):
    distances = [euclidean_distance(node, point) for node in tree]
    return tree[np.argmin(distances)]





if __name__ == '__main__':
    map_subscriber = MapSubscriber()
    map_subscriber.run()
