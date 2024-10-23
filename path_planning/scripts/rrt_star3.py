#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        # Variables to store the map and the robot's path
        self.map_data = None
        self.robot_path = None

        # Subscriptions to the map and robot path topics
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/trajectory', Path, self.path_callback)
        self.optimal_path_pub = rospy.Publisher('/optimal_trajectory', Path, queue_size=10)

    def map_callback(self, data):
        self.map_data = data

    def path_callback(self, data):
        self.robot_path = data

        # Call the function to find the optimal path
        optimal_path = self.find_optimal_path()
        rospy.loginfo("Optimal path to start: {}".format(optimal_path))
        self.publish_optimal_path(optimal_path)

    def publish_optimal_path(self, optimal_path):
        # Create a message of type Path
        optimal_path_msg = Path()

        # Assign the header of the message (you can copy it from the robot_path message)
        optimal_path_msg.header = self.robot_path.header

        # Convert the coordinates of the shortest path to PoseStamped objects and add them to the message
        for coord in optimal_path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = coord[0]
            pose_stamped.pose.position.y = coord[1]
            # You can set other properties of the PoseStamped object as needed
            optimal_path_msg.poses.append(pose_stamped)

        # Publish the Path message
        self.optimal_path_pub.publish(optimal_path_msg)

    def find_optimal_path(self):
        # Check if map and robot path data have been received
        if self.map_data is None or self.robot_path is None:
            rospy.logwarn("Map or robot path data is missing.")
            return []   

        # Get the occupancy matrix of the map
        map_data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

        # Convert the robot path to a list of coordinates
        path_coords = [(pose.pose.position.x, pose.pose.position.y) for pose in self.robot_path.poses]

        # Get the current position of the robot
        current_position = path_coords[-1]

        # Get the cell index corresponding to the current position
        current_cell = self.world_to_map(current_position)

        # Get the cell index corresponding to the starting point (0,0)
        start_cell = self.world_to_map((0, 0))

        # Use the A* algorithm to find the optimal path
        optimal_path_indices = self.a_star_with_safe_distance(map_data, current_cell, start_cell)

        # Convert the indices of the optimal path to world coordinates
        optimal_path_coords = [self.map_to_world(index) for index in optimal_path_indices]

        return optimal_path_coords

    def world_to_map(self, world_coords):
        # Convert world coordinates to map cell indices
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        map_x = int((world_coords[0] - origin.x) / resolution)
        map_y = int((world_coords[1] - origin.y) / resolution)

        return (map_y, map_x)

    def map_to_world(self, map_index):
        # Convert map cell indices to world coordinates
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        world_x = map_index[1] * resolution + origin.x
        world_y = map_index[0] * resolution + origin.y

        return (world_x, world_y)

    def a_star_with_safe_distance(self, grid, start, goal):
        # Implement the A* algorithm considering the safe distance to obstacles
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current_cost, current_node = heapq.heappop(frontier)

            if current_node == goal:
                break

            for next_node in self.get_neighbors(current_node, grid.shape[0], grid.shape[1]):
                new_cost = cost_so_far[current_node] + 1  # Assuming a cost of 1 for each movement

                # Consider the distance to the nearest obstacle as part of the cost
                obstacle_distance_cost = self.distance_to_nearest_obstacle(next_node, grid)
                new_cost += obstacle_distance_cost

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current_node

        path = []
        current = goal
        while current:
            path.append(current)
            current = came_from[current]

        return path[::-1]

    def heuristic(self, a, b):
        # Heuristic for A* (Manhattan distance)
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node, height, width):
        # Get valid neighbors of a node in a grid
        neighbors = []
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_node = (node[0] + i, node[1] + j)
            if 0 <= new_node[0] < height and 0 <= new_node[1] < width:
                neighbors.append(new_node)
        return neighbors

    def distance_to_nearest_obstacle(self, node, grid):
        # Calculate the distance to the nearest obstacle using the grid representation
        # You can customize this function based on the specific information of your map and obstacles
        # In this example, the Euclidean distance to the nearest obstacle is used.

        # Get the world coordinates of the node
        node_world_coords = self.map_to_world(node)

        # Find the coordinates of obstacles in the map
        obstacle_coords = np.argwhere(grid > 0)

        # Calculate the Euclidean distance to the nearest obstacle
        distances = np.linalg.norm(obstacle_coords - np.array(node), axis=1)
        min_distance = np.min(distances)

        return min_distance

if __name__ == '__main__':
    path_planner = PathPlanner()
    rospy.spin()