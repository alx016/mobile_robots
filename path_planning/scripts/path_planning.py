#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        # Variables para almacenar el mapa y el camino del robot
        self.map_data = None
        self.robot_path = None
        self.aux = True

        # Suscripciones a los topicos del mapa y del camino del robot
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/trajectory', Path, self.path_callback)
        self.optimal_path_pub = rospy.Publisher('/optimal_trajectory', Path, queue_size=10)

    def map_callback(self, data):
        self.map_data = data

    def path_callback(self, data):
        self.robot_path = data

        # Llamar a la funcion para encontrar el camino mas optimo
        optimal_path = self.find_optimal_path()
        rospy.loginfo("Optimal path to start: {}".format(optimal_path))
        self.publish_optimal_path(optimal_path)

    def publish_optimal_path(self, optimal_path):
        # Crear un mensaje de tipo Path
        optimal_path_msg = Path()

        # Asignar el encabezado (header) del mensaje (puedes copiarlo del mensaje del robot_path)
        optimal_path_msg.header = self.robot_path.header

        # Convertir las coordenadas del camino mas corto a objetos PoseStamped y agregarlos al mensaje
        for coord in optimal_path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = coord[0]
            pose_stamped.pose.position.y = coord[1]
            # Puedes configurar otras propiedades del objeto PoseStamped segun sea necesario
            optimal_path_msg.poses.append(pose_stamped)

        # Publicar el mensaje Path
        self.optimal_path_pub.publish(optimal_path_msg)
        

    def find_optimal_path(self):
        if self.aux == True:

            self.aux == False
            # Verificar si se han recibido datos del mapa y del camino del robot
            if self.map_data is None or self.robot_path is None:
                rospy.logwarn("Map or robot path data is missing.")
                return []   

            # Obtener la matriz de ocupacion del mapa
            map_data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

            # Convertir el camino del robot a una lista de coordenadas
            path_coords = [(pose.pose.position.x, pose.pose.position.y) for pose in self.robot_path.poses]

            # Obtener la posicion actual del robot
            current_position = path_coords[-1]

            # Obtener el indice de la celda correspondiente a la posicion actual
            current_cell = self.world_to_map(current_position)

            # Obtener el indice de la celda correspondiente al punto de inicio (0,0)
            start_cell = self.world_to_map((0, 0))

            # Utilizar el algoritmo A* para encontrar el camino mas optimo
            optimal_path_indices = self.a_star(map_data, current_cell, start_cell)

            # Convertir los indices del camino optimo a coordenadas del mundo
            optimal_path_coords = [self.map_to_world(index) for index in optimal_path_indices]

            return optimal_path_coords

    def world_to_map(self, world_coords):
        # Convertir coordenadas del mundo a indices de celda del mapa
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        map_x = int((world_coords[0] - origin.x) / resolution)
        map_y = int((world_coords[1] - origin.y) / resolution)

        return (map_y, map_x)

    def map_to_world(self, map_index):
        # Convertir indices de celda del mapa a coordenadas del mundo
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        world_x = map_index[1] * resolution + origin.x
        world_y = map_index[0] * resolution + origin.y

        return (world_x, world_y)

    def a_star(self, grid, start, goal):
        # Implementar el algoritmo A*
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
    
    def heuristic(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def get_neighbors(self, node, height, width):
        # Obtener los vecinos validos de un nodo en una cuadricula
        neighbors = []
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_node = (node[0] + i, node[1] + j)
            if 0 <= new_node[0] < height and 0 <= new_node[1] < width:
                neighbors.append(new_node)
        return neighbors

if __name__ == '__main__':
    path_planner = PathPlanner()
    rospy.spin()
