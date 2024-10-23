#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy.ma as ma  # Import masked array module

class MapSubscriber:
    def __init__(self):
        rospy.init_node('map_subscriber', anonymous=True)
        self.map_data = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.map_data = msg
        self.save_map_image()

    def save_map_image(self):
        if self.map_data is not None:
                        # Convert the 1D occupancy grid data to a 2D numpy array
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

            # Crop the image to the region containing known information
            cropped_map_array = self.crop_map(map_array)

            # Create a masked array, masking NaN values
            masked_map_array = ma.masked_where(cropped_map_array == -1, cropped_map_array)

            # Create a figure and axis
            fig, ax = plt.subplots()

            # Display the occupancy grid map
            cmap = plt.cm.get_cmap('viridis', 256)  # You can choose a different colormap here
            im = ax.imshow(masked_map_array, cmap=cmap, origin='lower', interpolation='none', vmin=0, vmax=100)

            # Add colorbar for reference
            cbar = plt.colorbar(im, ax=ax, ticks=[0, 100])
            cbar.set_label('Occupancy')

            # Set labels and title
            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_title('Occupancy Grid Map (Relevant Information Only)')

            # Save the map image
            plt.savefig('/home/jetauto/Pictures/occupancy_grid_map.png')
            rospy.loginfo('Occupancy grid map image saved as occupancy_grid_map.png')

            # Optionally, display the map using plt.show()
            # plt.show()

            # Close the figure
            plt.close()

    def crop_map(self, map_array):
        # Define a threshold for considering a cell as relevant (occupied)
        threshold = 50  # You can adjust this threshold based on your needs

        # Find indices of relevant cells
        relevant_indices = np.where(map_array >= threshold)

        # Determine the bounding box of relevant cells
        min_x, max_x = np.min(relevant_indices[1]), np.max(relevant_indices[1])
        min_y, max_y = np.min(relevant_indices[0]), np.max(relevant_indices[0])

        # Crop the image to the bounding box of relevant cells
        cropped_map_array = map_array[min_y:max_y + 1, min_x:max_x + 1]

        return cropped_map_array

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    map_subscriber = MapSubscriber()
    map_subscriber.run()
