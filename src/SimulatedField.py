import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class SimulatedField:
    
    field = None

    def __init__(self, width=1024, height=1024, feature="maximum", feature_coords=(512, 512)):
        """
        Initialization function.
        """
        self.field = self.create_heatmap(height, width, feature_coords)
        plt.imshow(self.field, 
                   cmap='hot', 
                   interpolation='nearest', 
                   origin="lower", 
                   aspect="auto")
        plt.colorbar()
        plt.xlim(auto=True)
        plt.ylim(auto=True)
        plt.scatter(width//2, height//2, color="blue")
        plt.show()

    def initRobots(self, robots, poses):
        """
        Places the robots on the heatmap.
        """
        if len(robots) < 1:
            raise Exception("At least one robot must be added to the simulation.")
        if len(robots) != len(poses):
            raise Exception("Not enough poses entered.")

    def create_heatmap(self, rows, cols, hotspot_coords, sigma=10):
        """
        Create a 2D heatmap with a hotspot at a given row and column.

        :param rows: Number of rows in the heatmap
        :param cols: Number of columns in the heatmap
        :param hotspot_row: The row index for the hotspot
        :param hotspot_col: The column index for the hotspot
        :param sigma: The standard deviation of the Gaussian, controlling the "spread" of the hotspot
        """
        # Create a grid of coordinates
        y = np.arange(rows)
        x = np.arange(cols)
        X, Y = np.meshgrid(x, y)
        hotspot_col = hotspot_coords[1]
        hotspot_row = hotspot_coords[0]


        # Create the Gaussian function centered at the hotspot
        heatmap = np.exp(-((X - hotspot_col)**2 + (Y - hotspot_row)**2) / (2 * sigma**2))

        return heatmap


        
