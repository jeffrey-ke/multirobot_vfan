import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class SimulatedField:
    
    _field = None
    _fig = None
    _ax = None
    _robots_dict = {}
    
    def __init__(self, width=1024, height=1024, feature="m_aximum", feature_coords=(512, 512)):
        """
        Initialization function.
        """

        plt.ion()

        self._field = self.create_heatmap(height, width, feature_coords)
        self._fig, self._ax = plt.subplots()

        im = self._ax.imshow(self._field, 
                       cmap='hot', 
                       interpolation='nearest',
                       origin="lower", 
                       aspect="auto")
        self._fig.colorbar(im)

        self._robots_dict = {
            "0": [0, 0],
            "1": [2, 0],
            "2": [1, 1]
        }
        poses = np.array(list(self._robots_dict.values()))
        scatter = self._ax.scatter(poses[:,0], 
                                   poses[:,1],
                                   color="blue")
        
        plt.pause(0.1)
        plt.show()

    def updateField(self, robots, poses):
        """
        Places the robots on the heatmap.
        """
        if len(robots) < 1:
            raise Exception("At least one robot must be added to the simulation.")
        if len(robots) != len(poses):
            raise Exception("Amount of poses provided does not match number of robots provided.")
        
        self._ax.clear()

        self._robots_dict = dict(zip(robots, poses))
        poses = np.array(list(self._robots_dict.values()))
        scatter = self._ax.scatter(poses[:,0], 
                                   poses[:,1],
                                   color="blue")
        
        im = self._ax.imshow(self._field, 
                       cmap='hot', 
                       interpolation='nearest',
                       origin="lower", 
                       aspect="auto")
        
        self._fig.canvas.draw()
        plt.pause(0.1)
        plt.show()
        


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


        
