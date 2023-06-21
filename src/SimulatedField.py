import numpy as np
import matplotlib.pyplot as plt
from math import floor
from src.SensorReadingsStamped import SensorReadingsStamped

class SimulatedField:
    
    _field = None
    _fig = None
    _ax = None
    _robots_dict = {}
    _robot_state = {}

    def __init__(self, width=1024, height=1024, feature="m_aximum", feature_coords=(512, 512), feature_sigma=10):
        """
        Initialization function.
        """

        plt.ion()

        self._field = self.create_heatmap(height, width, feature_coords, feature_sigma)
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

    def runSim(self, steps=100):
        for _ in range(steps):
            robot_poses = [self._robot_state._data[robot]["coords"] for robot in self._robot_state._data.keys()]

            gradient_xy = self.getGradient()[:2] 
            for i, component in enumerate(gradient_xy):
                gradient_xy[i] = floor(component + 1 if component > 0 else component - 1)

            for i, pose in enumerate(robot_poses):
                robot_poses[i] = (pose + gradient_xy).astype(int)

            self.updateField(robot_ids=list(self._robot_state._data.keys()),
                             poses=robot_poses)
        

    def getGradient(self):
        robot_vectors = []

        for robot in self._robot_state._data.keys():
            coords = self._robot_state._data[robot]["coords"]
            robot_vectors.append([coords[0], coords[1], self._robot_state._data[robot]["sensor_reading"]])
        robot_vectors = np.array(robot_vectors)

        R_01 = robot_vectors[1] - robot_vectors[0]
        R_02 = robot_vectors[2] - robot_vectors[0]
        
        return -np.cross(R_01, R_02)

    def updateField(self, robot_ids, poses):
        """
        Places the robots on the heatmap.
        """
        if len(robot_ids) < 1:
            raise Exception("At least one robot must be added to the simulation.")
        if len(robot_ids) != len(poses):
            raise Exception("Amount of poses provided does not match number of robots provided.")
        
        self._ax.clear()

        self._robots_dict = dict(zip(robot_ids, poses))
        
        for robot in self._robots_dict.keys():
            self._ax.annotate(robot, 
                              (self._robots_dict[robot][0], 
                               self._robots_dict[robot][1]),#(x,y) coord
                              color='b') 
            
            self._ax.scatter(self._robots_dict[robot][0], 
                             self._robots_dict[robot][1],
                             color="blue")
        
        self._ax.imshow(self._field, 
                       cmap='hot', 
                       interpolation='nearest',
                       origin="lower", 
                       aspect="auto")
        
        self._fig.canvas.draw()
        plt.pause(0.1)
        plt.show()

        sensor_readings = [self._field[x][y] for x, y in poses]
        self._robot_state = SensorReadingsStamped(robot_ids=robot_ids, poses=poses, sensor_readings=sensor_readings)
        return self._robot_state


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


    