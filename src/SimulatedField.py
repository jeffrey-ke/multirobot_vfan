import numpy as np
import matplotlib.pyplot as plt
from math import floor
from src.SensorReadingsStamped import SensorReadingsStamped
from src.HeatMap import createHeatmap

class SimulatedField:
    
    _field = None
    _fig = None
    _ax = None
    _robot_state = {}

    def __init__(self, width=1024, height=1024, feature="m_aximum", feature_coords=(512, 512), feature_sigma=10):
        """
        Initialization function.
        """

        plt.ion()

        self._field = createHeatmap(height, width, feature_coords, feature_sigma)
        self._fig, self._ax = plt.subplots()

        im = self._ax.imshow(self._field, 
                       cmap='hot', 
                       interpolation='nearest',
                       origin="lower", 
                       aspect="auto")
        self._fig.colorbar(im)

        plt.pause(0.1)
        plt.show()

    def runSim(self, steps=100):
        for _ in range(steps):
            robot_poses = [self._robot_state._data[robot]["coords"] for robot in self._robot_state._data.keys()]

            gradient_xy = np.array(
                [floor(component + 1 if component > 0 else component - 1) for component in self.getGradient()[:2]]
                )

            robot_poses = (robot_poses + gradient_xy).astype(int)

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
        
        sensor_readings = [self._field[x][y] for x, y in poses]
        self._robot_state = SensorReadingsStamped(robot_ids=robot_ids, poses=poses, sensor_readings=sensor_readings)

        for robot in robot_ids:
            self._ax.annotate(robot, 
                              (self._robot_state._data[robot]["coords"][0], 
                               self._robot_state._data[robot]["coords"][1]),#(x,y) coord
                              color='b') 
            
            self._ax.scatter(self._robot_state._data[robot]["coords"][0], 
                             self._robot_state._data[robot]["coords"][1],
                             color="b")
        
        self._ax.imshow(self._field, 
                       cmap='hot', 
                       interpolation='nearest',
                       origin="lower", 
                       aspect="auto")
        
        self._fig.canvas.draw()
        plt.pause(0.1)
        plt.show()

        return self._robot_state


    


    