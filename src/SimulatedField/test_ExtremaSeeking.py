from src.SimulatedField.SimulatedField import SimulatedField
from src.Robot.Robot import Robot
import time
import numpy as np
import pytest
import math

def test_ExtremaSeeking():
    sf = SimulatedField(width=100, height=100, blocks_per_meter=10)
    sf.spawnFeature(id="maximum", location=(50, 50))

    _0 = Robot(id="0", location=(0,0))
    _1 = Robot(id="1", location=(20, 0))
    _2 = Robot(id="2", location=(10, 10), leader=True)

    sf.spawnRobot(id="0", location=(0,0))
    sf.spawnRobot(id="1", location=(20, 0))
    sf.spawnRobot(id="2", location=(10, 10))

    for _ in range(10):
        _0.readSensor(sf._robot_state[_0._id]["sensor_reading"])
        _1.readSensor(sf._robot_state[_1._id]["sensor_reading"])
        _2.readSensor(sf._robot_state[_2._id]["sensor_reading"])

        # calculating gradient
        robot_vectors = []
        for robot in [_0, _1, _2]:
            coords = robot._pose #pose = [x, y]
            robot_vectors.append([coords[0], coords[1], robot._sensor_reading])
        robot_vectors = np.array(robot_vectors)
        R_01 = robot_vectors[1] - robot_vectors[0]
        R_02 = robot_vectors[2] - robot_vectors[0]

        R_dot = -np.cross(R_01, R_02)[:2]
        _0.giveCommand(R_dot)
        _1.giveCommand(R_dot)
        _2.giveCommand(R_dot)

        for robot in [_0, _1, _2]:
            sf.moveRobot(id=robot._id, location=tuple(robot._pose[:2]))

