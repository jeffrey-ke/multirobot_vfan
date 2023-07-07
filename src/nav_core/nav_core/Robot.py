import math
class Robot:

    _id = ""
    _pose: "list[float]" = [0.0, 0.0]
    _speed = 5
    _sensor_reading: float = 0.0
    _is_leader = False

    def __init__(self, id, location, leader=False):
        self._id = id
        self._pose = list(location)
        self._is_leader = leader

    def readSensor(self, reading: float):
        self._sensor_reading = reading

    def giveCommand(self, R_dot: list):
        """_summary_

        Args:
            R_dot (list): R_dot = [x_dot, y_dot]
        """
        theta = math.atan2(R_dot[1], R_dot[0])
        x,y = self._speed * math.cos(theta), self._speed * math.sin(theta)
        self._pose[0], self._pose[1] = self._pose[0] + x, self._pose[1] + y

        