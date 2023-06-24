class Robot:
    _velocity: list = [0.0, 0.0]
    _position: list = [0.0, 0.0]
    _heading: float = 0.0
    _sensor_reading: float = 0.0
    def __init__(self) -> None:
        pass

    def command(self, velocity=[0.0, 0.0]):
        """
            Cluster Space Controller calls this function on all of
            the robots in its cluster and gives it a v
        """
        self._velocity = velocity
        