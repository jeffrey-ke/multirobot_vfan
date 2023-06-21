  
class SensorReadingsStamped:
    _data = {}
    _sensorReadings = None
    
    def __init__(self, robot_ids, poses, sensor_readings):
        self._sensorReadings = sensor_readings
        for i, robot in enumerate(robot_ids):
            self._data[robot] = {
                                    "coords": poses[i],
                                    "sensor_reading": sensor_readings[i]
                                }
            
    def getSensorReadingsOnly(self):
        return self._sensorReadings