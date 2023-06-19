import time, random
import numpy as np
from src.SimulatedField import SimulatedField

def test_MapCreation():
    sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))
    assert sf._field.shape == (100, 100)

def test_updateField():
    sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))

    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [10, 10]])
    assert sf._robots_dict == {"0": [0, 0], "1": [20, 0], "2": [10, 10]}

    sf.updateField(robots=["0", "1", "2"], poses=[[0, 0], [20, 0], [80, 80]])
    assert sf._robots_dict == {"0": [0, 0], "1": [20, 0], "2": [80, 80]}

    time.sleep(5)

def test_FieldMeasurement():
    width = 100
    height = 100
    hotspot_col = 50
    hotspot_row = 50
    sigma = 10
    sf = SimulatedField(width=width, height=height, feature="maximum", feature_coords=(hotspot_row,hotspot_col), feature_sigma=sigma)

    sensor_readings = sf.updateField(robots=["0", "1", "2"], poses=[[50, 50], [50, 50], [50, 50]])
    assert sensor_readings == [1.0, 1.0, 1.0]
    
    for _ in range(10): # adjust the range according to the number of points you want to check
        row = random.randint(0, height-1)
        col = random.randint(0, width-1)
        expected_val = np.exp(-((col - hotspot_col)**2 + (row - hotspot_row)**2) / (2 * sigma**2))
        np.testing.assert_almost_equal(sf._field[row, col], expected_val)

        sensor_readings = sf.updateField(robots=["0", "1", "2"], poses=[[row, col], [row, col], [row, col]])
        np.testing.assert_almost_equal(sensor_readings[0], expected_val)



