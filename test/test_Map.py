import time, random
import numpy as np
from src.SimulatedField import SimulatedField

# def test_MapCreation():
#     sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))
#     assert sf._field.shape == (100, 100)

# def test_updateField():
#     sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))

#     sf.updateField(robot_ids=["0", "1", "2"], 
#                    poses=[[0, 0], [20, 0], [10, 10]])
#     assert sf._robots_dict == {"0": [0, 0], "1": [20, 0], "2": [10, 10]}

#     sf.updateField(robot_ids=["0", "1", "2"], 
#                    poses=[[0, 0], [20, 0], [80, 80]])
#     assert sf._robots_dict == {"0": [0, 0], "1": [20, 0], "2": [80, 80]}


# def test_FieldMeasurement():
#     width = 100
#     height = 100
#     hotspot_col = 50
#     hotspot_row = 50
#     sigma = 10
#     sf = SimulatedField(width=width, height=height, feature="maximum", feature_coords=(hotspot_row,hotspot_col), feature_sigma=sigma)

#     robot_state = sf.updateField(robot_ids=["0", "1", "2"], poses=[[50, 50], [50, 50], [50, 50]])
#     assert robot_state.getSensorReadingsOnly() == [1.0, 1.0, 1.0]
    
#     for _ in range(10): # adjust the range according to the number of points you want to check
#         row = random.randint(0, height-1)
#         col = random.randint(0, width-1)
#         expected_val = np.exp(-((col - hotspot_col)**2 + (row - hotspot_row)**2) / (2 * sigma**2))
#         np.testing.assert_almost_equal(sf._field[row, col], expected_val)

#         robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
#                                          poses=[[row, col], [row, col], [row, col]])
#         expected_robot_state = {"0": 
#                                     {"coords":[row, col],
#                                     "sensor_reading": expected_val
#                                     },
#                                 "1": {"coords":[row, col],
#                                     "sensor_reading": expected_val
#                                     },
#                                 "2": {"coords":[row, col],
#                                     "sensor_reading": expected_val
#                                     }
#                                 }
#         np.testing.assert_almost_equal(robot_state.getSensorReadingsOnly()[0], expected_val)
#         assert robot_state._data == expected_robot_state

# def test_GradientDirection():
#     sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))
#     robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
#                                      poses=[[0, 0], [20, 0], [10, 10]])
    
#     robot_vectors = []
#     for robot in robot_state._data.keys():
#         coords = robot_state._data[robot]["coords"]
#         robot_vectors.append([coords[0], coords[1], robot_state._data[robot]["sensor_reading"]])
#     robot_vectors = np.array(robot_vectors)

#     R_01 = robot_vectors[1] - robot_vectors[0]
#     R_02 = robot_vectors[2] - robot_vectors[0]
#     gradient_vector = -np.cross(R_01, R_02)
#     print(gradient_vector)
#     assert np.allclose(gradient_vector, sf.getGradient())

#     robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
#                                      poses=[[80, 80], [99, 80], [90, 99]])
    
#     robot_vectors = []
#     for robot in robot_state._data.keys():
#         coords = robot_state._data[robot]["coords"]
#         robot_vectors.append([coords[0], coords[1], robot_state._data[robot]["sensor_reading"]])
#     robot_vectors = np.array(robot_vectors)

#     R_01 = robot_vectors[1] - robot_vectors[0]
#     R_02 = robot_vectors[2] - robot_vectors[0]
#     gradient_vector = -np.cross(R_01, R_02)
#     print(gradient_vector)
#     assert np.allclose(gradient_vector, sf.getGradient())

def test_ClusterMovement():
    sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))
    robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
                                     poses=[[0, 0], [20, 0], [10, 10]])
    sf.runSim(steps=60)

    robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
                                     poses=[[80, 80], [99, 80], [90, 99]])
    sf.runSim(steps=60)

    robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
                                     poses=[[0, 50], [20, 50], [10, 70]])
    sf.runSim(steps=60)
    assert True
    

def test_DifferentFeatureLocations():
    sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(80,80))
    robot_state = sf.updateField(robot_ids=["0", "1", "2"], 
                                     poses=[[0, 0], [20, 0], [10, 10]])
    sf.runSim(steps=70)
