import sys
from src.SimulatedField import SimulatedField

def test_MapCreation():
    sf = SimulatedField(width=100, height=100, feature="maximum", feature_coords=(50,50))
    assert sf.field.shape == (100, 100)
    sf.initRobots(robots=["0", "1", "2"], poses=[(0, 0), (2, 0), (1, 1)])
